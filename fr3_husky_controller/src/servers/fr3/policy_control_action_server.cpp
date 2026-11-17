#include <fr3_husky_controller/servers/fr3/policy_control_action_server.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace fr3_husky_controller::servers::fr3
{

namespace
{
FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* updater = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!updater)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *updater;
}

const char* robotModeName(franka::RobotMode mode)
{
    switch (mode)
    {
        case franka::RobotMode::kOther: return "Other";
        case franka::RobotMode::kIdle: return "Idle";
        case franka::RobotMode::kMove: return "Move";
        case franka::RobotMode::kGuiding: return "Guiding";
        case franka::RobotMode::kReflex: return "Reflex";
        case franka::RobotMode::kUserStopped: return "UserStopped";
        case franka::RobotMode::kAutomaticErrorRecovery: return "AutomaticErrorRecovery";
    }
    return "unknown";
}
}  // namespace

PolicyControl::PolicyControl(
    const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    command_buffer_.writeFromNonRT(CommandData{});
    command_sub_ = node_->create_subscription<fr3_husky_msgs::msg::PolicyJointCommand>(
        "policy_joint_command",
        rclcpp::QoS(1).best_effort(),
        std::bind(&PolicyControl::commandCallback, this, std::placeholders::_1));
    // 1 kHz position-mode diagnostics (layout in publishPositionDebug); record it in the bag.
    debug_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        node_->create_publisher<std_msgs::msg::Float64MultiArray>("~/policy_position_debug", rclcpp::QoS(100)));

    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] PolicyControl created; command topic: policy_joint_command",
        name_.c_str());
}

int PolicyControl::resolveRobotIndex(const std::string& robot_name) const
{
    for (size_t i = 0; i < model_updater_.robot_names_.size(); ++i)
    {
        if (model_updater_.robot_names_[i] == robot_name)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool PolicyControl::acceptGoal(const ActionT::Goal& goal)
{
    const bool dual = goal.robot_name == "dual";
    if (dual && (model_updater_.robot_names_.size() != 2 ||
                 model_updater_.manipulator_dof_ != 2 * FR3_DOF))
    {
        RCLCPP_WARN(
            node_->get_logger(), "[%s] robot_name 'dual' requires exactly two FR3 arms",
            name_.c_str());
        return false;
    }
    if (!dual && resolveRobotIndex(goal.robot_name) < 0)
    {
        RCLCPP_WARN(
            node_->get_logger(), "[%s] Unknown robot_name '%s'",
            name_.c_str(), goal.robot_name.c_str());
        return false;
    }
    if (dual && goal.control_gripper)
    {
        RCLCPP_WARN(
            node_->get_logger(), "[%s] dual policy control does not support grippers",
            name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.command_timeout_s) ||
        goal.command_timeout_s <= 0.0 || goal.command_timeout_s > 1.0)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] command_timeout_s must be in (0, 1]", name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.max_duration_s) ||
        goal.max_duration_s < 0.0 || goal.max_duration_s > 300.0)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] max_duration_s must be in [0, 300]", name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.max_policy_target_delta_rad) ||
        goal.max_policy_target_delta_rad <= 0.0 ||
        goal.max_policy_target_delta_rad > 1.0)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] max_policy_target_delta_rad must be in (0, 1]",
            name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.max_actuator_step_rad) ||
        goal.max_actuator_step_rad <= 0.0 || goal.max_actuator_step_rad > 0.01)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] max_actuator_step_rad must be in (0, 0.01]",
            name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.joint_velocity_scale) ||
        goal.joint_velocity_scale <= 0.0 || goal.joint_velocity_scale > 1.0)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] joint_velocity_scale must be in (0, 1]",
            name_.c_str());
        return false;
    }
    if (!std::isfinite(goal.joint_acceleration_scale) ||
        goal.joint_acceleration_scale <= 0.0 || goal.joint_acceleration_scale > 1.0)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] joint_acceleration_scale must be in (0, 1]",
            name_.c_str());
        return false;
    }
    if (goal.isaac_relative_control)
    {
        // Every arm of the active controller must be commanded, otherwise the
        // other arm would get zero torque (effort) or no target (position).
        const size_t goal_dof = dual ? 2 * FR3_DOF : FR3_DOF;
        if (goal_dof != model_updater_.manipulator_dof_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] Isaac-relative control must command every arm of the active controller "
                "(robot_name 'dual' for a dual-arm controller)",
                name_.c_str());
            return false;
        }
        if (model_updater_.HasEffortCommandInterface())
        {
            // Isaac PD torque path (TorqueRateLimitedPDActuator).
        }
        else if (model_updater_.HasPositionCommandInterface())
        {
            std::string error;
            if (!readPositionTrackerParams(error))
            {
                RCLCPP_WARN(node_->get_logger(), "[%s] %s", name_.c_str(), error.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] Isaac-relative control requires an effort or position command interface",
                name_.c_str());
            return false;
        }
    }
    return true;
}

bool PolicyControl::readPositionTrackerParams(std::string& error)
{
    utils::JointCommandTrackerParams params;
    const auto read = [this](const char* name, double default_value) {
        if (!node_->has_parameter(name))
        {
            node_->declare_parameter(name, default_value);
        }
        return node_->get_parameter(name).get_value<double>();
    };
    const auto read_bool = [this](const char* name, bool default_value) {
        if (!node_->has_parameter(name))
        {
            node_->declare_parameter(name, default_value);
        }
        return node_->get_parameter(name).get_value<bool>();
    };
    bool sync = true;
    bool debug = true;
    try
    {
        params.wp = read(kTrackerWpParam, params.wp);
        params.limit_fraction = read(kTrackerLimitFractionParam, params.limit_fraction);
        params.brake_fraction = read(kTrackerBrakeFractionParam, params.brake_fraction);
        params.interpolate = read_bool(kTrackerInterpolateParam, params.interpolate);
        params.interpolation_period = read(kTrackerInterpolationPeriodParam, params.interpolation_period);
        sync = read_bool(kTrackerSyncParam, sync);
        debug = read_bool(kTrackerDebugParam, debug);
    }
    catch (const std::exception& e)
    {
        error = std::string("position tracker parameter error (floats for numbers, true/false for flags): ") + e.what();
        return false;
    }
    if (!std::isfinite(params.wp) || params.wp < 5.0 || params.wp > 200.0)
    {
        error = "policy_position_tracker.wp must be in [5, 200] 1/s";
        return false;
    }
    if (!std::isfinite(params.limit_fraction) || params.limit_fraction <= 0.0 || params.limit_fraction > 0.95)
    {
        error = "policy_position_tracker.limit_fraction must be in (0, 0.95]";
        return false;
    }
    if (!std::isfinite(params.brake_fraction) || params.brake_fraction <= 0.0 || params.brake_fraction > 0.95)
    {
        error = "policy_position_tracker.brake_fraction must be in (0, 0.95]";
        return false;
    }
    if (!std::isfinite(params.interpolation_period) ||
        params.interpolation_period < 0.005 || params.interpolation_period > 0.2)
    {
        error = "policy_position_tracker.interpolation_period must be in [0.005, 0.2] s";
        return false;
    }
    position_tracker_params_ = params;
    position_sync_ = sync;
    position_debug_ = debug;
    return true;
}

void PolicyControl::onGoalAccepted(const ActionT::Goal& goal)
{
    controlled_robot_ = goal.robot_name;
    controlled_dual_ = goal.robot_name == "dual";
    controlled_robot_index_ = controlled_dual_ ? 0 : resolveRobotIndex(goal.robot_name);
    controlled_dof_ = controlled_dual_ ? 2 * FR3_DOF : FR3_DOF;
    command_timeout_s_ = goal.command_timeout_s;
    max_duration_s_ = goal.max_duration_s;
    max_policy_target_delta_rad_ = goal.max_policy_target_delta_rad;
    max_actuator_step_rad_ = goal.max_actuator_step_rad;
    joint_velocity_scale_ = goal.joint_velocity_scale;
    joint_acceleration_scale_ = goal.joint_acceleration_scale;
    isaac_relative_control_ = goal.isaac_relative_control;
    refresh_isaac_relative_target_ = true;
    control_gripper_ = goal.control_gripper;
    position_mode_ = isaac_relative_control_ &&
                     !model_updater_.HasEffortCommandInterface() &&
                     model_updater_.HasPositionCommandInterface();
}

void PolicyControl::onStart()
{
    fr3_model_updater_.setInitFromCurrent();
    q_hold_ = fr3_model_updater_.q_total_;
    q_target_ = q_hold_;
    q_desired_ = q_hold_;
    qdot_desired_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    q_offset_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    previous_applied_effort_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    activation_time_s_ = node_->now().seconds();
    last_command_time_s_ = activation_time_s_;
    last_sequence_ = 0;
    has_command_ = false;
    last_gripper_state_ = -1;
    result_message_ = "running";
    position_stopping_ = false;
    position_new_target_ = false;

    // Real robot state per arm (q_d sync, reflex stop).  MuJoCo has none.
    const size_t num_arms = model_updater_.robot_names_.size();
    robot_states_.assign(num_arms, nullptr);
    robot_state_available_ = num_arms > 0 && num_arms * FR3_DOF == model_updater_.manipulator_dof_;
    for (size_t r = 0; r < num_arms; ++r)
    {
        robot_states_[r] = model_updater_.getFrankaRobotState(r);
        robot_state_available_ = robot_state_available_ && robot_states_[r] != nullptr;
    }
    robot_time_valid_ = false;
    robot_interrupted_ = false;
    last_robot_time_ms_.assign(num_arms, 0);
    motion_error_seen_.assign(num_arms, false);
    if (robot_state_available_)
    {
        // Errors already latched before this goal are not this goal's reflex.
        for (size_t r = 0; r < num_arms; ++r)
        {
            motion_error_seen_[r] = static_cast<bool>(robot_states_[r]->last_motion_errors);
        }
    }
    robot_q_d_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    robot_dq_d_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    robot_ddq_d_ = Eigen::VectorXd::Zero(model_updater_.manipulator_dof_);
    resync_count_ = 0;
    resync_run_ = 0;
    max_resync_run_ = 0;
    debug_cycle_ = 0;

    if (position_mode_)
    {
        // Continue from the position the robot is commanded to now (the halt
        // command), not the measured one: any step in q_d is a reflex.
        Eigen::VectorXd q_start = fr3_model_updater_.q_total_;
        if (!fr3_model_updater_.readPositionCommand(q_start))
        {
            q_start = fr3_model_updater_.q_total_;
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] no valid held position command; tracker starts at the measured position",
                name_.c_str());
        }
        if (position_sync_ && readRobotCommandState())
        {
            // Start from what the robot executes (normally equal to the held command).
            q_start = robot_q_d_;
        }
        position_tracker_.setParams(position_tracker_params_);
        position_tracker_.reset(q_start);
        q_hold_ = q_start;
        q_target_ = q_start;
        q_desired_ = q_start;
    }

    const char* mode = !isaac_relative_control_ ? "rate-limited-absolute"
                       : position_mode_          ? "isaac-position-tracker"
                                                 : "isaac-pd";
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] started robot=%s timeout=%.3fs max_duration=%.3fs "
        "policy_delta=%.3frad actuator_step=%.4frad velocity_scale=%.2f "
        "acceleration_scale=%.2f mode=%s reference=command-selected gripper=%s",
        name_.c_str(), controlled_robot_.c_str(), command_timeout_s_, max_duration_s_,
        max_policy_target_delta_rad_, max_actuator_step_rad_, joint_velocity_scale_,
        joint_acceleration_scale_, mode, control_gripper_ ? "true" : "false");
    if (position_mode_)
    {
        const std::string reference = position_tracker_params_.interpolate
            ? "linear " + std::to_string(std::lround(1000.0 * position_tracker_params_.interpolation_period)) + " ms"
            : std::string("hold");
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] position tracker wp=%.1f/s limit_fraction=%.2f brake_fraction=%.2f "
            "reference=%s sync_to_robot_q_d=%s debug_topic=%s",
            name_.c_str(), position_tracker_params_.wp,
            position_tracker_params_.limit_fraction, position_tracker_params_.brake_fraction,
            reference.c_str(),
            !position_sync_ ? "off" : robot_state_available_ ? "on" : "unavailable (sim)",
            position_debug_ ? "on" : "off");
    }
    RCLCPP_INFO(
        node_->get_logger(), "[%s] robot state monitor (reflex / control restart stops the policy): %s",
        name_.c_str(), robot_state_available_ ? "on" : "unavailable (sim)");
}

void PolicyControl::commandCallback(
    const fr3_husky_msgs::msg::PolicyJointCommand::SharedPtr msg)
{
    CommandData command;
    command.dual = msg->robot_name == "dual";
    command.robot_index = command.dual ? 0 : resolveRobotIndex(msg->robot_name);
    command.sequence = msg->sequence;
    command.gripper_action = msg->gripper_action;
    command.relative_position_offsets = msg->relative_position_offsets;
    command.received_time_s = node_->now().seconds();
    command.target_count = msg->target_positions.size();

    const size_t expected_count = command.dual ? 2 * FR3_DOF : FR3_DOF;
    command.valid = command.robot_index >= 0 &&
                    command.target_count == expected_count &&
                    command.target_count <= command.target_positions.size() &&
                    std::isfinite(command.gripper_action);

    for (size_t i = 0;
         i < std::min(command.target_count, command.target_positions.size()); ++i)
    {
        command.target_positions[i] = msg->target_positions[i];
        command.valid = command.valid && std::isfinite(command.target_positions[i]);
    }
    command_buffer_.writeFromNonRT(command);
}

bool PolicyControl::validateTarget(const CommandData& command, std::string& error) const
{
    if (!command.valid)
    {
        error = "invalid policy command dimension or non-finite value";
        return false;
    }
    if (command.robot_index != controlled_robot_index_ ||
        command.dual != controlled_dual_ ||
        command.target_count != controlled_dof_)
    {
        error = "policy command dimension/robot_name does not match active goal";
        return false;
    }
    if (command.relative_position_offsets && !isaac_relative_control_)
    {
        error = "relative policy command requires Isaac control mode";
        return false;
    }
    if (command.relative_position_offsets && position_mode_)
    {
        // measured + offset refreshed every cycle would drift with the tracked position.
        error = "position command interface needs policy-step absolute targets, not relative offsets";
        return false;
    }

    const Eigen::Index offset = static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
    for (size_t i = 0; i < controlled_dof_; ++i)
    {
        const double requested = command.target_positions[i];
        const double measured =
            fr3_model_updater_.q_total_(offset + static_cast<Eigen::Index>(i));
        const double target = command.relative_position_offsets
            ? measured + requested
            : requested;
        const size_t joint_index = i % FR3_DOF;
        // Relative Isaac-style commands are measured-relative offsets.  Their
        // refreshed target is projected into the safe band below; rejecting
        // them here would abort the whole policy as soon as a joint reaches a
        // limit.  Keep strict validation for absolute commands.
        if (!command.relative_position_offsets &&
            (target < kLowerLimits[joint_index] + kJointLimitMargin ||
             target > kUpperLimits[joint_index] - kJointLimitMargin))
        {
            error = "joint" + std::to_string(i + 1) + " target violates position limit margin";
            return false;
        }
        const double step = std::abs(target - measured);
        if (step > max_policy_target_delta_rad_)
        {
            error = "joint" + std::to_string(i + 1) +
                    " policy target delta exceeds max_policy_target_delta_rad";
            return false;
        }
    }
    return true;
}

void PolicyControl::updateRateLimitedTarget(double period_s)
{
    if (!std::isfinite(period_s) || period_s <= 0.0)
    {
        period_s = kNominalControlPeriodS;
    }
    period_s = std::min(period_s, kMaxLimiterPeriodS);

    const Eigen::Index offset =
        static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
    for (size_t i = 0; i < controlled_dof_; ++i)
    {
        const Eigen::Index index = offset + static_cast<Eigen::Index>(i);
        const size_t joint_index = i % FR3_DOF;
        const double max_velocity = std::min(
            kMaxJointVelocities[joint_index] * joint_velocity_scale_,
            max_actuator_step_rad_ / period_s);
        const double max_acceleration =
            kMaxJointAccelerations[joint_index] * joint_acceleration_scale_;
        const double error = q_target_(index) - q_desired_(index);
        const double current_velocity = qdot_desired_(index);

        if (std::abs(error) <= kPositionTolerance &&
            std::abs(current_velocity) <= max_acceleration * period_s)
        {
            q_desired_(index) = q_target_(index);
            qdot_desired_(index) = 0.0;
            continue;
        }

        const double braking_velocity =
            std::sqrt(2.0 * max_acceleration * std::abs(error));
        const double target_velocity = std::copysign(
            std::min(max_velocity, braking_velocity), error);
        const double velocity_delta = std::clamp(
            target_velocity - current_velocity,
            -max_acceleration * period_s,
            max_acceleration * period_s);
        const double next_velocity = std::clamp(
            current_velocity + velocity_delta, -max_velocity, max_velocity);
        const double max_step = std::min(
            max_actuator_step_rad_, max_velocity * period_s);
        const double position_step = std::clamp(
            next_velocity * period_s, -max_step, max_step);

        if (position_step * error > 0.0 &&
            std::abs(position_step) >= std::abs(error))
        {
            q_desired_(index) = q_target_(index);
            qdot_desired_(index) = 0.0;
        }
        else
        {
            q_desired_(index) += position_step;
            qdot_desired_(index) = position_step / period_s;
        }
    }
}

void PolicyControl::refreshIsaacRelativeTarget()
{
    const Eigen::Index offset =
        static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
    for (size_t i = 0; i < controlled_dof_; ++i)
    {
        const Eigen::Index index = offset + static_cast<Eigen::Index>(i);
        const size_t joint_index = i % FR3_DOF;
        const double raw_target =
            fr3_model_updater_.q_total_(index) + q_offset_(index);
        const double target = std::clamp(
            raw_target,
            kLowerLimits[joint_index] + kJointLimitMargin,
            kUpperLimits[joint_index] - kJointLimitMargin);
        q_target_(index) = target;
        // Match Isaac Lab RelativeJointPositionAction: the latest measured-relative
        // target goes directly into the PD actuator without a trajectory limiter.
        q_desired_(index) = target;
        qdot_desired_(index) = 0.0;
    }
}

void PolicyControl::writeIsaacEffortCommand(double period_s)
{
    fr3_model_updater_.torque_desired_total_.setZero();
    if (!std::isfinite(period_s) || period_s <= 0.0)
    {
        period_s = kNominalControlPeriodS;
    }
    // A late controller cycle must not relax the 1 Nm/update safety bound.
    period_s = std::min(period_s, kMaxLimiterPeriodS);

    const Eigen::Index offset =
        static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
    for (size_t i = 0; i < controlled_dof_; ++i)
    {
        const Eigen::Index index = offset + static_cast<Eigen::Index>(i);
        const size_t joint_index = i % FR3_DOF;
        const double effort =
            kIsaacStiffness[joint_index] *
                (q_desired_(index) - fr3_model_updater_.q_total_(index)) +
            kIsaacDamping[joint_index] *
                (qdot_desired_(index) - fr3_model_updater_.qdot_total_(index));
        const double magnitude_limited_effort = std::clamp(
            effort,
            -kIsaacEffortLimits[joint_index],
            kIsaacEffortLimits[joint_index]);
        // Isaac Lab reach v2 applies torque magnitude limiting followed by a
        // 1000 Nm/s torque-rate limiter.  Apply the same continuous-time
        // limit here; at the normal 1 kHz controller period this is 1 Nm per update.
        const double max_effort_step =
            kIsaacTorqueRateLimit * period_s;
        const double previous = previous_applied_effort_(index);
        const double applied_effort = previous + std::clamp(
            magnitude_limited_effort - previous,
            -max_effort_step,
            max_effort_step);
        previous_applied_effort_(index) = applied_effort;
        fr3_model_updater_.torque_desired_total_(index) = applied_effort;
    }
    // The Franka hardware layer provides gravity compensation. This matches
    // the training asset, where robot-link gravity is disabled.
    fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_);
}

void PolicyControl::writeTrackedPositionCommand()
{
    // Re-base on the command the robot executed (normally identical to our previous
    // command, so a no-op).  If the robot stops executing commands, consecutive
    // re-syncs keep the command one limited step from q_d.
    bool resynced = false;
    if (robot_state_available_ && readRobotCommandState() && position_sync_)
    {
        resynced = position_tracker_.syncToRobot(
            robot_q_d_, robot_dq_d_, robot_ddq_d_, kSyncPositionTolerance);
    }
    if (resynced)
    {
        ++resync_count_;
        ++resync_run_;
        if (resync_run_ == kFrozenWarnCycles)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] robot q_d has not followed the position command for %lu ms "
                "(robot not executing commands); command held at q_d",
                name_.c_str(), static_cast<unsigned long>(resync_run_));
        }
    }
    else if (resync_run_ > 0)
    {
        if (resync_run_ >= kFrozenWarnCycles)
        {
            RCLCPP_WARN(
                node_->get_logger(), "[%s] robot q_d follows the command again after %lu ms",
                name_.c_str(), static_cast<unsigned long>(resync_run_));
        }
        max_resync_run_ = std::max(max_resync_run_, resync_run_);
        resync_run_ = 0;
    }

    // Fixed 1 ms step: the robot evaluates q_d derivatives per 1 kHz packet, so
    // a late controller cycle must not enlarge the step.
    position_tracker_.step();
    q_desired_ = position_tracker_.position();
    qdot_desired_ = position_tracker_.velocity();
    fr3_model_updater_.q_desired_total_ = q_desired_;
    fr3_model_updater_.writeCommand(fr3_model_updater_.q_desired_total_);
    publishPositionDebug(resynced);
    position_new_target_ = false;
}

bool PolicyControl::readRobotCommandState()
{
    if (!robot_state_available_)
    {
        return false;
    }
    for (size_t r = 0; r < robot_states_.size(); ++r)
    {
        const franka::RobotState& state = *robot_states_[r];
        for (size_t j = 0; j < FR3_DOF; ++j)
        {
            const Eigen::Index index = static_cast<Eigen::Index>(r * FR3_DOF + j);
            robot_q_d_(index) = state.q_d[j];
            robot_dq_d_(index) = state.dq_d[j];
            robot_ddq_d_(index) = state.ddq_d[j];
        }
    }
    return true;
}

bool PolicyControl::robotControlInterrupted(std::string& reason)
{
    if (!robot_state_available_)
    {
        return false;
    }
    for (size_t r = 0; r < robot_states_.size(); ++r)
    {
        const franka::RobotState& state = *robot_states_[r];
        const uint64_t time_ms = state.time.toMSec();
        const bool motion_error = static_cast<bool>(state.last_motion_errors);
        const std::string& arm = model_updater_.robot_names_[r];
        if (robot_time_valid_ && time_ms > last_robot_time_ms_[r] + kRobotTimeGapMs)
        {
            reason = arm + ": robot time jumped " +
                     std::to_string(time_ms - last_robot_time_ms_[r]) + " ms";
        }
        else if (state.robot_mode != franka::RobotMode::kMove)
        {
            reason = arm + ": robot mode " + robotModeName(state.robot_mode);
        }
        else if (static_cast<bool>(state.current_errors))
        {
            reason = arm + ": current errors " + static_cast<std::string>(state.current_errors);
        }
        else if (motion_error && !motion_error_seen_[r])
        {
            reason = arm + ": new last motion errors";
        }
        last_robot_time_ms_[r] = time_ms;
        motion_error_seen_[r] = motion_error;
        if (!reason.empty())
        {
            if (motion_error)
            {
                reason += ", last motion errors " + static_cast<std::string>(state.last_motion_errors);
            }
            return true;
        }
    }
    robot_time_valid_ = true;
    return false;
}

void PolicyControl::publishPositionDebug(bool resynced)
{
    // Layout per sample (n = manipulator dof, m = arms):
    //   cycle, ros_time_s, period_s, policy_sequence, flags,
    //   robot_time_s[m], command_success_rate[m],
    //   q_cmd[n] (written), q_d[n], dq_d[n] (robot, read this cycle before the sync),
    //   q_ref[n] (tracker reference), q_target[n] (latest policy target), q[n] (measured)
    // flags: 1 re-synced to q_d, 2 acceleration clamped, 4 velocity clamped,
    //        8 new policy target this cycle, 16 braking to stop.
    // NaN where the robot state is unavailable (MuJoCo).
    if (!position_debug_ || !debug_pub_)
    {
        return;
    }
    const uint64_t cycle = debug_cycle_++;
    if (!debug_pub_->trylock())
    {
        return;  // publisher busy: this sample is dropped (cycle counter shows the gap)
    }
    const size_t n = static_cast<size_t>(q_desired_.size());
    const size_t m = robot_states_.size();
    const size_t header = 5 + 2 * m;
    auto& msg = debug_pub_->msg_;
    if (msg.data.size() != header + 6 * n)
    {
        msg.data.resize(header + 6 * n);
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label =
            "cycle,ros_time_s,period_s,policy_sequence,flags,robot_time_s[m],command_success_rate[m],"
            "q_cmd[n],q_d[n],dq_d[n],q_ref[n],q_target[n],q[n]; m=" + std::to_string(m) +
            " n=" + std::to_string(n);
        msg.layout.dim[0].size = static_cast<uint32_t>(header + 6 * n);
        msg.layout.dim[0].stride = static_cast<uint32_t>(header + 6 * n);
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const int flags = (resynced ? 1 : 0) |
                      (position_tracker_.accelerationClamped() ? 2 : 0) |
                      (position_tracker_.velocityClamped() ? 4 : 0) |
                      (position_new_target_ ? 8 : 0) |
                      (position_stopping_ ? 16 : 0);
    auto* d = msg.data.data();
    d[0] = static_cast<double>(cycle);
    d[1] = cycle_time_s_;
    d[2] = cycle_period_s_;
    d[3] = static_cast<double>(last_sequence_);
    d[4] = static_cast<double>(flags);
    for (size_t r = 0; r < m; ++r)
    {
        const franka::RobotState* state = robot_state_available_ ? robot_states_[r] : nullptr;
        d[5 + r] = state ? state->time.toSec() : nan;
        d[5 + m + r] = state ? state->control_command_success_rate : nan;
    }
    const auto& q_ref = position_tracker_.reference();
    const auto& q_meas = fr3_model_updater_.q_total_;
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Index k = static_cast<Eigen::Index>(i);
        d[header + i] = q_desired_(k);
        d[header + n + i] = robot_state_available_ ? robot_q_d_(k) : nan;
        d[header + 2 * n + i] = robot_state_available_ ? robot_dq_d_(k) : nan;
        d[header + 3 * n + i] = k < q_ref.size() ? q_ref(k) : nan;
        d[header + 4 * n + i] = k < q_target_.size() ? q_target_(k) : nan;
        d[header + 5 * n + i] = k < q_meas.size() ? q_meas(k) : nan;
    }
    debug_pub_->unlockAndPublish();
}

PolicyControl::ComputeResult PolicyControl::finishOrBrake(ComputeResult result, double now_s)
{
    if (!position_mode_)
    {
        return result;
    }
    position_stopping_ = true;
    position_stop_result_ = result;
    position_stop_start_s_ = now_s;
    position_stop_target_ = position_tracker_.stoppingTarget();
    position_tracker_.holdTarget(position_stop_target_);
    RCLCPP_INFO(
        node_->get_logger(), "[%s] braking position command to rest before stopping (%s)",
        name_.c_str(), result_message_.c_str());
    return continuePositionStop(now_s);
}

PolicyControl::ComputeResult PolicyControl::continuePositionStop(double now_s)
{
    writeTrackedPositionCommand();
    const bool at_rest =
        position_tracker_.isAtRest(kPositionRestVelocity, kPositionRestAcceleration);
    if (at_rest || now_s - position_stop_start_s_ >= kPositionStopTimeoutS)
    {
        if (!at_rest)
        {
            RCLCPP_WARN(
                node_->get_logger(), "[%s] position brake timeout; max |v| %.4f rad/s",
                name_.c_str(), position_tracker_.velocity().cwiseAbs().maxCoeff());
        }
        return position_stop_result_;
    }
    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->command_age_s = now_s - last_command_time_s_;
    feedback->max_joint_error =
        (q_desired_ - fr3_model_updater_.q_total_).cwiseAbs().maxCoeff();
    feedback->last_sequence = last_sequence_;
    feedback->status = "stopping: " + result_message_;
    publishFeedback(feedback);
    return ComputeResult::RUNNING;
}

void PolicyControl::writeDesiredCommand(
    const Eigen::VectorXd& q_desired, const Eigen::VectorXd& qdot_desired)
{
    if (model_updater_.HasEffortCommandInterface())
    {
        if (!fr3_model_updater_.robot_controller_)
        {
            throw std::runtime_error("robot_controller is null");
        }
        const Eigen::VectorXd torque =
            fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                q_desired, qdot_desired, false);
        fr3_model_updater_.torque_desired_total_ = torque - fr3_model_updater_.g_total_;
        fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_);
    }
    else if (model_updater_.HasVelocityCommandInterface())
    {
        fr3_model_updater_.qdot_desired_total_ = qdot_desired;
        fr3_model_updater_.writeCommand(fr3_model_updater_.qdot_desired_total_);
    }
    else if (model_updater_.HasPositionCommandInterface())
    {
        fr3_model_updater_.q_desired_total_ = q_desired;
        fr3_model_updater_.writeCommand(fr3_model_updater_.q_desired_total_);
    }
    else
    {
        throw std::runtime_error("no supported manipulator command interface");
    }
}

PolicyControl::ComputeResult PolicyControl::compute(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const double now_s = time.seconds();
    cycle_time_s_ = now_s;
    cycle_period_s_ = period.seconds();

    std::string interruption;
    if (!robot_interrupted_ && robotControlInterrupted(interruption))
    {
        // A reflex (or anything that made the hardware restart control) ends the policy:
        // never continue it on the restarted robot.
        robot_interrupted_ = true;
        result_message_ = "robot reflex or control restart (" + interruption + ")";
        RCLCPP_ERROR(
            node_->get_logger(), "[%s] %s; policy stopped",
            name_.c_str(), result_message_.c_str());
        if (position_mode_ && readRobotCommandState())
        {
            // Continue from what the robot executes now (after a restart: the pose it
            // holds), so the restarted motion's first command is consistent.
            position_tracker_.syncToRobot(robot_q_d_, robot_dq_d_, robot_ddq_d_, 0.0);
        }
        return finishOrBrake(ComputeResult::ABORTED, now_s);
    }

    if (position_stopping_)
    {
        // Ignore further policy commands while braking to rest.
        return continuePositionStop(now_s);
    }
    const CommandData* command = command_buffer_.readFromRT();

    if (command && command->received_time_s >= activation_time_s_ &&
        command->sequence > last_sequence_)
    {
        std::string error;
        if (!validateTarget(*command, error))
        {
            result_message_ = error;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), error.c_str());
            return finishOrBrake(ComputeResult::ABORTED, now_s);
        }

        const Eigen::Index offset =
            static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
        for (size_t i = 0; i < controlled_dof_; ++i)
        {
            const Eigen::Index index = offset + static_cast<Eigen::Index>(i);
            if (isaac_relative_control_)
            {
                if (command->relative_position_offsets)
                {
                    // Physics-step reference: hold the processed offset, then
                    // add it to the latest measurement in every control cycle.
                    q_offset_(index) = command->target_positions[i];
                }
                else
                {
                    // Policy-step reference: the policy node captured the
                    // measured position at its 20 Hz tick. Hold that absolute
                    // target until the next command while retaining Isaac PD.
                    q_target_(index) = command->target_positions[i];
                    q_desired_(index) = q_target_(index);
                    qdot_desired_(index) = 0.0;
                }
            }
            else
            {
                q_target_(index) = command->target_positions[i];
            }
        }
        if (isaac_relative_control_)
        {
            refresh_isaac_relative_target_ = command->relative_position_offsets;
            if (position_mode_)
            {
                // New segment of the tracker reference towards this policy target.
                position_tracker_.setTarget(q_target_);
                position_new_target_ = true;
            }
        }
        last_sequence_ = command->sequence;
        last_command_time_s_ = command->received_time_s;
        has_command_ = true;

        if (control_gripper_ && !controlled_dual_)
        {
            const int gripper_state = command->gripper_action < 0.0 ? 1 : 0;
            if (gripper_state != last_gripper_state_)
            {
                const bool ok = gripper_state == 1
                    ? fr3_model_updater_.GripperGrasp(controlled_robot_)
                    : fr3_model_updater_.GripperOpen(controlled_robot_);
                if (!ok)
                {
                    RCLCPP_WARN(
                        node_->get_logger(), "[%s] failed to send gripper command",
                        name_.c_str());
                }
                last_gripper_state_ = gripper_state;
            }
        }
    }

    const double command_age_s = now_s - last_command_time_s_;
    if (command_age_s > command_timeout_s_)
    {
        result_message_ =
            has_command_ ? "policy command timeout" : "no policy command received";
        RCLCPP_ERROR(
            node_->get_logger(), "[%s] %s (age %.3fs)",
            name_.c_str(), result_message_.c_str(), command_age_s);
        return finishOrBrake(ComputeResult::ABORTED, now_s);
    }

    if (max_duration_s_ > 0.0 &&
        now_s - activation_time_s_ >= max_duration_s_)
    {
        result_message_ = "maximum policy duration reached";
        return finishOrBrake(ComputeResult::SUCCEEDED, now_s);
    }

    if (isaac_relative_control_)
    {
        if (model_updater_.HasEffortCommandInterface())
        {
            // Both reference modes evaluate PD torque and its torque-rate limit on
            // every controller cycle. Only physics-step mode refreshes the target
            // from measured_q + held_offset; policy-step mode holds q_desired.
            if (refresh_isaac_relative_target_)
            {
                refreshIsaacRelativeTarget();
            }
            writeIsaacEffortCommand(period.seconds());
        }
        else if (model_updater_.HasPositionCommandInterface())
        {
            // The policy-step target (measured q at the 20 Hz tick + offset) starts a
            // linear reference segment; the tracker turns it into a limit-respecting q_d.
            writeTrackedPositionCommand();
        }
        else
        {
            result_message_ = "no supported command interface for Isaac-relative control";
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), result_message_.c_str());
            return ComputeResult::ABORTED;
        }
    }
    else
    {
        updateRateLimitedTarget(period.seconds());
        writeDesiredCommand(q_desired_, qdot_desired_);
    }

    double max_joint_error = 0.0;
    const Eigen::Index offset =
        static_cast<Eigen::Index>(controlled_robot_index_ * FR3_DOF);
    for (size_t i = 0; i < controlled_dof_; ++i)
    {
        max_joint_error = std::max(
            max_joint_error,
            std::abs(
                q_desired_(offset + static_cast<Eigen::Index>(i)) -
                fr3_model_updater_.q_total_(offset + static_cast<Eigen::Index>(i))));
    }

    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->command_age_s = command_age_s;
    feedback->max_joint_error = max_joint_error;
    feedback->last_sequence = last_sequence_;
    if (!has_command_)
    {
        feedback->status = "waiting for first command";
    }
    else if (position_mode_)
    {
        feedback->status = position_tracker_.accelerationClamped()
            ? "tracking-policy-step-position (acceleration limited)"
            : "tracking-policy-step-position";
    }
    else if (isaac_relative_control_)
    {
        feedback->status = refresh_isaac_relative_target_
            ? "tracking-physics-step-reference"
            : "tracking-policy-step-reference";
    }
    else
    {
        feedback->status = "tracking";
    }
    publishFeedback(feedback);
    return ComputeResult::RUNNING;
}

void PolicyControl::onStop(StopReason reason)
{
    model_updater_.haltCommands();
    if (position_mode_ && robot_state_available_)
    {
        max_resync_run_ = std::max(max_resync_run_, resync_run_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] q_d sync: %lu re-syncs, longest run %lu ms (0 when the robot executed every command)",
            name_.c_str(), static_cast<unsigned long>(resync_count_),
            static_cast<unsigned long>(max_resync_run_));
    }
    if (reason == StopReason::CANCELED)
    {
        result_message_ = "policy control canceled";
    }
    else if (reason == StopReason::ABORTED && result_message_ == "running")
    {
        result_message_ = "policy control aborted";
    }
    RCLCPP_INFO(
        node_->get_logger(), "[%s] stopped: %s",
        name_.c_str(), result_message_.c_str());
}

PolicyControl::ResultPtr PolicyControl::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    result->success = reason == StopReason::SUCCEEDED;
    result->message = result_message_;
    result->last_sequence = last_sequence_;
    return result;
}

REGISTER_FR3_ACTION_SERVER(PolicyControl, "fr3_policy_control")

}  // namespace fr3_husky_controller::servers::fr3
