#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <franka/robot_state.h>
#include <fr3_husky_msgs/action/run_policy_control.hpp>
#include <fr3_husky_msgs/msg/policy_joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/utils/joint_command_tracker.hpp>

namespace fr3_husky_controller::servers::fr3
{

class PolicyControl final : public ActionServerBase<fr3_husky_msgs::action::RunPolicyControl>
{
public:
    using ActionT = fr3_husky_msgs::action::RunPolicyControl;
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    PolicyControl(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
    ~PolicyControl() override = default;

    int priority() const override { return 9; }
    bool requiresRobotDataUpdate() const override { return !isaac_relative_control_; }

    // Controller node parameters (optional, read at goal acceptance) for the position
    // tracker; defaults match the Isaac Lab position-mode action term.
    static constexpr const char* kTrackerWpParam = "policy_position_tracker.wp";
    static constexpr const char* kTrackerLimitFractionParam = "policy_position_tracker.limit_fraction";
    static constexpr const char* kTrackerBrakeFractionParam = "policy_position_tracker.brake_fraction";
    // bool: linear segment to each new target over interpolation_period (false: hold target).
    static constexpr const char* kTrackerInterpolateParam = "policy_position_tracker.interpolate";
    static constexpr const char* kTrackerInterpolationPeriodParam = "policy_position_tracker.interpolation_period";
    // bool: re-base the tracker on the robot's executed command q_d (real robot only).
    static constexpr const char* kTrackerSyncParam = "policy_position_tracker.sync_to_robot";
    // bool: publish the 1 kHz position diagnostics on ~/policy_position_debug.
    static constexpr const char* kTrackerDebugParam = "policy_position_tracker.debug_topic";
    bool allowPreemption() const override { return false; }

private:
    struct CommandData
    {
        std::array<double, 2 * FR3_DOF> target_positions{};
        size_t target_count{0};
        double gripper_action{0.0};
        double received_time_s{-1.0};
        uint64_t sequence{0};
        int robot_index{-1};
        bool dual{false};
        bool relative_position_offsets{false};
        bool valid{false};
    };

    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

    void commandCallback(const fr3_husky_msgs::msg::PolicyJointCommand::SharedPtr msg);
    int resolveRobotIndex(const std::string& robot_name) const;
    bool validateTarget(const CommandData& command, std::string& error) const;
    void updateRateLimitedTarget(double period_s);
    void refreshIsaacRelativeTarget();
    void writeDesiredCommand(const Eigen::VectorXd& q_desired, const Eigen::VectorXd& qdot_desired);
    void writeIsaacEffortCommand(double period_s);
    // Position command interface: held policy target -> 1 kHz tracker -> joint position command.
    bool readPositionTrackerParams(std::string& error);
    // One 1 kHz cycle: sync to the robot's q_d (if available), step along the reference, write.
    void writeTrackedPositionCommand();
    // Robot state monitor (real robot): a robot time jump, a robot mode other than
    // Move, current errors or a new last_motion_errors means a reflex or a control
    // restart by the hardware's automatic recovery.  The policy must not resume.
    bool robotControlInterrupted(std::string& reason);
    bool readRobotCommandState();
    void publishPositionDebug(bool resynced);
    // Effort: return result now.  Position: brake the tracker to rest first (a step
    // in the position command would trigger a Franka discontinuity reflex), then return it.
    ComputeResult finishOrBrake(ComputeResult result, double now_s);
    ComputeResult continuePositionStop(double now_s);

    FR3ModelUpdater& fr3_model_updater_;
    rclcpp::Subscription<fr3_husky_msgs::msg::PolicyJointCommand>::SharedPtr command_sub_;
    realtime_tools::RealtimeBuffer<CommandData> command_buffer_;

    std::string controlled_robot_;
    int controlled_robot_index_{-1};
    size_t controlled_dof_{FR3_DOF};
    bool controlled_dual_{false};
    double command_timeout_s_{0.15};
    double max_duration_s_{8.0};
    double max_policy_target_delta_rad_{1.0};
    double max_actuator_step_rad_{0.001};
    double joint_velocity_scale_{0.10};
    double joint_acceleration_scale_{0.20};
    bool isaac_relative_control_{false};
    // True for physics-step measured-relative references; false for an
    // absolute target captured at the policy step.
    bool refresh_isaac_relative_target_{true};
    bool control_gripper_{true};

    double activation_time_s_{0.0};
    double last_command_time_s_{0.0};
    uint64_t last_sequence_{0};
    bool has_command_{false};
    int last_gripper_state_{-1};
    Eigen::VectorXd q_hold_;
    Eigen::VectorXd q_target_;
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd qdot_desired_;
    Eigen::VectorXd q_offset_;
    // Matches dual_fr3_lab TorqueRateLimitedPDActuator.  The state is reset
    // at policy start and is used only by the Isaac-relative effort path.
    Eigen::VectorXd previous_applied_effort_;

    // Isaac-relative control on a position command interface (else effort).
    bool position_mode_{false};
    utils::JointCommandTrackerParams position_tracker_params_{};
    utils::JointCommandTracker position_tracker_;
    bool position_stopping_{false};
    bool position_sync_{true};
    bool position_debug_{true};
    bool position_new_target_{false};
    ComputeResult position_stop_result_{ComputeResult::ABORTED};
    double position_stop_start_s_{0.0};
    Eigen::VectorXd position_stop_target_;
    static constexpr double kPositionStopTimeoutS = 1.0;
    static constexpr double kPositionRestVelocity = 1.0e-3;      // rad/s
    static constexpr double kPositionRestAcceleration = 0.5;     // rad/s^2

    // Real robot state per arm (robot_names_ order); empty / unavailable in MuJoCo.
    std::vector<const franka::RobotState*> robot_states_;
    bool robot_state_available_{false};
    bool robot_time_valid_{false};
    bool robot_interrupted_{false};
    std::vector<uint64_t> last_robot_time_ms_;
    std::vector<bool> motion_error_seen_;
    Eigen::VectorXd robot_q_d_;
    Eigen::VectorXd robot_dq_d_;
    Eigen::VectorXd robot_ddq_d_;
    uint64_t resync_count_{0};
    uint64_t resync_run_{0};
    uint64_t max_resync_run_{0};
    double cycle_time_s_{0.0};
    double cycle_period_s_{0.0};
    uint64_t debug_cycle_{0};
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> debug_pub_;
    // A normal cycle advances robot time by 1 ms (2 ms after one missed packet);
    // the automatic error recovery + control restart takes tens of ms.
    static constexpr uint64_t kRobotTimeGapMs = 20;
    // q_d equals the previous command in normal operation; re-sync beyond this.
    static constexpr double kSyncPositionTolerance = 1.0e-6;  // rad
    // Consecutive re-syncs = robot not executing the commands; warn after this many cycles.
    static constexpr uint64_t kFrozenWarnCycles = 50;

    std::string result_message_{"not started"};

    static constexpr std::array<double, FR3_DOF> kLowerLimits{
        -2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
    static constexpr std::array<double, FR3_DOF> kUpperLimits{
        2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
    // franka_description/robots/fr3/joint_limits.yaml
    static constexpr std::array<double, FR3_DOF> kMaxJointVelocities{
        2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
    // fr3_husky_moveit_config/config/dual/dual_fr3_joint_limits.yaml
    static constexpr std::array<double, FR3_DOF> kMaxJointAccelerations{
        3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0};
    // dual_fr3_lab DUAL_FR3_PLATE_EE_CFG implicit actuator parameters.
    static constexpr std::array<double, FR3_DOF> kIsaacStiffness{
        80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0};
    static constexpr std::array<double, FR3_DOF> kIsaacDamping{
        4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0};
    static constexpr std::array<double, FR3_DOF> kIsaacEffortLimits{
        87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
    static constexpr double kIsaacTorqueRateLimit = 1000.0;  // Nm/s
    static constexpr double kJointLimitMargin = 0.02;
    static constexpr double kPositionTolerance = 1.0e-6;
    static constexpr double kNominalControlPeriodS = 0.001;
    static constexpr double kMaxLimiterPeriodS = kNominalControlPeriodS;
};

}  // namespace fr3_husky_controller::servers::fr3
