"""Isaac Lab 3.0 cube lift Sim2Real-100Hz on the real robot: controller (dual, Isaac PD) + policy node.

The policy node drives the right arm through /fr3_policy_control (robot_name dual, left arm held) and sends the
right Franka gripper Grasp/Move goals itself. Start with shadow_mode:=true, then shadow_mode:=false and call
/ppo_liftcube_sim2real_policy_node/start_policy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    policy_share = FindPackageShare("fr3_husky_nn_policy")
    controller_share = FindPackageShare("fr3_husky_controller")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([controller_share, "launch", "fr3_action_controller.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        launch_arguments={
            "robot_side": "dual",
            "load_mobile": LaunchConfiguration("load_mobile"),
            "use_mujoco": LaunchConfiguration("use_mujoco"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "load_gripper": "true",
            # no wrist D435i bodies, as in training (dual_fr3_no_handeye.usd)
            "with_realsense": "false",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
        }.items(),
    )

    policy = Node(
        package="fr3_husky_nn_policy",
        executable="ppo_liftcube_sim2real_policy_node",
        name="ppo_liftcube_sim2real_policy_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([policy_share, "config", "dual_fr3_lift_sim2real100hz.yaml"]),
            {
                "model_path": LaunchConfiguration("model_path"),
                "object_pose_topic": LaunchConfiguration("object_pose_topic"),
                "object_position_offset_m": ParameterValue(LaunchConfiguration("object_position_offset_m"),
                                                           value_type=list[float]),
                "goal_z_m": ParameterValue(LaunchConfiguration("goal_z_m"), value_type=float),
                "goal_xy_offset_m": ParameterValue(LaunchConfiguration("goal_xy_offset_m"), value_type=list[float]),
                "goal_xy_reference": LaunchConfiguration("goal_xy_reference"),
                "shadow_mode": LaunchConfiguration("shadow_mode"),
                "auto_start": LaunchConfiguration("auto_start"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_move_group", default_value="false"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("object_pose_topic", default_value="/object_pose",
                                  description="PoseStamped of the cube centre, any frame with a TF to base"),
            DeclareLaunchArgument("object_position_offset_m", default_value="[0.0, 0.0, 0.0]",
                                  description="added to the cube position in base before the policy sees it"),
            DeclareLaunchArgument("goal_z_m", default_value="0.50",
                                  description="goal z in base (trained 0.45-0.60; table top 0.345)"),
            DeclareLaunchArgument("goal_xy_offset_m", default_value="[0.0, 0.0]",
                                  description="goal x/y = reference x/y + this (base), must stay in the trained "
                                              "goal range x 0.35-0.55, y -0.45--0.15"),
            DeclareLaunchArgument("goal_xy_reference", default_value="range_center",
                                  description="range_center (0.45, -0.30, center of the trained goal range) or "
                                              "cube_start (cube x/y at start_policy)"),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [policy_share, "models", "dual_fr3_lift_sim2real100hz_model4999.npz"]
                ),
            ),
            controller,
            policy,
        ]
    )
