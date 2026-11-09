"""Isaac Lab 3.0 cube pick-and-place on the real robot: controller (dual, Isaac PD) + policy node.

The policy node drives the right arm through /fr3_policy_control (robot_name dual, left arm held) and sends the
right Franka gripper Grasp/Move goals itself. Start with shadow_mode:=true, then shadow_mode:=false and call
/ppo_pickplace_cube_policy_node/start_policy. The goal is goal_xy_m (base); change it with
`ros2 param set /ppo_pickplace_cube_policy_node goal_xy_m "[x, y]"` (centre) or goal_offset_xy_m "[dx, dy]" while IDLE.
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
            # no wrist D435i bodies, as in training
            "with_realsense": "false",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
        }.items(),
    )

    policy = Node(
        package="fr3_husky_nn_policy",
        executable="ppo_pickplace_cube_policy_node",
        name="ppo_pickplace_cube_policy_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([policy_share, "config", "dual_fr3_pick_place_cube.yaml"]),
            {
                "model_path": LaunchConfiguration("model_path"),
                "object_pose_topic": LaunchConfiguration("object_pose_topic"),
                "object_position_offset_m": ParameterValue(LaunchConfiguration("object_position_offset_m"),
                                                           value_type=list[float]),
                "goal_xy_m": ParameterValue(LaunchConfiguration("goal_xy_m"), value_type=list[float]),
                "goal_offset_xy_m": ParameterValue(LaunchConfiguration("goal_offset_xy_m"), value_type=list[float]),
                "shadow_mode": LaunchConfiguration("shadow_mode"),
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
            DeclareLaunchArgument("object_pose_topic", default_value="/cube_tf",
                                  description="PoseStamped of the cube centre, any frame with a TF to base"),
            DeclareLaunchArgument("object_position_offset_m", default_value="[0.0, 0.0, 0.0]",
                                  description="added to the cube position in base before the policy sees it"),
            DeclareLaunchArgument("goal_offset_xy_m", default_value="[0.0, 0.0]",
                                  description="added to goal_xy_m (offset around a fixed goal centre)"),
            DeclareLaunchArgument("goal_xy_m", default_value="[0.50, -0.20]",
                                  description="goal centre x/y in base (trained x 0.45-0.65, y -0.45-0.05, "
                                              "within 0.62 m of (0, -0.215), >= 0.10 m from the cube)"),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [policy_share, "models", "dual_fr3_pick_place_cube_gripper_busy_model4000.npz"]
                ),
                description="exported actor (README PickPlaceCube)",
            ),
            controller,
            policy,
        ]
    )
