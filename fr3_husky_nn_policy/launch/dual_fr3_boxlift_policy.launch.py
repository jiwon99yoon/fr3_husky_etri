"""Dual-FR3 plate box-lift policy: controller (optional) + ppo_boxlift_policy_node.

Examples::

    # shadow (default): inference + logging only, nothing is commanded
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=/path/boxlift.npz
    ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger
    ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger

    # command mode (stopping drops a lifted box: no hold mode yet)
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=... shadow_mode:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
            "load_gripper": "false",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
        }.items(),
    )

    policy = Node(
        package="fr3_husky_nn_policy",
        executable="ppo_boxlift_policy_node",
        name="ppo_boxlift_policy_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([policy_share, "config", "dual_fr3_boxlift.yaml"]),
            {
                "model_path": LaunchConfiguration("model_path"),
                "shadow_mode": ParameterValue(LaunchConfiguration("shadow_mode"), value_type=bool),
                "goal_z_m": ParameterValue(LaunchConfiguration("goal_z_m"), value_type=float),
                "box_pose_topic": LaunchConfiguration("box_pose_topic"),
                "log_task_name": PythonExpression(
                    [
                        "'dual_fr3_boxlift' if '",
                        LaunchConfiguration("use_mujoco"),
                        "'.lower() == 'true' else 'dual_fr3_boxlift_real'",
                    ]
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", description="NPZ from export_boxlift_actor.py (required)"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("goal_z_m", default_value="0.15",
                                  description="box-center lift height in base (trained 0.15-0.35)"),
            DeclareLaunchArgument("box_pose_topic", default_value="/box_pose"),
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_move_group", default_value="false"),
            controller,
            policy,
        ]
    )
