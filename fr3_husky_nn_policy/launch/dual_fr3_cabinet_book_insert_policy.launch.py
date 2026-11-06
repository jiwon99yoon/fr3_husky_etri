"""Launch the cabinet book-insertion policy, shadow by default.

Examples::

    ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py model_path:=/path/model.npz
    ros2 service call /ppo_cabinet_book_insert_policy_node/start_policy std_srvs/srv/Trigger
    ros2 service call /ppo_cabinet_book_insert_policy_node/stop_policy std_srvs/srv/Trigger

Set ``shadow_mode:=false`` only after the upstream book-pose and door-angle estimates
have been checked and the start state has been reviewed on the robot.
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
    controller_share = FindPackageShare("fr3_husky_controller")
    policy_share = FindPackageShare("fr3_husky_nn_policy")
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([controller_share, "launch", "fr3_action_controller.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        launch_arguments={
            "robot_side": "dual",
            "load_mobile": LaunchConfiguration("load_mobile"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
            # UMI hands: closed on the handle and the book with gripper_move before start_policy
            "load_gripper": LaunchConfiguration("load_gripper"),
        }.items(),
    )
    policy = Node(
        package="fr3_husky_nn_policy",
        executable="ppo_cabinet_book_insert_policy_node",
        name="ppo_cabinet_book_insert_policy_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([policy_share, "config", "dual_fr3_cabinet_book_insert.yaml"]),
            {
                "model_path": LaunchConfiguration("model_path"),
                "shadow_mode": ParameterValue(LaunchConfiguration("shadow_mode"), value_type=bool),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "book_pose_topic": LaunchConfiguration("book_pose_topic"),
                "door_angle_topic": LaunchConfiguration("door_angle_topic"),
                "joint_velocity_scale": ParameterValue(LaunchConfiguration("joint_velocity_scale"), value_type=float),
                "max_actuator_step_rad": ParameterValue(LaunchConfiguration("max_actuator_step_rad"), value_type=float),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("model_path", description="Portable NPZ exported from the exact training run"),
        DeclareLaunchArgument("shadow_mode", default_value="true"),
        DeclareLaunchArgument("launch_controller", default_value="true"),
        DeclareLaunchArgument("load_mobile", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_move_group", default_value="false"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        # validated by the controller only; the Isaac PD mode used here does not slow the arms with these
        DeclareLaunchArgument("joint_velocity_scale", default_value="0.10"),
        DeclareLaunchArgument("max_actuator_step_rad", default_value="0.001"),
        DeclareLaunchArgument("joint_states_topic", default_value="/dual_fr3/joint_states"),
        DeclareLaunchArgument("book_pose_topic", default_value="/cabinet_book_insert/book_pose"),
        DeclareLaunchArgument("door_angle_topic", default_value="/cabinet_book_insert/door_angle"),
        controller,
        policy,
    ])
