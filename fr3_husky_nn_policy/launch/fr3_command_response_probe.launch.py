"""FR3 command-response probe (absolute 20 Hz targets) with the FR3 action controller.

Torque mode:   ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py robot:=left
Position mode: ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py robot:=left \
                   command_interface:=position
Live motion needs dry_run:=false.  Start / stop:
    ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
    ros2 service call /fr3_command_response_probe/stop_probe std_srvs/srv/Trigger {}
Use launch_controller:=false when the controller is already running.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

REPOSITORY_ROOT = Path(os.path.realpath(__file__)).parents[2]
DEFAULT_DATA_ROOT = REPOSITORY_ROOT / "data" / "command_response"

# (name, default, type) forwarded to the probe node.  Lists are YAML strings.
PROBE_ARGS = [
    ("excite_arm", "auto", str),
    ("tests", "[step, sine, chirp]", list),
    ("joints", "[1, 2, 3, 4, 5, 6, 7]", list),
    ("target_rate_hz", "20.0", float),
    ("step_amplitudes_rad", "[0.02, 0.05, 0.10]", list),
    ("sine_frequencies_hz", "[0.5, 1.0, 2.0, 3.0]", list),
    ("sine_amplitude_rad", "0.05", float),
    ("chirp_f0_hz", "0.2", float),
    ("chirp_f1_hz", "5.0", float),
    ("chirp_duration_s", "30.0", float),
    ("chirp_amplitude_rad", "0.03", float),
    ("press_levels_rad", "[0.002, 0.005, 0.01]", list),
    ("press_direction", "1.0", float),
    ("press_max_dtau_nm", "10.0", float),
    ("hold_duration_s", "3.0", float),
    ("replay_file", "none", str),
    ("max_policy_target_delta_rad", "0.20", float),
    ("label", "auto", str),
    ("notes", "-", str),
    ("dry_run", "true", bool),
    ("auto_start", "false", bool),
]


def generate_launch_description():
    controller_share = FindPackageShare("fr3_husky_controller")
    log_root = PythonExpression([
        "'", LaunchConfiguration("log_base"), "/' + ('mujoco' if '",
        LaunchConfiguration("use_mujoco"), "' in ('true', 'True', '1') else 'real')",
    ])

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([controller_share, "launch", "fr3_action_controller.launch.py"])
        ),
        launch_arguments={
            "robot_side": LaunchConfiguration("robot"),
            "command_interface": LaunchConfiguration("command_interface"),
            "load_mobile": LaunchConfiguration("load_mobile"),
            "use_mujoco": LaunchConfiguration("use_mujoco"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "load_gripper": "false",
            "with_realsense": LaunchConfiguration("with_realsense"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_controller")),
    )

    params = {
        "robot": LaunchConfiguration("robot"),
        "command_interface": LaunchConfiguration("command_interface"),
        "log_root": log_root,
    }
    for name, _default, kind in PROBE_ARGS:
        value_type = {str: str, float: float, bool: bool, list: None}[kind]
        params[name] = (
            ParameterValue(LaunchConfiguration(name))
            if value_type is None
            else ParameterValue(LaunchConfiguration(name), value_type=value_type)
        )

    probe = Node(
        package="fr3_husky_nn_policy",
        executable="fr3_command_response_probe",
        name="fr3_command_response_probe",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="left", description="left | right | dual"),
            DeclareLaunchArgument("command_interface", default_value="effort",
                                  description="effort | position (controller and probe check)"),
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            # Plate EE without camera, as in fr3_sysid.launch.py (true only for the C2 vision test).
            DeclareLaunchArgument("with_realsense", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            # move_group is needed for move_to_joint (start pose) before start_probe.
            DeclareLaunchArgument("launch_move_group", default_value="true"),
            DeclareLaunchArgument("log_base", default_value=str(DEFAULT_DATA_ROOT)),
        ]
        + [DeclareLaunchArgument(name, default_value=default) for name, default, _ in PROBE_ARGS]
        + [controller, probe]
    )
