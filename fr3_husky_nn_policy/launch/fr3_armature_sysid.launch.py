"""Safe acceleration-rich dual-FR3 SysID collection for armature estimation."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


REPOSITORY_ROOT = Path(os.path.realpath(__file__)).parents[2]
DEFAULT_LOG_BASE = REPOSITORY_ROOT / "data" / "sysid_armature"


def generate_launch_description():
    package_share = FindPackageShare("fr3_husky_nn_policy")
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "fr3_sysid.launch.py"])
        ),
        launch_arguments={
            "load_mobile": LaunchConfiguration("load_mobile"),
            "use_mujoco": LaunchConfiguration("use_mujoco"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "with_realsense": "false",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "excitation_profile": "armature",
            "sweep_mode": LaunchConfiguration("sweep_mode"),
            "target_joint": LaunchConfiguration("target_joint"),
            "command_rate_hz": LaunchConfiguration("command_rate_hz"),
            "dry_run": LaunchConfiguration("dry_run"),
            "auto_start": LaunchConfiguration("auto_start"),
            "log_base": LaunchConfiguration("log_base"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_move_group", default_value="false"),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/dual_fr3/joint_states"
            ),
            # Start live collection with one manually verified joint.
            DeclareLaunchArgument("sweep_mode", default_value="false"),
            DeclareLaunchArgument("target_joint", default_value="5"),
            DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("log_base", default_value=str(DEFAULT_LOG_BASE)),
            base,
        ]
    )
