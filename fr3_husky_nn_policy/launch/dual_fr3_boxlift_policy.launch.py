"""Dual-FR3 plate box-lift policy: controller (optional) + ppo_boxlift_policy_node.

Examples::

    # shadow (default): inference + logging only, nothing is commanded
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=/path/boxlift.npz
    ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger
    ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger

    # command mode (stopping drops a lifted box: no hold mode yet)
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=... shadow_mode:=false

    # MuJoCo sim-to-sim: training scene (gravity on the box only, base 0.405 m above the floor) with the
    # box and both arms at one row of the pre-grasp table embedded in the NPZ. MuJoCo publishes the
    # ground-truth box pose on /box_pose, so do not also publish it by hand.
    # model_2999 candidates (docs/dual_fr3_box_experi_step.md): A 1919, B 3788, C 3925, D 720, E 2228
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=... \
        use_mujoco:=true mujoco_start_row:=1919
"""

import math

import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _mujoco_start(context):
    """Controller launch arguments that place the arms and the box at one pre-grasp table row."""
    if LaunchConfiguration("use_mujoco").perform(context).lower() != "true":
        return {}
    row = int(LaunchConfiguration("mujoco_start_row").perform(context))
    model_path = LaunchConfiguration("model_path").perform(context)
    with np.load(model_path, allow_pickle=False) as archive:
        if "pregrasp_joint_position" not in archive.files:
            raise RuntimeError(f"{model_path} has no pre-grasp table; mujoco_start_row needs a -Pregrasp policy")
        table_q = archive["pregrasp_joint_position"]
        table_box = archive["pregrasp_box_pose_base"]
    if not 0 <= row < len(table_q):
        raise RuntimeError(f"mujoco_start_row {row} is outside the pre-grasp table (0..{len(table_q) - 1})")
    x, y = table_box[row, 0], table_box[row, 1]
    qz, qw = table_box[row, 5], table_box[row, 6]  # (x, y, z, w), yaw-only box orientation
    yaw = 2.0 * math.atan2(qz, qw)
    friction = float(LaunchConfiguration("mujoco_box_friction").perform(context))
    scene_args = (
        f"gravity_z:=-9.81 mobile_mount_z:={LaunchConfiguration('mujoco_mount_z').perform(context)}"
        f" with_box:=true box_x:={x:.6f} box_y:={y:.6f} box_yaw:={yaw:.6f}"
        f" box_mass:={LaunchConfiguration('mujoco_box_mass').perform(context)}"
        f" box_friction:={friction} plate_friction:={friction}"
        f" box_condim:={LaunchConfiguration('mujoco_box_condim').perform(context)}"
        f" contact_timeconst:={LaunchConfiguration('mujoco_contact_timeconst').perform(context)}"
    )
    print(f"[dual_fr3_boxlift] MuJoCo start: table row {row}, box ({x:.4f}, {y:.4f}, yaw {math.degrees(yaw):.2f} deg)")
    return {
        "mujoco_scene_extra_args": scene_args,
        "mujoco_initial_joint_positions": " ".join(f"{q:.6f}" for q in table_q[row]),
        "mujoco_object_pose_body": "box",
    }


def _launch_setup(context, *args, **kwargs):
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
            # MJCF only: no wrist D435i bodies, as in training and the SysID models.
            "with_realsense": "false",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_move_group": LaunchConfiguration("launch_move_group"),
            **_mujoco_start(context),
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
                "box_size_m": ParameterValue(LaunchConfiguration("box_size_m"), value_type=list[float]),
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
    return [controller, policy]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", description="NPZ from export_boxlift_actor.py (required)"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("goal_z_m", default_value="0.15",
                                  description="box-center lift height in base (trained 0.15-0.35)"),
            DeclareLaunchArgument("box_size_m", default_value="[0.410, 0.310, 0.280]",
                                  description="physical box (x, y squeezed, z) in base; must match the policy's box"),
            DeclareLaunchArgument("box_pose_topic", default_value="/box_pose"),
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_move_group", default_value="false"),
            # MuJoCo only (use_mujoco:=true)
            DeclareLaunchArgument("mujoco_start_row", default_value="1919",
                                  description="pre-grasp table row for the arm and box start (model_2999: A 1919)"),
            DeclareLaunchArgument("mujoco_mount_z", default_value="0.2727",
                                  description="arm mount above Husky base_link; 0.2727 = 0.405 m above the floor "
                                              "as in training, 0.26 = current URDF/MJCF (0.392 m)"),
            DeclareLaunchArgument("mujoco_box_mass", default_value="0.5", description="trained 0.4-0.6 kg"),
            DeclareLaunchArgument("mujoco_box_friction", default_value="0.8",
                                  description="box and plate friction (trained box friction 0.4-1.2)"),
            DeclareLaunchArgument("mujoco_box_condim", default_value="4",
                                  description="box contact dimension (3 slide, 4 +torsion, 6 +rolling)"),
            DeclareLaunchArgument("mujoco_contact_timeconst", default_value="0.02",
                                  description="box/plate solref time constant [s]; MuJoCo default 0.02, smaller is stiffer"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
