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
    # Wide candidates (docs/dual_fr3_box_experi_step.md): A 1965, B 2790, C 2166, D 371, E 1257
    # (box410 model_2999: A 1919, B 3788, C 3925, D 720, E 2228 with the default box_size_m)
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=... \
        box_size_m:="[0.310, 0.410, 0.280]" use_mujoco:=true mujoco_start_row:=1965

    # hover start (-Curriculum checkpoints, manifest start.hover): arms at the fixed hover pose, box anywhere in the
    # trained hover range. MuJoCo puts the box at mujoco_hover_box (x, y in base [m], yaw [deg])
    ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py model_path:=... \
        box_size_m:="[0.310, 0.410, 0.280]" start_pose:=hover use_mujoco:=true mujoco_hover_box:="[0.45, 0.0, 0.0]"
"""

import json
import math

import numpy as np
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _mujoco_start(context):
    """Controller launch arguments that place the arms and the box at one pre-grasp table row (or the hover start)."""
    if LaunchConfiguration("use_mujoco").perform(context).lower() != "true":
        return {}
    model_path = LaunchConfiguration("model_path").perform(context)
    if LaunchConfiguration("start_pose").perform(context) == "hover":
        with np.load(model_path, allow_pickle=False) as archive:
            hover = json.loads(str(archive["manifest_json"].item())).get("start", {}).get("hover")
        if hover is None:
            raise RuntimeError(f"{model_path} has no hover start (manifest start.hover); start_pose:=hover needs a "
                               "-Curriculum policy")
        box = yaml.safe_load(LaunchConfiguration("mujoco_hover_box").perform(context))
        if len(box) != 3:
            raise RuntimeError(f"mujoco_hover_box must be [x, y, yaw_deg], got {box!r}")
        start_q = np.asarray(hover["joint_position"], dtype=np.float64)
        x, y, yaw = float(box[0]), float(box[1]), math.radians(float(box[2]))
        start_label = "hover pose"
    else:
        row = int(LaunchConfiguration("mujoco_start_row").perform(context))
        with np.load(model_path, allow_pickle=False) as archive:
            if "pregrasp_joint_position" not in archive.files:
                raise RuntimeError(f"{model_path} has no pre-grasp table; mujoco_start_row needs a -Pregrasp policy")
            table_q = archive["pregrasp_joint_position"]
            table_box = archive["pregrasp_box_pose_base"]
        if not 0 <= row < len(table_q):
            raise RuntimeError(f"mujoco_start_row {row} is outside the pre-grasp table (0..{len(table_q) - 1})")
        start_q = table_q[row]
        x, y = table_box[row, 0], table_box[row, 1]
        qz, qw = table_box[row, 5], table_box[row, 6]  # (x, y, z, w), yaw-only box orientation
        yaw = 2.0 * math.atan2(qz, qw)
        start_label = f"table row {row}"
    friction = float(LaunchConfiguration("mujoco_box_friction").perform(context))
    # The MuJoCo box is the physical box the node is told about (and checks against the policy).
    size = yaml.safe_load(LaunchConfiguration("box_size_m").perform(context))
    if len(size) != 3:
        raise RuntimeError(f"box_size_m must be [x, y, z], got {size!r}")
    goal_offset = yaml.safe_load(LaunchConfiguration("goal_xy_offset_m").perform(context))
    if len(goal_offset) != 2:
        raise RuntimeError(f"goal_xy_offset_m must be [dx, dy], got {goal_offset!r}")
    if LaunchConfiguration("goal_xy_reference").perform(context) == "range_center":
        with np.load(model_path, allow_pickle=False) as archive:
            xy_range = json.loads(str(archive["manifest_json"].item())).get("goal", {}).get("xy_range_base")
        if xy_range is None:
            raise RuntimeError("goal_xy_reference:=range_center needs a checkpoint with goal.xy_range_base")
        center = [0.5 * (xy_range[a][0] + xy_range[a][1]) for a in "xy"]
        goal_offset = [center[0] + goal_offset[0] - x, center[1] + goal_offset[1] - y]
    scene_args = (
        f"reach_markers:=false gravity_z:=-9.81 mobile_mount_z:={LaunchConfiguration('mujoco_mount_z').perform(context)}"
        f" with_box:=true box_x:={x:.6f} box_y:={y:.6f} box_yaw:={yaw:.6f}"
        f" box_size_x:={size[0]} box_size_y:={size[1]} box_size_z:={size[2]}"
        f" goal_z:={LaunchConfiguration('goal_z_m').perform(context)}"
        f" goal_dx:={goal_offset[0]} goal_dy:={goal_offset[1]}"
        f" box_mass:={LaunchConfiguration('mujoco_box_mass').perform(context)}"
        f" box_friction:={friction} plate_friction:={friction}"
        f" box_condim:={LaunchConfiguration('mujoco_box_condim').perform(context)}"
        f" contact_timeconst:={LaunchConfiguration('mujoco_contact_timeconst').perform(context)}"
        f" cone:={LaunchConfiguration('mujoco_cone').perform(context)}"
        f" impratio:={LaunchConfiguration('mujoco_impratio').perform(context)}"
    )
    print(f"[dual_fr3_boxlift] MuJoCo start: {start_label}, box ({x:.4f}, {y:.4f}, yaw {math.degrees(yaw):.2f} deg), "
          f"size {size}")
    return {
        "mujoco_scene_extra_args": scene_args,
        "mujoco_initial_joint_positions": " ".join(f"{q:.6f}" for q in start_q),
        "mujoco_object_pose_body": "box",
        # With the vision stand-in, MuJoCo's exact pose goes to /box_pose_gt and only the noisy copy
        # reaches the policy on box_pose_topic.
        "mujoco_object_pose_topic": "/box_pose_gt" if _vision_sim(context)
        else LaunchConfiguration("box_pose_topic").perform(context),
    }


def _vision_sim(context):
    return (LaunchConfiguration("use_mujoco").perform(context).lower() == "true"
            and LaunchConfiguration("mujoco_vision_sim").perform(context).lower() == "true")


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
            # effort (Isaac PD torque) or position (1 kHz tracker -> Franka joint impedance)
            "command_interface": LaunchConfiguration("command_interface"),
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
                "goal_xy_offset_m": ParameterValue(LaunchConfiguration("goal_xy_offset_m"), value_type=list[float]),
                "goal_xy_reference": LaunchConfiguration("goal_xy_reference"),
                "start_pose": LaunchConfiguration("start_pose"),
                "box_size_m": ParameterValue(LaunchConfiguration("box_size_m"), value_type=list[float]),
                "box_pose_topic": LaunchConfiguration("box_pose_topic"),
                "box_position_offset_m": ParameterValue(LaunchConfiguration("box_position_offset_m"),
                                                        value_type=list[float]),
                # log_task_name:="" -> dual_fr3_boxlift (MuJoCo) / dual_fr3_boxlift_real (robot)
                "log_task_name": PythonExpression(
                    [
                        "'", LaunchConfiguration("log_task_name"), "' or ('dual_fr3_boxlift' if '",
                        LaunchConfiguration("use_mujoco"),
                        "'.lower() == 'true' else 'dual_fr3_boxlift_real')",
                    ]
                ),
            },
        ],
    )
    if not _vision_sim(context):
        return [controller, policy]
    vision = Node(
        package="fr3_husky_nn_policy",
        executable="box_pose_vision_sim",
        name="box_pose_vision_sim",
        output="screen",
        parameters=[{
            "input_topic": "/box_pose_gt",
            "output_topic": LaunchConfiguration("box_pose_topic"),
            "delay_s": ParameterValue(LaunchConfiguration("mujoco_vision_delay_s"), value_type=float),
            "position_bias_m": ParameterValue(LaunchConfiguration("mujoco_vision_bias_m"), value_type=list[float]),
            "drop_probability": ParameterValue(LaunchConfiguration("mujoco_vision_drop_probability"), value_type=float),
        }],
    )
    return [controller, policy, vision]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", description="NPZ from export_boxlift_actor.py (required)"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("goal_z_m", default_value="0.15",
                                  description="box-center lift height in base (trained 0.15-0.35)"),
            DeclareLaunchArgument("goal_xy_offset_m", default_value="[0.0, 0.0]",
                                  description="goal x/y = box start x/y + this (base); checked against the "
                                              "manifest goal.xy_offset_range_base (old checkpoints: 0 only) "
                                              "or goal.xy_range_base (absolute goal x/y)"),
            DeclareLaunchArgument("goal_xy_reference", default_value="box_start",
                                  description="what goal_xy_offset_m is added to: box_start (box x/y at start_policy) "
                                              "or range_center (center of the trained goal.xy_range_base)"),
            DeclareLaunchArgument("start_pose", default_value="manifest",
                                  description="manifest (the checkpoint's start.mode: ready pose or pre-grasp table row) "
                                              "or hover (fixed hover pose of -Curriculum checkpoints)"),
            DeclareLaunchArgument("log_task_name", default_value="",
                                  description="data/<name>/<session> logs and experi_result/<name>/results.csv; "
                                              "empty: dual_fr3_boxlift (MuJoCo) or dual_fr3_boxlift_real"),
            DeclareLaunchArgument("box_size_m", default_value="[0.410, 0.310, 0.280]",
                                  description="physical box (x, y squeezed, z) in base; must match the policy's box"),
            DeclareLaunchArgument("box_pose_topic", default_value="/box_pose"),
            DeclareLaunchArgument("box_position_offset_m", default_value="[0.0, 0.0, 0.0]",
                                  description="added to the box position in base before the policy sees it "
                                              "(real robot with /box_tf: [0.0, 0.0, -0.03])"),
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("command_interface", default_value="effort",
                                  description="effort or position; the policy must be trained for that interface"),
            DeclareLaunchArgument("load_mobile", default_value="true"),
            DeclareLaunchArgument("use_mujoco", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_move_group", default_value="false"),
            # MuJoCo only (use_mujoco:=true)
            DeclareLaunchArgument("mujoco_start_row", default_value="1919",
                                  description="pre-grasp table row for the arm and box start (model_2999: A 1919)"),
            DeclareLaunchArgument("mujoco_hover_box", default_value="[0.45, 0.0, 0.0]",
                                  description="start_pose:=hover only: MuJoCo box start [x, y (base, m), yaw (deg)]"),
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
            DeclareLaunchArgument("mujoco_cone", default_value="elliptic",
                                  description="friction cone: elliptic, or pyramidal (MuJoCo default, creeps in grasps)"),
            DeclareLaunchArgument("mujoco_impratio", default_value="10",
                                  description="frictional-to-normal impedance ratio; 1 is the MuJoCo default"),
            DeclareLaunchArgument("mujoco_vision_sim", default_value="false",
                                  description="feed the policy a vision-like box pose (noise, delay, drops) "
                                              "instead of MuJoCo's exact pose (box_pose_vision_sim)"),
            DeclareLaunchArgument("mujoco_vision_delay_s", default_value="0.03",
                                  description="vision stand-in: capture-to-receive delay [s]"),
            DeclareLaunchArgument("mujoco_vision_bias_m", default_value="[0.0, 0.0, 0.0]",
                                  description="vision stand-in: constant position offset in base [m]"),
            DeclareLaunchArgument("mujoco_vision_drop_probability", default_value="0.0",
                                  description="vision stand-in: probability that a frame is not published"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
