# Cabinet book-insertion policy deployment

Trained table/cabinet/book layout in `base` and in both FR3 `link0` frames, with figures:
[../docs/dual_fr3_cabinet_book_real_setup.md](../docs/dual_fr3_cabinet_book_real_setup.md).

This package mirrors the deployment structure of `ppo_boxlift_policy_node`: a portable
NumPy actor NPZ, an embedded JSON manifest, a ROS 2 node, operator start/stop services,
shadow mode by default, and per-step CSV traces. The checkpoint actor is 50 -> 14,
ELU hidden layers `[512, 256, 128]`, deterministic tanh output, and a 20 Hz policy.

## Export a checkpoint

Run the exporter in the training environment with the `fr3_husky_nn_policy` source
package on `PYTHONPATH` (the script uses PyTorch and PyYAML, but the robot node does not):

```bash
export PYTHONPATH=/home/dyros/etri_ws/src/fr3_husky_etri/fr3_husky_nn_policy:$PYTHONPATH
python3 -m fr3_husky_nn_policy.export_cabinet_book_insert_actor \
  /home/dyros/dual_fr3_lab/logs/rsl_rl/<experiment>/<run>/model_XXXX.pt \
  /home/dyros/etri_ws/src/fr3_husky_etri/fr3_husky_nn_policy/models/cabinet_book_insert_model_XXXX.npz
```

The exporter reads `params/env.yaml` and `params/agent.yaml` beside that checkpoint;
it refuses a different observation order, joint order, action type, actor size, or
normalization setup. It records the reset door angle too, and `start_policy` refuses a
checkpoint if the measured door angle is outside that checkpoint's start-angle tolerance
(open-door pretraining and full-task runs therefore cannot be silently interchanged). Do not
export a checkpoint without its matching run metadata.

## ROS input contract

* `/dual_fr3/joint_states`: the 14 FR3 arm joints, in either order by name.
* `/cabinet_book_insert/book_pose`: `PoseStamped` of the **book center**, expressed in
  `base`. Quaternion is `(x, y, z, w)`.
* `/cabinet_book_insert/door_angle`: `std_msgs/Float64`, radians, positive when opening.
* `/cabinet_book_insert/left_tcp_pose`: diagnostic `PoseStamped` for hinge calibration.

`cabinet_book_insert_fk_node` derives book pose from the robot-state-publisher TF for the
right `link7`, then applies the training's link7-to-UMI and UMI-to-book grasp transforms.
It derives door angle from the left UMI TCP orbit around a vertical cabinet hinge. This
matches the simulation's rigid grasp assumptions and does not use a camera. The UMI and
grasp transforms default to the cabinet training configuration; change them if the physical
tool/grasp differs.

### Cabinet hinge calibration

The estimator intentionally does not publish a door angle until its hinge and closed-door
TCP reference are configured. In the deployment YAML, set `hinge_position_base` to the
measured cabinet hinge center and `left_tcp_closed_position_base` to the left TCP position
with the door closed and the gripper holding the handle. Both are `[x, y, z]` in the same
`base` frame as the policy. The FK node publishes the current left TCP to
`/cabinet_book_insert/left_tcp_pose`; place the door closed, inspect that topic, and use its
position as the closed reference. Measure the hinge in that same robot base frame.

Launch only the state estimator while validating it:

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_fk.launch.py
ros2 topic echo /cabinet_book_insert/book_pose
ros2 topic echo /cabinet_book_insert/left_tcp_pose
ros2 topic echo /cabinet_book_insert/door_angle
```

The book pose requires `robot_state_publisher` TF for `left_fr3_link7` and
`right_fr3_link7`. Before starting inference, open the door a small known amount and check
that `/door_angle` increases (default `door_angle_sign: -1.0` matches the trained layout, hinge at robot left; flip it if the angle decreases). Also compare the
book center and orientation against a measured pose with the gripper holding it. Validate
these estimates in shadow mode before enabling command mode.

## Shadow inference

Build and source the ROS workspace, then launch with command publication disabled:

```bash
cd /home/dyros/etri_ws
colcon build --packages-select fr3_husky_nn_policy
source install/setup.bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_fk.launch.py
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py \
  model_path:=/path/to/cabinet_book_insert_model_XXXX.npz shadow_mode:=true
ros2 service call /ppo_cabinet_book_insert_policy_node/start_policy std_srvs/srv/Trigger
```

### Start pose check (2026-11-06)

The exporter embeds the run's start table (121 rows: joints in node order and book centre in
`base`; the open-door stage with the left arm at its 90 deg pose). `start_policy` refuses unless
every joint is within `start_joint_tolerance_rad` (0.10) of one row, and logs the nearest row's
`ros2 run fr3_husky_task_manager move_to_joint ...` command. NPZs exported before this have no rows:
shadow mode still starts, command mode refuses; re-export them.

### Controller speed (2026-11-06)

The goal fields are node parameters now (`max_actuator_step_rad` 0.001, `joint_velocity_scale` 0.10,
`joint_acceleration_scale` 0.20, `relative_target_refresh_hz` 1000; the boxlift values). Before,
the node sent `max_actuator_step_rad` 0.03, which the controller rejects (it accepts (0, 0.01]).
The controller limits j1-j4 to 2.62 x `joint_velocity_scale` rad/s (0.10 -> 0.26 rad/s). The node
logs this next to the speed the checkpoint was trained for (action clip / policy dt) and warns
when the robot is slower:

| checkpoint | trained joint step | trained speed | `joint_velocity_scale` for that speed on j1-j4 |
|---|---|---|---|
| v2 open-door 2499 / closed-door 7498 | 0.10 rad / 50 ms | 2.0 rad/s | 0.76 and `max_actuator_step_rad` >= 0.002 (not recommended; run slower and expect lag) |
| v3 Slow | 0.025 rad / 50 ms | 0.5 rad/s | 0.19 |

Shadow mode logs observation, action, and target without publishing robot commands.
Each session is recorded under `fr3_husky_nn_policy/data/cabinet_book_insert/`.
The node refuses to start with missing/stale observations and stops at the checkpoint's
trained episode horizon. Command mode uses the existing `/fr3_policy_control` controller
path and should only be considered after the FK estimates and shadow trace are compared
against simulation. The node does not command either gripper.
