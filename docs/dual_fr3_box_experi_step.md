# dual FR3 box lift 실로봇 실험 순서 (Pregrasp box410 model_2999)

2026-11-03 기준이다. 학습한 시작 자세로 팔을 먼저 보내고, 박스를 놓은 뒤 RL 정책을 켜서 조이기와 들기를 본다.

- 팔 이동(MoveIt, `move_to_joint`)과 RL 노드는 따로 실행한다. 둘은 서로 모른다.
- 비전은 아직 없다. 박스 pose는 손으로 놓은 위치를 고정 publish한다(4번). 나중에 비전 노드로 바꾼다.
- 좌표계와 박스 놓는 위치: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)
- 실로봇 전에 같은 노드로 MuJoCo에서 돌려 보는 절차는 맨 아래 [Isaac → MuJoCo sim-to-sim](#isaac--mujoco-sim-to-sim)에 있다.

| 항목 | 경로 |
|---|---|
| 정책 | `~/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz` |
| 시작 자세 후보 | `~/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_box410_model2999_start_poses.yaml` (이 문서 맨 아래에도 정리) |
| 노드 설정 | `fr3_husky_nn_policy/config/dual_fr3_boxlift.yaml` |

- 정책: `dual_fr3_lab`의 `Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Pregrasp-v0`, 3000 iter.
- sim 결과: 성공 100%, 성공까지 1.4 s, 최대 판 힘 16 N. 아래 후보 다섯 자세에서 시작해도 100%였다.
- 정책 제어: 20 Hz, 관절 목표 = 측정 q + ±0.1 rad, base frame, 박스 410 × 310 × 280 mm, 0.4~0.6 kg.

## 0. 준비 (한 번)

1. 시작 검사는 설정을 바꿀 필요가 없다(2026-11-03 노드 수정).
   - 노드가 npz 매니페스트의 `start.mode`를 보고 검사 방식을 고른다. 이 정책은 `pregrasp_table`이다.
   - 학습 때처럼 "팔이 표의 한 줄 자세에 있고, 박스가 같은 줄의 위치에 있는지"를 본다. 표는 npz 안에 있다.
   - 허용 범위: 관절마다 0.03 rad(학습 노이즈 0.01 + `pregrasp_joint_margin_rad` 0.02), 박스 x·y ±1 cm, yaw ±0.05 rad(학습 노이즈 그대로).
   - `ready_tolerance_rad`는 이 정책에서 쓰지 않는다. 예전처럼 2.6으로 올리면 안 된다(팔 검사가 사실상 꺼진다).
2. 빌드:

   ```bash
   cd ~/jw_ws
   colcon build --symlink-install --allow-overriding mujoco_ros_hardware \
     --packages-select mujoco_ros_hardware fr3_husky_description fr3_husky_controller fr3_husky_nn_policy
   source install/setup.bash
   ```

   - husky-rcu PC 기준이다. `~/jw_ws`가 `~/js_ws/etri_ws/install` 위에 overlay로 올라가 있고, `mujoco_ros_hardware`는 jw_ws 복사본이 덮어쓴다(sim-to-sim 0번 참고).
   - 실로봇 터미널은 `.bashrc`의 `ROS_DOMAIN_ID=23`을 그대로 쓴다. MuJoCo를 같이 띄울 때만 다른 domain으로 바꾼다.

3. 바닥 표시: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)의 "실물에 놓는 방법"대로 표시한다.
   - O'(두 팔 장착점 중점을 바닥에 내린 점)를 찍는다.
   - O'에서 로봇 정면 방향으로 중심선을 긋는다.
   - 쓸 후보의 "로봇 쪽 면 x" 위치에 가로선을 긋는다(후보 A는 O'에서 0.255 m).
4. base 높이 확인: 바닥에서 팔 장착면까지 높이를 잰다.
   - sim 학습값은 0.405 m, 실물 URDF는 0.392 m다.
   - 4번의 박스 z가 이 값에 달려 있다: z = −(base 높이) + 0.140.

## 1. 터미널 1: 컨트롤러 + move_group + box lift 노드 (shadow)

```bash
source ~/jw_ws/install/setup.bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$HOME/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz \
  shadow_mode:=true launch_move_group:=true goal_z_m:=0.15
```

- `shadow_mode:=true`: 추론과 로그만 하고 로봇에는 명령을 보내지 않는다. 처음에는 반드시 이것으로 한다.
- `launch_move_group:=true`: 2번의 `move_to_joint`가 쓰는 MoveIt을 같이 띄운다.
- `goal_z_m`: 박스 중심을 올릴 높이(base frame)다. 학습 범위 0.15~0.35에서 가장 낮은 0.15로 시작한다(바닥에서 41.5 cm 들기).
- MuJoCo로 먼저 확인할 때는 맨 아래 [Isaac → MuJoCo sim-to-sim](#isaac--mujoco-sim-to-sim) 절차를 따른다(박스와 시작 자세가 자동으로 놓인다).

## 2. 터미널 2: 팔을 시작 자세로 (박스 없이)

```bash
source ~/jw_ws/install/setup.bash
# (1) ready 자세
ros2 run fr3_husky_task_manager move_to_joint
# (2) 시작 자세: 맨 아래 후보 중 하나. 후보 A (박스 중심 x 0.46, y 0)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0741 1.6246 -0.2056 -1.3818 -1.0845 1.6946 0.8721 --right-target-positions 0.0759 1.6278 0.2027 -1.3784 1.0821 1.6903 0.7089
```

- **박스를 놓기 전에** 움직인다. `MoveToJoint`는 박스를 모르므로, 박스가 먼저 있으면 판이 박스를 지나는 경로가 나올 수 있다.
- 기본 속도·가속도 배율은 0.1이다.
- 도착하면 판 중심이 바닥에서 약 19 cm(박스 중심 + 5 cm), 박스가 놓일 자리 양옆 면에서 4 cm 바깥에 선다. 판은 면 안에서 약 18° 돌아가 있어 바닥에서 약 11~28 cm를 덮는다.

## 3. 박스 놓기

- 후보의 박스 위치에 놓는다. 후보 A라면:
  - 로봇 쪽 면이 O'에서 0.255 m 앞이다.
  - 박스 중심이 중심선 위(y 0)다.
  - 410 mm 변이 중심선과 나란하다(yaw 0).
- 로봇 앞쪽(+x)에서 두 판 사이로 밀어 넣는다. 판과 박스 면 사이가 양쪽 4 cm씩이다.
- 허용 오차는 학습 노이즈만큼이다: 위치 ±1 cm, yaw ±3°. 노드가 시작할 때 이 범위를 검사한다.
  - 넓혀야 하면 `start_position_margin_m`, `start_yaw_margin_rad`만큼 더해진다. 학습 분포 밖이 되므로 결과 해석에 주의한다.

## 4. 터미널 3: 박스 pose publish (비전 대신 고정값)

```bash
source ~/jw_ws/install/setup.bash
# 후보 A, base 높이 0.392 m일 때: z = -0.392 + 0.140 = -0.252
ros2 topic pub -r 30 /box_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: 0.460, y: 0.0, z: -0.252}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

- 박스 **중심** pose를 base frame으로 보낸다. 박스 x축은 410 mm 변, y축은 310 mm 변 방향이다.
- 노드 시작 검사(`start_policy` 때 한 번):
  - 팔과 박스가 표의 같은 줄에 맞아야 한다. 관절마다 ±0.03 rad, 박스 x·y ±1 cm, yaw ±0.05 rad. 2번에서 보낸 후보 줄과 같은 줄이 잡혀야 한다(응답 message에 줄 번호가 나온다).
  - 박스 z가 학습값 −0.265에서 ±0.03 안이어야 한다.
- **주의: 고정값은 박스가 들려도 바뀌지 않는다.** 정책은 매 step 박스 위치·자세를 관측하므로, 들기 시작한 뒤의 관측이 실제와 달라진다.
  - shadow 확인에는 문제없다.
  - 명령 모드로 실제로 들 때는 비전이 붙기 전까지 결과를 학습 분포 밖으로 보고 해석한다.
- 박스 pose가 0.30 s 넘게 안 들어오면 노드가 fault를 낸다(`box_pose_timeout_s`). publish를 계속 켜 둔다.

## 5. 터미널 4: 정책 시작과 정지

```bash
source ~/jw_ws/install/setup.bash
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
# 정지
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- 시작이 거부되면 응답 message에 이유가 나온다.
  - 예: `not at a trained pre-grasp start: nearest table row 1919 (...) has box x/y 0.015 m off (allowed 0.010)`, 박스 z가 바닥이 아니다.
  - 가장 가까운 표 줄과, 팔·박스가 각각 얼마나 벗어났는지 나온다. 팔이 벗어났으면 `move_to_joint`를 다시, 박스가 벗어났으면 박스를 다시 놓는다.
- 시작되면 응답 message와 `metadata.json`(`start_table_row`)에 잡힌 줄 번호가 남는다.
- 세션마다 로그 폴더(`metadata.json`, `trace.csv`)가 생긴다(`log_root`, 비우면 기본 data 경로).
- shadow에서 확인할 것:
  - 정책 출력 크기(관절 목표 변화 ≤ 0.1 rad).
  - 첫 몇 step 동안 판이 박스 쪽으로 다가가는 방향의 목표가 나오는지.
  - fault가 없는지.

## 6. 명령 모드

1. 터미널 1을 끄고 `shadow_mode:=false`로 다시 띄운다.
2. 2~5번을 반복한다.

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$HOME/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz \
  shadow_mode:=false launch_move_group:=true goal_z_m:=0.15
```

- sim에서는 시작 후 약 1.4 s 안에 조여서 goal 높이까지 든다. 판 최대 힘은 약 16 N이다.
- 학습 에피소드는 4 s다. `max_duration_s: 0.0`이면 `stop_policy`까지 계속 돈다.
- **정지하면 조임이 풀려 박스가 떨어진다**(아직 hold 모드 없음). 박스 아래를 받치거나 낮은 goal로 시험한다.
- 알려진 거동: 들 때 판을 V자로 최대 약 27° 기울여 아래 모서리로 받친다. 시작 자세에서 판이 면 안에서 약 18° 돌아가 있다. 둘 다 이 checkpoint의 특성이고, 시작 자세 표를 고친 재학습(RollAlign)이 진행 중이다.

---

## MoveIt 시작 자세 후보 (model_2999 전용)

- `dual_fr3_lab/scripts/tools/pregrasp_start_poses.py`로 정책 npz 안의 학습 표에서 뽑았다.
  - 요청한 박스 위치에 가장 가까운 표 줄을 고른다. 거리 기준은 max(|dxy| / 1 cm, |dyaw| / 0.05 rad)다.
- **model_2999 전용이다.** RollAlign 재학습 모델은 그 모델의 npz에서 다시 뽑는다:
  `python scripts/tools/pregrasp_start_poses.py <npz> <yaml> --box X Y YAW_DEG ...`
- 박스 중심 z는 sim 기준 −0.265다. 실물 base 높이가 다르면 4번의 z를 그에 맞춘다.
- 관절 순서는 각 팔 j1~j7, 단위는 rad다.
- sim 확인: 다섯 줄에서 노이즈 없이 500 env, 질량·마찰 DR과 외란을 켜고 돌렸다. 성공 100%, FR3 한계 여유 최소 0.115 rad였다.

| 후보 | 표 줄 | 박스 중심 x, y [m], yaw | 로봇 쪽 면 x [m] | 관절 여유 [rad] |
|---|---|---|---|---|
| **A** (첫 시험) | 1919 | 0.460, −0.001, 0.0° | 0.255 | 0.106 |
| B | 3788 | 0.477, +0.000, 0.0° | 0.272 | 0.083 |
| C | 3925 | 0.498, −0.003, −0.3° | 0.293 | 0.052 |
| D | 720 | 0.479, +0.052, 0.0° | 0.274 | 0.066 |
| E | 2228 | 0.481, −0.051, +0.3° | 0.276 | 0.061 |

관절 여유는 Margin 한계(FCI − 0.05 rad)까지 거리다.

**A** (박스 0.460, 0.00)
```bash
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0741 1.6246 -0.2056 -1.3818 -1.0845 1.6946 0.8721 --right-target-positions 0.0759 1.6278 0.2027 -1.3784 1.0821 1.6903 0.7089
```
`/box_pose` position x 0.460, y −0.001

**B** (박스 0.477, 0.00)
```bash
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0892 1.6507 -0.2084 -1.317 -1.0874 1.6777 0.7852 --right-target-positions 0.0884 1.6461 0.209 -1.3172 1.0877 1.6789 0.7826
```
`/box_pose` position x 0.477, y 0.000

**C** (박스 0.498, 0.00)
```bash
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.1061 1.6813 -0.2238 -1.2225 -1.0799 1.6646 0.7973 --right-target-positions 0.092 1.6755 0.2237 -1.2327 1.0804 1.6675 0.745
```
`/box_pose` position x 0.498, y −0.003, yaw −0.3°

**D** (박스 0.479, +0.05)
```bash
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0047 1.668 -0.1967 -1.248 -1.008 1.6223 0.8554 --right-target-positions 0.1696 1.6431 0.2345 -1.3437 1.1591 1.7299 0.6958
```
`/box_pose` position x 0.479, y 0.052

**E** (박스 0.481, −0.05)
```bash
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.1686 1.6435 -0.2317 -1.3432 -1.1705 1.7297 0.7869 --right-target-positions 0.0095 1.6732 0.1953 -1.2286 1.0038 1.6137 0.7589
```
`/box_pose` position x 0.481, y −0.051, yaw +0.3°

---
---

# Isaac → MuJoCo sim-to-sim

Isaac에서 학습한 정책을 MuJoCo에서 inference한다. 실로봇과 **같은 노드, 같은 npz, 같은 컨트롤러**를 쓰고, 로봇과 박스만 MuJoCo로 바꾼다.

```
MuJoCo (로봇 + 박스 물리) ──/dual_fr3/joint_states, /box_pose(GT 30 Hz)──▶ ppo_boxlift_policy_node (npz, 20 Hz)
        ▲                                                                       │
        └──── dual_fr3_action_controller (isaac_relative_control PD) ◀── 관절 목표 ┘
```

목적은 두 가지다.

- 배포 경로 확인: 매니페스트 검사, 시작 검사, 20 Hz tick, 명령 전송, 로그까지 실로봇과 같은 코드가 도는지 본다.
- 엔진 강건성 확인: 접촉 모델이 다른 MuJoCo에서도 정책이 박스를 드는지 본다. MuJoCo에서 실패해도 바로 정책 문제로 보지 않는다. 접촉 파라미터 탓일 수 있다.

## 실로봇 절차와 다른 점

| 항목 | 실로봇 (위 0~6번) | MuJoCo |
|---|---|---|
| 팔 시작 자세 | `move_to_joint`로 이동 | launch가 npz 표의 한 줄로 바로 시작 (`mujoco_start_row`) |
| 박스 놓기 | 손으로 | launch가 같은 줄의 박스 pose로 놓음 |
| `/box_pose` | `ros2 topic pub` 고정값 (나중에 비전) | MuJoCo가 GT를 30 Hz로 publish. **손으로 publish하지 않는다** |
| 들기 중 박스 관측 | 고정값이라 실제와 다름 | 실제 박스를 따라감 |
| 중력 | 로봇은 FCI가 보상 | 로봇 body는 `gravcomp`, 박스만 중력을 받음 |
| base 높이 | 실측 필요 | 학습값 0.405 m (`mujoco_mount_z:=0.2727`) |

## 0. 준비 (한 번)

1. MuJoCo 하드웨어 패키지: 박스 pose publisher가 들어간 `mujoco_ros_hardware`(`boxlift-object-pose` 브랜치)가 workspace에 있어야 한다.
   - husky-rcu PC: `~/jw_ws/src/mujoco_ros_hardware` (js_ws 버전을 복사해 overlay로 빌드했다).
   - 이 브랜치가 없으면 launch는 뜨지만 `/box_pose`가 안 나오고, 노드가 `waiting for the box pose`에서 멈춘다.
2. 빌드:

   ```bash
   cd ~/jw_ws
   colcon build --symlink-install --allow-overriding mujoco_ros_hardware \
     --packages-select mujoco_ros_hardware fr3_husky_description fr3_husky_controller fr3_husky_nn_policy
   source install/setup.bash
   ```

3. **`ROS_DOMAIN_ID`를 실로봇과 다르게 둔다.** 같은 domain이면 `/dual_fr3/joint_states`, `/fr3_policy_control`, `/policy_joint_command`가 실로봇과 겹쳐서 MuJoCo용 정책 노드가 실로봇에 명령을 보낼 수 있다.
   - 실로봇은 23(`.bashrc`), MuJoCo는 예를 들어 87을 쓴다. 아래 모든 터미널에서 똑같이 export한다.

## 1. 터미널 1: MuJoCo + 컨트롤러 + box lift 노드

```bash
source ~/jw_ws/install/setup.bash
export ROS_DOMAIN_ID=87
cd ~/jw_ws/src/fr3_husky
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz \
  use_mujoco:=true shadow_mode:=false launch_rviz:=false \
  mujoco_start_row:=1919 goal_z_m:=0.15
```

- `mujoco_start_row`: npz 안 pre-grasp 표의 줄 번호다. 팔 초기 관절각과 박스 x, y, yaw를 이 줄에서 읽는다.
  - model_2999 후보: A 1919, B 3788, C 3925, D 720, E 2228 (맨 위 "MoveIt 시작 자세 후보"와 같은 줄).
  - 다른 정책(예: RollAlign)은 그 npz의 표에서 줄을 고른다. `model_path`와 `mujoco_start_row`만 바꾸면 된다.
- `shadow_mode:=true`로 띄우면 추론과 로그만 하고 팔은 움직이지 않는다. 배포 경로만 볼 때 쓴다.
- 뷰어 창이 뜬다. launch 로그에 다음 두 줄이 보이면 준비된 것이다.
  - `[dual_fr3_boxlift] MuJoCo start: table row 1919, box (0.4598, -0.0005, yaw 0.04 deg)`
  - `Object pose publisher started: body=box frame=base topic=/box_pose (30.0 Hz)`

| launch 인자 (MuJoCo 전용) | 기본값 | 의미 |
|---|---|---|
| `mujoco_start_row` | 1919 | 시작 표 줄 (팔 + 박스) |
| `mujoco_mount_z` | 0.2727 | Husky base_link 위 팔 장착면 높이. 0.2727 = 바닥에서 0.405 m(학습), 0.26 = 기존 URDF/MJCF(0.392 m) |
| `mujoco_box_mass` | 0.5 | 박스 질량 [kg] (학습 0.4~0.6) |
| `mujoco_box_friction` | 0.8 | 박스와 판의 마찰 (학습 박스 마찰 0.4~1.2) |
| `mujoco_box_condim` | 4 | 박스 접촉 차원 (3 미끄럼, 4 +비틀림, 6 +구름) |
| `mujoco_contact_timeconst` | 0.02 | 박스와 판의 solref time constant [s]. MuJoCo 기본 0.02, 작을수록 단단함 |

- 손목 D435i는 boxlift launch에서 항상 뺀다(`with_realsense:=false`). 학습과 SysID 모델에 없고, link8 끝 관성이 달라지기 때문이다. Azure Kinect(Husky 고정)는 남긴다.

## 2. 터미널 2: 정책 시작과 정지

```bash
source ~/jw_ws/install/setup.bash
export ROS_DOMAIN_ID=87
# 확인: 박스가 학습 위치에 있는지 (z -0.265 근처)
ros2 topic echo --once /box_pose
# 시작 -> 응답 message에 "pre-grasp table row 1919"
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
# 4 s(학습 에피소드 길이) 뒤 정지
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- 정지하면 실로봇처럼 조임이 풀려 박스가 떨어진다.
- 4 s를 넘기면 노드가 `Past the trained episode length` 경고를 낸다. 그 뒤는 학습하지 않은 구간이다.
- 다시 돌리려면 터미널 1을 끄고 다시 띄운다(박스와 팔을 처음 자리로 되돌리는 reset이 아직 없다).

## 3. 결과 보기

세션마다 `fr3_husky/data/dual_fr3_boxlift/<시각>/`에 `metadata.json`과 `trace.csv`가 생긴다. `trace.csv`의 `obs_*`는 정책 입력 그대로다.

| 열 | 내용 |
|---|---|
| `obs_0`~`obs_13` | 관절각 − 기본 자세 |
| `obs_14`~`obs_27` | 관절 속도 |
| `obs_28`~`obs_30` | 박스 중심 위치 (base) |
| `obs_31`~`obs_37` | goal (x, y, z, 쿼터니언) |
| `obs_38`~`obs_51` | 직전 행동 |
| `obs_52`~`obs_55` | 박스 쿼터니언 (base, xyzw) |

박스 궤적 요약:

```bash
python3 - <<'PY' ~/jw_ws/src/fr3_husky/data/dual_fr3_boxlift/<시각>/trace.csv
import csv, math, sys
for r in list(csv.DictReader(open(sys.argv[1])))[::5]:
    x, y, z = (float(r[f"obs_{i}"]) for i in (28, 29, 30))
    qx, qy = float(r["obs_52"]), float(r["obs_53"])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))
    print(f"t {float(r['elapsed_s']):.2f}  box x {x:.3f} y {y:.3f} z {z:.3f}  tilt {tilt:.1f} deg")
PY
```

Isaac 기준값(위 "후보 다섯 줄" sim 확인): 성공 100%, 성공까지 1.4 s, 끝 기울기 4.7°, 박스 yaw 0.4°, 최대 판 힘 16 N.

## 4. 첫 결과 (2026-11-03, husky-rcu, 후보 A, 마찰 0.8, 질량 0.5 kg, 1회)

- 배포 경로: 시작 자세 오차 5e-7 rad, 시작 검사 1919번 줄 통과, tick 50 ms, 추론 0.2 ms, fault 없음.
- 들기: 1.3 s에 박스 중심 z 0.139(goal 0.15)까지 들고 4 s까지 놓치지 않았다.
- Isaac과 다른 점: 들면서 박스가 앞으로 약 8 cm 밀렸다(x 0.46 → 0.54). 들 때 최대 기울기 38°, 1.8~2.3 s에 3~5°로 안정됐다가 4 s에 25°까지 다시 기울었다.
- 접촉 파라미터(마찰, condim 4, MuJoCo 기본 solref)는 아직 Isaac `env.yaml`과 맞추지 않았다. 위 차이가 정책 때문인지 접촉 모델 때문인지는 마찰, condim sweep으로 가려야 한다.

### 접촉 sweep (2026-11-03, 후보 A, 질량 0.5 kg, 조건마다 1회, 4.5 s)

| 조건 | z @1.4 s | z @4 s | 최대 기울기 | 4 s 기울기 | x 최대 밀림 | 떨어짐 |
|---|---|---|---|---|---|---|
| 마찰 0.8, condim 4, tc 0.02 (기존) | 0.133 | −0.265 | 43.7° | − | 0.119 m | **약 2 s에 떨어짐** |
| 마찰 1.2, condim 4, tc 0.02 | 0.142 | 0.132 | 28.5° | 2.8° | 0.068 m | 아니오 |
| 마찰 0.8, condim 6, tc 0.005 (단단함) | 0.128 | −0.265 | 39.9° | − | 0.100 m | **약 2.5 s에 떨어짐** |
| 마찰 1.2, condim 6, tc 0.005 | 0.142 | 0.133 | 27.4° | 2.7° | 0.055 m | 아니오 |

- **마찰이 결정적이다.** 마찰 1.2면 4 s 기울기가 2.7~2.8°로 Isaac(4.7°)과 비슷해진다. 접촉을 단단하게 하는 것(condim 6, tc 0.005)만으로는 떨어진다.
- 마찰 0.8은 경계 조건이다. 같은 설정의 앞선 실행은 4 s까지 버텼고 이번에는 약 2 s에 떨어졌다.
- 학습 DR은 마찰 0.4~1.2에서 성공 100%였다. MuJoCo는 같은 마찰 숫자에서 더 쉽게 미끄러진다(pyramidal cone, soft contact, Isaac 쪽 마찰 결합 방식 미확인).
- 최대 기울기 27~28°는 마찰 1.2에서도 남는다. 문서에 적힌 model_2999의 V자 판 기울기(약 27°)와 같은 거동이다.
- 실로봇에 대한 의미: 판과 박스 사이 실제 마찰이 이 정책에 결정적이다. 실물 마찰계수를 재야 한다(박스를 판 위에 놓고 기울여 미끄러지는 각 θ에서 μ = tan θ).

## MuJoCo 씬 구성 (참고)

- 씬: `fr3_husky_description/mjcf/dual_fr3.xml.xacro`. boxlift launch가 아래 xacro 인자를 넘긴다. 기본값(Reach용 씬)은 바뀌지 않는다.
  - `gravity_z:=-9.81`, `mobile_mount_z`, `with_box:=true`, `box_x/box_y/box_yaw`(표 줄), `box_mass`, `box_friction`, `plate_friction`(= 박스 마찰. MuJoCo는 두 geom 중 큰 마찰을 쓴다), `box_condim`(기본 4), `contact_timeconst`(박스와 판 solref, 기본 0.02).
- 팔 초기 자세: `fr3_action_controller.launch.py`의 `mujoco_initial_joint_positions`가 `robot_description`의 ros2_control `initial_value`를 바꾼다(MuJoCo일 때만).
- 박스 pose: `mujoco_object_pose_body:=box`이면 `mujoco_ros_hardware`가 박스 body pose를 `base` 기준 `PoseStamped`로 `/box_pose`에 30 Hz로 publish한다. stamp는 publish 시각이다.
