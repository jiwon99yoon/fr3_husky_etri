# dual FR3 box lift 실로봇 실험 순서 (Pregrasp box410 model_2999)

2026-11-03 기준이다. 학습한 시작 자세로 팔을 먼저 보내고, 박스를 놓은 뒤 RL 정책을 켜서 조이기와 들기를 본다.

- 팔 이동(MoveIt, `move_to_joint`)과 RL 노드는 따로 실행한다. 둘은 서로 모른다.
- 비전은 아직 없다. 박스 pose는 손으로 놓은 위치를 고정 publish한다(4번). 나중에 비전 노드로 바꾼다.
- 좌표계와 박스 놓는 위치: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)

| 항목 | 경로 |
|---|---|
| 정책 | `/home/dyros/etri_ws/src/fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz` |
| 시작 자세 후보 | `/home/dyros/etri_ws/src/fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999_start_poses.yaml` (이 문서 맨 아래에도 정리) |
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
   cd /home/dyros/etri_ws
   colcon build --packages-up-to fr3_husky_nn_policy fr3_husky_controller
   source install/setup.bash
   ```

3. 바닥 표시: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)의 "실물에 놓는 방법"대로 표시한다.
   - O'(두 팔 장착점 중점을 바닥에 내린 점)를 찍는다.
   - O'에서 로봇 정면 방향으로 중심선을 긋는다.
   - 쓸 후보의 "로봇 쪽 면 x" 위치에 가로선을 긋는다(후보 A는 O'에서 0.255 m).
4. base 높이 확인: 바닥에서 팔 장착면까지 높이를 잰다.
   - sim 학습값은 0.405 m, 실물 URDF는 0.392 m다.
   - 4번의 박스 z가 이 값에 달려 있다: z = −(base 높이) + 0.140.

## 1. 터미널 1: 컨트롤러 + move_group + box lift 노드 (shadow)

```bash
source /home/dyros/etri_ws/install/setup.bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=/home/dyros/etri_ws/src/fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz \
  shadow_mode:=true launch_move_group:=true goal_z_m:=0.15
```

- `shadow_mode:=true`: 추론과 로그만 하고 로봇에는 명령을 보내지 않는다. 처음에는 반드시 이것으로 한다.
- `launch_move_group:=true`: 2번의 `move_to_joint`가 쓰는 MoveIt을 같이 띄운다.
- `goal_z_m`: 박스 중심을 올릴 높이(base frame)다. 학습 범위 0.15~0.35에서 가장 낮은 0.15로 시작한다(바닥에서 41.5 cm 들기).
- MuJoCo로 먼저 확인할 때는 `use_mujoco:=true`를 붙인다. MuJoCo에는 박스가 없으므로 shadow로만 본다.

## 2. 터미널 2: 팔을 시작 자세로 (박스 없이)

```bash
source /home/dyros/etri_ws/install/setup.bash
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
source /home/dyros/etri_ws/install/setup.bash
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
source /home/dyros/etri_ws/install/setup.bash
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
  model_path:=/home/dyros/etri_ws/src/fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz \
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
