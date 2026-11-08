# dual FR3 캐비닛 책 넣기 실로봇 실험 순서 (2026-11-06)

배치(책상 높이, 수납장·책 위치, link0 기준 좌표)는 [dual_fr3_cabinet_book_real_setup.md](dual_fr3_cabinet_book_real_setup.md)(45 cm 책상)와 [dual_fr3_cabinet_book_layout_75cm.md](dual_fr3_cabinet_book_layout_75cm.md)(75 cm 책상), 노드 계약과 export는 [../fr3_husky_nn_policy/CABINET_BOOK_INSERT_DEPLOYMENT.md](../fr3_husky_nn_policy/CABINET_BOOK_INSERT_DEPLOYMENT.md)에 있다.

## 정책 (`fr3_husky_nn_policy/models/`)

| 파일 | 시작 상태 | sim 결과 |
|---|---|---|
| `cabinet_book_insert_75cm_door_first_model_2998.npz` (**75 cm 책상**) | 문 닫힘 | 121개 중 121 성공, 책이 입구에 닿을 때 문 중앙 97° (최소 92°). 아래 "75 cm 책상: DoorFirst 2998" 절 |
| `cabinet_book_insert_open_door_upright_pretrain_model_2499.npz` | 문 90° 열림, 왼손이 열린 문의 손잡이를 잡음 | 영상 3/3 성공, 약 1.5 s에 넣음, 끝 기울기 2.5°, 깊이 11.8 cm |
| `cabinet_book_insert_closed_door_stage2_model_7498.npz` | 문 닫힘 | 121개 시작 위치 중 120 성공, 첫 성공 중앙 1.2 s, 끝 기울기 중앙 4.1°, 깊이 12.9 cm |
| `cabinet_book_insert_v1_model_14999.npz` | 문 닫힘 | 121개 중 성공 0 (책을 약 31° 기울인 채 깊이 12.6 cm까지 넣음), 비교용 |

- 2998은 75 cm 책상 전용이다. 나머지 셋(2499, 7498, v1)은 45 cm 책상 전용이다.
- 2499, 7498은 v2(codex 2단계 학습)다. 2499가 1단계, 7498이 그 이어 학습이다.
- v1(`v1_5070`, 15000 iteration)은 보상이 책 위치만 보고 기울기를 안 봐서, 책을 기울인 채 넣는 국소해에 머물렀다(sim 성공 0, book_depth 1.68/2.0, book_lane 0.85/1.0). v2와 비교하려고 export했다(2026-11-07). 관측·행동 형식은 v2와 같다.
- 실로봇 관찰(2026-11-07): 7498이 문을 끝까지 열기 전에 책을 넣으려 했다. sim(121개 시작 위치, 결정적 행동)에서도 같고, v1도 마찬가지다(아래 표).

| | 책 앞면이 입구에 닿을 때 문 각도 | 책 중심이 입구를 지날 때 | 문 85° 도달 | 문 최대 | 끝 기울기 |
|---|---|---|---|---|---|
| 7498 | 중앙 37° (30~44°), 0.75 s | 42° (36~52°) | 1.75 s | 92° | 4° |
| v1 | 중앙 52° (46~70°), 1.15 s | 60° (53~79°) | 1.75 s | 119° | 31° |

- 책 보상은 문 60~80°에서 켜지지만, 그 전에 책을 옮기는 것을 막는 항이 없다. 그래서 두 정책 모두 문을 여는 동안 책을 미리 넣는다.
- **세 정책 모두 sim에서 관절 약 2 rad/s로 움직인다**(행동이 매 step 0.1 rad까지). **실로봇도 같은 속도로 움직인다.** 노드는 Isaac PD 모드(`isaac_relative_control`)로 제어기를 부르는데, 이 모드에서는 `joint_velocity_scale`·`max_actuator_step_rad`가 범위 검사에만 쓰이고 속도를 제한하지 않는다(2026-11-07 제어기 코드 확인). 더 느리게 하려면 느린 체크포인트(v3 Slow)를 써야 한다.
- 7498은 sim에서 손잡이에 중앙 56 N(최대 448 N)을 썼다. 실물 UMI 파지가 미끄러질 수 있다.

## box lift와 같은 것, 다른 것

| | box lift | 캐비닛 |
|---|---|---|
| 제어기 경로 | `/fr3_policy_control` action, `/policy_joint_command`, `isaac_relative_control` | 같음 |
| 제어기 limit | step 0.001, velocity scale 0.10, acceleration scale 0.20, refresh 1000 Hz | 같음 (launch 인자로 바꿀 수 있음) |
| 행동 | 20 Hz, q_meas + clip(0.1 a, ±0.1), `max_policy_target_delta_rad` 0.20 | 같음 |
| 물체 입력 | 비전 `/box_tf` (박스 pose) | **FK 노드**: 오른손 link7 TF → 책 중심 pose, 왼손 TCP가 경첩 주위로 돈 각도 → 문 각도 (카메라 없음) |
| 그리퍼 | 없음 (판) | **있음**: 손잡이와 책을 잡아야 해서 `load_gripper:=true` (기본). 정책은 그리퍼에 명령하지 않는다 |
| 시작 검사 | 박스 위치 → 표 줄 | 관절이 121개 시작 행 중 하나와 0.10 rad 안 + 문 각도가 시작 각도 ±0.2 rad |
| 시간 | 학습 에피소드 길이에서 정지 | 같음 (12 s) |

## 0. 준비 (한 번)

1. 이 PC의 변경을 로봇 PC 작업공간(`~/jw_ws/src/fr3_husky`)에 받는다(git pull).
2. 새 실행 파일이 생겼으니 빌드한다.
   ```bash
   cd ~/jw_ws && colcon build --symlink-install --packages-select fr3_husky_nn_policy && source install/setup.bash
   ```
3. 배치를 맞춘다(45 cm 책상. 75 cm 책상은 아래 "75 cm 책상: DoorFirst 2998" 절): 책상 윗면은 **팔 장착면 + 0.045 m**, 수납장 밑면 중심 (0.950, 0.100), 경첩 축 (0.7635, 0.2555), 손잡이 막대 축 (0.710, −0.035). 자세한 것은 배치 문서.
   - 실물 장착면은 바닥 위 약 0.38 m라, 학습 배치는 책상 0.425 m다. 45 cm 책상이면 2.5 cm 높다. 책과 수납장이 함께 올라가 상대 위치는 같으니 그대로 시험한다. 다만 책 pose(FK)의 z는 실제보다 2.5 cm 낮게 나온다(배치 문서 "바닥 높이 주의").

**모든 터미널 공통**

```bash
source ~/jw_ws/install/setup.bash
cd ~/jw_ws/src/fr3_husky
```

- 시작과 정지는 **따로** 보낸다.

## 1. 터미널 1: 제어기 + 정책 노드 (shadow)

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/cabinet_book_insert_closed_door_stage2_model_7498.npz \
  shadow_mode:=true launch_move_group:=true
```

- open-door 정책은 `model_path`만 `cabinet_book_insert_open_door_upright_pretrain_model_2499.npz`로 바꾼다.
- v1은 `model_path`만 `cabinet_book_insert_v1_model_14999.npz`로 바꾼다(문 닫힘 시작, 7498과 같음).
- 로그 확인:
  - `Joint speed: up to 2.00 rad/s per joint as trained ...`
  - `121 start rows`
- 기본값은 `load_gripper:=true`다. 그리퍼를 띄우지 않을 때만 `load_gripper:=false`를 준다.

## 2. 터미널 2: FK 노드 (책 pose, 문 각도)

```bash
ros2 run fr3_husky_nn_policy cabinet_book_insert_fk_node --ros-args \
  --params-file $PWD/fr3_husky_nn_policy/config/dual_fr3_cabinet_book_insert_fk.yaml \
  -p hinge_position_base:="[0.7635, 0.2555, 0.24]" \
  -p left_tcp_closed_position_base:="[0.7241, -0.035, 0.2259]"
```

- 두 값은 학습 배치(sim) 값이다. 실물을 배치 문서대로 놓았으면 그대로 쓰고, 재서 다르면 바꾼다.
  - `left_tcp_closed_position_base`: 문을 닫고 왼손이 손잡이를 잡은 상태의 `/cabinet_book_insert/left_tcp_pose`.
- `door_angle_sign` 기본은 −1이다. 학습 배치(경첩이 로봇 왼쪽)에서 문을 열면 +가 되게 맞춰 둔 값이다. 문을 조금 열었을 때 `/cabinet_book_insert/door_angle`이 커지는지 확인한다.

## 3. 터미널 3: 팔을 시작 자세로, 수납장과 책을 놓고 잡기

시작 행은 가운데인 **행 60**을 쓴다(책 중심 (0.63, −0.39, 0.156)). MoveIt은 수납장과 책을 모르므로, **팔을 움직이는 동안 수납장과 책을 치워 둔다.**

```bash
ros2 run fr3_husky_task_manager gripper_move --arm both --command open
ros2 run fr3_husky_task_manager move_to_joint     # ready

# closed-door 7498 · v1 14999 (문 닫힘), 행 60: 두 모델의 시작 표(121행)가 같다
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.2898 0.4389 -0.6271 -1.8862 1.4038 2.7138 -0.6641 \
  --right-target-positions -1.4168 1.1355 1.3881 -2.4121 -2.0347 2.6719 1.6555

# open-door 2499 (문 90°), 행 60: 왼팔만 다르다
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 1.3161 1.4299 -1.6901 -2.1074 0.6268 1.7453 2.1358 \
  --right-target-positions -1.4168 1.1355 1.3881 -2.4121 -2.0347 2.6719 1.6555
```

1. 수납장을 제자리에 놓는다. 손잡이 막대가 열린 왼손 손가락 사이에 오고, 손끝이 막대 축을 약 2 cm 지나게 한다.
   - closed-door 모델(7498, v1 14999)은 문을 닫은 채로 놓는다.
   - open-door 모델은 문을 90°까지 열고 놓는다. 이때 막대 축은 (0.473, 0.309)이다.
2. 책을 오른손 손가락 사이에 세운다. 밑면이 책상에 닿고, 면이 base 축과 평행해야 한다.
3. 잡는다(힘은 실물을 보고 조절).
   ```bash
   ros2 run fr3_husky_task_manager gripper_move --arm left --command grasp --force 30
   ros2 run fr3_husky_task_manager gripper_move --arm right --command grasp --force 30
   ```

다른 행을 쓰려면 아무 자세에서나 4번의 `start_policy`를 부른다. 그러면 가장 가까운 행의 `move_to_joint` 명령이 터미널 1 로그에 나온다.

## 4. 터미널 4: 입력 확인 → 시작 → 정지

```bash
ros2 topic echo --once /cabinet_book_insert/door_angle   # 닫힘(7498, v1) ≈ 0, open-door(2499) ≈ 1.57
ros2 topic echo --once /cabinet_book_insert/book_pose    # 행 60이면 ≈ (0.630, -0.390, 0.156), 방향 ≈ (0, 0, 0, 1)
ros2 service call /ppo_cabinet_book_insert_policy_node/start_policy std_srvs/srv/Trigger {}
```

```bash
# 정지 (따로 보낸다). 12 s가 되면 노드가 스스로 멈춘다
ros2 service call /ppo_cabinet_book_insert_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- 거부되는 경우
  - `door angle ... outside checkpoint start`: 문 각도가 시작 각도 ±0.2 rad 밖이다. FK 보정이나 부호를 확인한다.
  - `joints are ... from the nearest training start row`: 시작 자세가 행에서 벗어났다. 로그에 나온 `move_to_joint`로 옮긴다.
  - `book pose missing or stale`: FK 노드가 TF를 못 받는다(터미널 2 로그 확인).
- 로그: `fr3_husky_nn_policy/data/cabinet_book_insert/<시각>/trace.csv`(관측 50, 행동 14, 관절 목표 14), `metadata.json`.
  - shadow에서는 로봇이 움직이지 않으므로 정책이 매 step 같은 관측을 본다. 확인할 것은 첫 몇 step의 관측(책 위치, 문 각도)이 sim 시작값과 같은지와, 행동이 한쪽으로 꽉 차(±1) 있는지다.

## 5. 명령 모드

터미널 1을 `shadow_mode:=false`로 다시 띄운다. 비상정지를 손에 둔다.

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/cabinet_book_insert_open_door_upright_pretrain_model_2499.npz \
  shadow_mode:=false launch_move_group:=true
```

- open-door 2499부터 시작한다. 문을 여는 단계가 없어 손잡이 힘 위험이 작다.
- 속도: 시작하자마자 관절이 최대 약 2 rad/s로 움직인다. `joint_velocity_scale`·`max_actuator_step_rad`는 이 모드에서 속도를 바꾸지 않는다.
  - 실로봇(2026-11-07): 7498은 0.93 s에 관절 0.9 rad를 움직여 비상정지했다. v1 14999는 1.4 s에 1.4 rad를 움직이며 문을 42°까지 열다 Franka `cartesian_reflex`로 멈췄다. 로그는 `data/cabinet_book_insert/`.
- 손잡이 힘을 잴 센서가 없다. 왼손이 손잡이에서 미끄러지거나 수납장이 밀리면 바로 정지한다.

## 75 cm 책상: DoorFirst 2998 (2026-11-07)

실험실 책상(윗면 = 팔 장착면 + 0.345 m, 실측도 같음)에서 쓰는 정책이다. 배치는 [dual_fr3_cabinet_book_layout_75cm.md](dual_fr3_cabinet_book_layout_75cm.md)에 있다. 순서(0~5)는 위와 같고, **모델, FK 보정값, 시작 자세, 확인 값만 다르다.** 45 cm 모델(2499, 7498, v1)은 75 cm 책상에서 쓰지 않는다.

### 정책과 sim 결과

- 파일: `cabinet_book_insert_75cm_door_first_model_2998.npz`
- task `Isaac-Cabinet-BookInsert-DualFR3-UMI-Table75-DoorFirst-v0`
- 학습: 75 cm OpenDoor(문 90° 시작) 1000 iteration(`opendoor_75cm_5070` model_999) 뒤 DoorFirst로 2000 iteration 이어서 학습했다(run `door_first_75cm_5070`, 5070 PC).
- DoorFirst: 문이 80~90° 열릴 때까지 책 중심이 대기선 y −0.14(수납장 오른쪽 벽에서 오른쪽으로 4.5 cm) 오른쪽에 있어야 한다(`book_wait` 벌점). 성공에는 문 85° 이상이 필요하다.
- 책 무게는 학습 때 0.15~0.30 kg으로 무작위화했다(실물 0.25 kg).
- sim 결과 (121개 시작 위치, 결정적 행동, 12 s):

| | 값 |
|---|---|
| 성공 | 121/121 (끝까지 유지) |
| 문 85° 도달 | 중앙 1.15 s (1.05~1.25 s) |
| 책 앞면이 입구에 닿을 때 | 중앙 1.30 s, 문 중앙 97° (92~100°) |
| 책 중심이 입구를 지날 때 문 | 중앙 104° (101~107°) |
| 문 최대 | 111° |
| 책 기울기 | 입구에서 중앙 7° (최대 22°), 끝 8° |
| 깊이 | 중앙 12.7 cm (12.5~12.8 cm) |

- 7498은 책이 입구에 닿을 때 문이 37°였다. 2998은 문을 다 연 뒤 책이 들어간다.
- 속도는 7498과 같다. 관절이 최대 약 2 rad/s로 움직여, 1.3 s 안에 문을 97°까지 연다.
- 75 cm open-door 정책(1단계 model_999)은 export하지 않았다.

### 45 cm와 다른 값

| | 45 cm (2499, 7498, v1) | **75 cm (2998)** |
|---|---|---|
| 책상 윗면 (base z) | +0.045 | **+0.345** (실물 바닥에서 약 0.725 m) |
| 수납장 밑면 중심 (base x, y) | (0.950, 0.100) | **(0.850, 0.100)** |
| 손잡이 막대 축 (문 닫힘, base x, y) | (0.710, −0.035) | **(0.610, −0.035)** |
| FK `hinge_position_base` | [0.7635, 0.2555, 0.24] | **[0.665, 0.2555, 0.54]** |
| FK `left_tcp_closed_position_base` | [0.7241, −0.035, 0.2259] | **[0.6241, −0.035, 0.5259]** |
| 행 60 책 중심 (`book_pose`) | (0.630, −0.390, 0.156) | **(0.630, −0.390, 0.456)** |
| 책 목표 중심 | (0.892, 0.060, 0.240) | **(0.795, 0.060, 0.540)** |

- 손이 책과 손잡이를 잡는 방식은 같다. 그래서 FK 노드의 손-책 변환과 손-손잡이 변환은 그대로 쓴다.

### 실행

준비(git pull, 빌드)는 위 0번과 같다. 책상과 수납장은 위 표의 75 cm 값으로 놓는다.

**터미널 1: 정책 노드 (shadow)**

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/cabinet_book_insert_75cm_door_first_model_2998.npz \
  shadow_mode:=true launch_move_group:=true
```

- 로그에서 `121 start rows`와 `door_first_75cm_5070`을 확인한다.

**터미널 2: FK 노드 (75 cm 보정값)**

```bash
ros2 run fr3_husky_nn_policy cabinet_book_insert_fk_node --ros-args \
  --params-file $PWD/fr3_husky_nn_policy/config/dual_fr3_cabinet_book_insert_fk.yaml \
  -p hinge_position_base:="[0.665, 0.2555, 0.54]" \
  -p left_tcp_closed_position_base:="[0.6241, -0.035, 0.5259]"
```

- 실물을 재서 다르면 바꾼다. `left_tcp_closed_position_base`는 문을 닫고 왼손이 손잡이를 잡은 상태의 `/cabinet_book_insert/left_tcp_pose`다.

**터미널 3: 시작 자세 (행 60, 문 닫힘), 수납장과 책을 놓고 잡기**

```bash
ros2 run fr3_husky_task_manager move_to_joint     # ready
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.4463 -0.1901 -0.4999 -1.8995 0.3425 2.4281 0.1574 \
  --right-target-positions 0.7872 -0.4824 -0.5670 -2.3592 -0.0158 2.6981 0.4468
```

1. 팔을 옮기는 동안 수납장과 책을 치워 둔다.
2. 수납장을 문 닫힌 채로 놓는다. 손잡이 막대가 열린 왼손 손가락 사이에 와야 한다.
3. 책을 오른손 손가락 사이에 세운다(밑면이 책상에 닿고, 면이 base 축과 평행).
4. 양손으로 잡는다. 아래는 franka action을 직접 부르는 명령이다. `gripper_move`는 이 저장소에 없는 `GripperCommand` action을 부른다(11-07 lift 작업 때 확인).
   ```bash
   ros2 action send_goal /left_franka_gripper/grasp franka_msgs/action/Grasp \
     "{width: 0.0, epsilon: {inner: 0.08, outer: 0.08}, speed: 0.05, force: 30.0}"
   ros2 action send_goal /right_franka_gripper/grasp franka_msgs/action/Grasp \
     "{width: 0.0, epsilon: {inner: 0.08, outer: 0.08}, speed: 0.05, force: 30.0}"
   # 풀 때
   ros2 action send_goal /left_franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.05}"
   ros2 action send_goal /right_franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.05}"
   ```

**터미널 4: 입력 확인 → 시작 → 정지**

```bash
ros2 topic echo --once /cabinet_book_insert/door_angle   # ≈ 0
ros2 topic echo --once /cabinet_book_insert/book_pose    # ≈ (0.630, -0.390, 0.456), 방향 ≈ (0, 0, 0, 1)
ros2 service call /ppo_cabinet_book_insert_policy_node/start_policy std_srvs/srv/Trigger {}
ros2 service call /ppo_cabinet_book_insert_policy_node/stop_policy std_srvs/srv/Trigger {}   # 따로 보낸다
```

- 거부 메시지와 로그 위치는 위 4번과 같다.
- `book_pose` z가 0.156 근처면 45 cm 모델이나 45 cm 값이 섞인 것이다.

**명령 모드**

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_cabinet_book_insert_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/cabinet_book_insert_75cm_door_first_model_2998.npz \
  shadow_mode:=false launch_move_group:=true
```

- 비상정지를 손에 둔다. 시작하고 1.3 s 안에 문이 97°까지 열린다.
- 문은 111°까지 열린다. 문이 열리는 쪽(로봇 왼쪽 앞, 책상 앞 모서리 밖)을 비워 둔다.
- 왼손이 손잡이에서 미끄러지거나 수납장이 밀리면 바로 정지한다.
- 볼 것: 문이 85° 넘게 열리기 전에 책이 대기선(y −0.14)을 넘는지. 넘으면 7498과 같은 문제다.

### 책 시작 위치 10곳 시험 (오른팔 시작 행, 2026-11-08)

학습은 에피소드마다 NPZ 시작 표 121행(책 중심 x 0.53~0.73, y −0.49~−0.29, 2 cm 간격 11 × 11) 중 하나를 골랐다. 행 번호 = 11·ix + iy, 책 x = 0.53 + 0.02·ix, y = −0.49 + 0.02·iy, z는 모두 0.456이다. 왼팔은 121행 모두 같고, 2998과 2000(curriculum)은 같은 표를 쓴다.

범위를 고르게 덮는 10행이다. 쉬운 순서로 적었다. `--arm`을 꼭 준다. 빼면 기본값 `both`라 다른 팔이 ready로 간다.

```bash
# 왼팔 (모든 행 공통, 문 닫힘)
ros2 run fr3_husky_task_manager move_to_joint --arm left --left-target-positions -0.4463 -0.1901 -0.4999 -1.8995 0.3425 2.4281 0.1574

# 1. 행 60 (0.63, -0.39) 가운데
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.7872 -0.4824 -0.5670 -2.3592 -0.0158 2.6981 0.4468
# 2. 행 82 (0.67, -0.39) 가운데에서 4 cm 뒤
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.7185 -0.3254 -0.4825 -2.2499 0.1877 2.7065 0.3212
# 3. 행 5 (0.53, -0.39) 앞 변 가운데
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.6160 -0.8020 -0.4859 -2.5808 -0.1273 2.5912 0.4724
# 4. 행 115 (0.73, -0.39) 뒤 변 가운데
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.6101 -0.1202 -0.3623 -2.0614 0.3708 2.6825 0.2244
# 5. 행 65 (0.63, -0.29) 왼쪽 변 가운데 (수납장 쪽)
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.6734 -0.5074 -0.2852 -2.4440 -0.0656 2.7338 0.6774
# 6. 행 55 (0.63, -0.49) 오른쪽 변 가운데
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 1.1791 -0.5952 -1.0555 -2.2028 -0.3068 2.6084 0.4778
# 7. 행 0 (0.53, -0.49) 앞 오른쪽 모서리
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.7084 -0.7211 -0.7354 -2.4223 -0.1064 2.5485 0.2696
# 8. 행 10 (0.53, -0.29) 앞 왼쪽 모서리
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.5897 -0.8567 -0.2467 -2.6738 -0.1248 2.6115 0.6934
# 9. 행 110 (0.73, -0.49) 뒤 오른쪽 모서리
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.5715 0.0064 -0.5045 -1.8959 0.5965 2.5493 -0.0597
# 10. 행 120 (0.73, -0.29) 뒤 왼쪽 모서리
ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.7072 -0.1863 -0.2949 -2.1558 0.0981 2.7561 0.5810
```

- 행 55는 오른팔 1번 관절이 1.18 rad로 가장 많이 돈다. 행 110은 2번 관절이 0.006 rad로 팔을 가장 앞으로 뻗는다. 옮기는 동안 주변을 비운다.
- 다른 행이 필요하면 `ROW`만 바꿔 명령을 뽑는다:
  ```bash
  cd ~/jw_ws/src/fr3_husky && ROW=60 python3 -c "
  import numpy as np,json,os
  m=json.loads(str(np.load('fr3_husky_nn_policy/models/cabinet_book_insert_75cm_door_first_model_2998.npz',allow_pickle=True)['manifest_json'].item()))['start']
  r=int(os.environ['ROW']); q=m['joint_position_rows'][r]; b=m['book_position_base_rows'][r]
  print('book centre (base):',[round(v,3) for v in b])
  print('ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions',' '.join('%.4f'%v for v in q[7:]))"
  ```

행마다 순서:
1. 오른손을 열고 책을 치운 뒤 위 명령으로 옮긴다.
2. RViz `Cabinet nearest start row`가 노랑인지 본다(빨강이면 자세가 어느 행과도 0.10 rad 안에 없다).
3. 책을 세워 잡는다. 주황 책(`Cabinet book (policy input)`, FK 값)이 노란 책과 겹쳐야 한다.
4. 문을 닫고 왼손이 손잡이를 잡았는지 본다(`door_angle` ≈ 0).
5. `start_policy`. 로그의 `Nearest start row N`이 고른 행인지 본다.

### 실로봇 결과 (2026-11-07 ~ 11-08)

세션별 값은 `experi_result/dual_fr3_cabinet_book_insert/results.csv`에, trace는 `data/cabinet_book_insert/<세션>/`에 있다.
값은 trace에서 다시 계산했다(책 위치 = obs 29~31 + goal, 문 각도 = FK 노드 값). 끝난 이유는 노드가 남기지 않아 없다.
오차는 마지막 step의 책 중심 − goal (0.795, 0.06, 0.54). 시작 위치는 중심 (0.66, −0.39) 기준 cm.

| 정책 | 세션 | 시작 | 결과 |
|---|---|---|---|
| 2998 | 224557, 225244, 225853 | 중심, x+1 | 0.9 s 만에 멈춤(reflex). 문 93~103° |
| 2998 | 230216, 230723 | x+1 | 12 s 끝. 책 오차 3.9 / 4.0 cm (x −3, y −2.4), 문 끝 80° |
| 2000 | 230901, 231136 | x+1 | 오차 0.6 cm, 문 115° |
| 2000 | 232933, 233134, 233300 | x+1 | 오차 3.1~3.4 cm (z +3), 문 148~152°. TF 기반 FK(15 Hz)일 때 값이라 문 각도를 그대로 믿기 어렵다 |
| 2000 | 000029 ~ 004942 (13회) | 중심, ±1, ±4, y+8, x+6 y−5, x+8 y−6, x+8 y+6 | 13회 모두 마지막 오차 0.3~1.1 cm, 문 118~125° |

- 2000은 시작 위치 13곳(x+8 두 곳은 학습 범위 밖) 모두 책을 1.1 cm 안에 넣었다.
- 2998은 끝까지 간 2회 모두 책이 goal 4 cm 앞(x −3 cm)에서 멈췄고 문이 80°로 닫혀 들어왔다.
- 1~2 cm 시작 차이는 결과에 영향이 없었다. 12 s보다 일찍 끝난 세션(178~218 step)은 손으로 멈춘 것으로 보인다.
- rosbag은 두 개뿐이다. `bag_20261107_231106`(16 s)은 정책 실행 사이에 녹화돼 실행 장면이 없다. `bag_20261107_231127`(18 s)에는 2000 세션 231136의 앞 9 s만 들어 있다.

### 확인한 것 / 못 한 것 (2026-11-07, 학습 PC)

- 확인
  - NPZ에 든 시작 표 121행, 행 60 관절값, 책 목표 (0.795, 0.06, 0.54)가 75 cm 배치 문서와 같다.
  - sim 평가는 위 표와 같다.
- 못 한 것
  - 2998로 FK 노드와 정책 노드(shadow)를 학습 PC에서 띄워 보지 않았다. 45 cm 7498로만 확인했다.
  - 실로봇에서 shadow부터 확인한다.

## 확인한 것 / 못 한 것 (2026-11-06, 학습 PC)

- 확인
  - 실물 URDF(`dual_fr3.urdf.xacro mobile:=true`)로 robot_state_publisher → FK 노드 → 정책 노드(shadow)까지 띄웠다.
  - 시작 행의 관절값을 넣으면 FK 책 pose와 문 각도가 sim과 같았다.
    - closed 행 60: 책 (0.630, −0.390, 0.156), 문 0.000 rad
    - open-door 행 0: 문 1.571 rad
  - 두 모델 모두 `start_policy`를 통과했다.
  - 시작 행에서 0.3 rad 벗어나면 거부되고, 12 s가 되면 멈췄다.
- 고친 것(codex 초안 대비)
  - 제어기가 거부하는 `max_actuator_step_rad` 0.03 → 0.001
  - FK 노드가 보정값을 넣으면 죽던 문제(빈 리스트 파라미터 타입)
  - 문 각도 부호 → −1
  - 시작 행 검사
  - 그리퍼와 move_group launch 인자
- 못 한 것
  - 학습 PC에는 제어기 패키지가 없어 **명령 모드는 확인하지 못했다.**
  - `load_gripper:=true`로 함께 띄우는 것도 처음이다.
  - 실로봇에서는 shadow → 명령 모드 순서로 확인한다.
