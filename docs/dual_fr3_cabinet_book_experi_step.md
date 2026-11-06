# dual FR3 캐비닛 책 넣기 실로봇 실험 순서 (2026-11-06)

배치(책상 높이, 수납장·책 위치, link0 기준 좌표)는 [dual_fr3_cabinet_book_real_setup.md](dual_fr3_cabinet_book_real_setup.md), 노드 계약과 export는 [../fr3_husky_nn_policy/CABINET_BOOK_INSERT_DEPLOYMENT.md](../fr3_husky_nn_policy/CABINET_BOOK_INSERT_DEPLOYMENT.md)에 있다.

## 정책 (`fr3_husky_nn_policy/models/`)

| 파일 | 시작 상태 | sim 결과 |
|---|---|---|
| `cabinet_book_insert_open_door_upright_pretrain_model_2499.npz` | 문 90° 열림, 왼손이 열린 문의 손잡이를 잡음 | 영상 3/3 성공, 약 1.5 s에 넣음, 끝 기울기 2.5°, 깊이 11.8 cm |
| `cabinet_book_insert_closed_door_stage2_model_7498.npz` | 문 닫힘 | 121개 시작 위치 중 120 성공, 첫 성공 중앙 1.2 s, 끝 기울기 중앙 4.1°, 깊이 12.9 cm |
| `cabinet_book_insert_v1_model_14999.npz` | 문 닫힘 | 성공 0 (책을 기울인 채 깊이·레인만 맞춤, 비교용) |

- 2499, 7498은 v2(codex 2단계 학습)다. 2499가 1단계, 7498이 그 이어 학습이다.
- v1(`v1_5070`, 15000 iteration)은 보상이 책 위치만 보고 기울기를 안 봐서, 책을 기울인 채 넣는 국소해에 머물렀다(sim 성공 0, book_depth 1.68/2.0, book_lane 0.85/1.0). v2와 비교하려고 export했다(2026-11-07). 관측·행동 형식은 v2와 같다.
- 실로봇 관찰(2026-11-07): 7498이 문을 끝까지 열기 전에 책을 넣으려 했다. sim에서도 문 약 60°에서 책이 들어가기 시작했다(책 보상 게이트가 60~80°).
- **둘 다 sim에서 관절 약 2 rad/s로 움직인다**(행동이 매 step 0.1 rad까지). 실로봇 제어기 기본(`joint_velocity_scale` 0.10)은 j1~4 0.26 rad/s라 로봇이 정책보다 약 8배 느리게 따라간다. 노드가 시작할 때 경고를 띄운다. 정책이 학습 때 보지 못한 상태가 되므로 sim처럼 움직이지 않을 수 있다.
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
3. 배치를 맞춘다: 책상 윗면은 **팔 장착면 + 0.045 m**, 수납장 밑면 중심 (0.950, 0.100), 경첩 축 (0.7635, 0.2555), 손잡이 막대 축 (0.710, −0.035). 자세한 것은 배치 문서.
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
  - `Joint speed: trained up to 2.00 rad/s ..., controller limit [0.26, ...]`
  - 느리다는 WARN
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

# closed-door 7498 (문 닫힘), 행 60
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.2898 0.4389 -0.6271 -1.8862 1.4038 2.7138 -0.6641 \
  --right-target-positions -1.4168 1.1355 1.3881 -2.4121 -2.0347 2.6719 1.6555

# open-door 2499 (문 90°), 행 60: 왼팔만 다르다
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 1.3161 1.4299 -1.6901 -2.1074 0.6268 1.7453 2.1358 \
  --right-target-positions -1.4168 1.1355 1.3881 -2.4121 -2.0347 2.6719 1.6555
```

1. 수납장을 제자리에 놓는다. 손잡이 막대가 열린 왼손 손가락 사이에 오고, 손끝이 막대 축을 약 2 cm 지나게 한다.
   - closed-door 모델은 문을 닫은 채로 놓는다.
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
ros2 topic echo --once /cabinet_book_insert/door_angle   # 닫힘 ≈ 0, open-door ≈ 1.57
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
- 속도: 기본 `joint_velocity_scale:=0.10`(j1~4 0.26 rad/s)으로 시작한다.
  - 더 빠르게 하려면 `joint_velocity_scale:=0.19`(0.5 rad/s)를 준다.
  - `max_actuator_step_rad`는 0.001이면 모든 관절이 1 rad/s에서 한 번 더 막힌다. 제어기는 0.01까지만 받는다.
- 손잡이 힘을 잴 센서가 없다. 왼손이 손잡이에서 미끄러지거나 수납장이 밀리면 바로 정지한다.

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
