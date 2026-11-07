# dual FR3 큐브 들기 (Isaac Lab 3.0 Sim2Real-100Hz) 실로봇 실험 순서 (2026-11-07)

목적: 3.0.0 큐브 들기 정책을 실로봇에 올려 **Franka 그리퍼 지연**을 잰다. 잰 값은 이후 큐브 pick-and-place 학습(`Isaac-PickPlace-Cube-DualFR3-UMI-v0`)에 넣는다.

![큐브 배치](figures/liftcube_sim2real_layout.png)

그림 다시 그리기 (dual_fr3_lab): `python scripts/tools/draw_liftcube_layout.py --out ~/etri_ws/src/fr3_husky_etri/docs/figures/liftcube_sim2real_layout.png`

## 정책

| | |
|---|---|
| 모델 | `fr3_husky_nn_policy/models/dual_fr3_lift_sim2real100hz_model4999.npz` (설명 `..._model4999.yaml`) |
| 학습 | `Isaac-Lift-Cube-DualFR3-Sim2Real-100Hz-v0`, `logs/rsl_rl/dual_fr3_lift_sim2real_100hz/2026-10-22_18-14-56/model_4999.pt` |
| export | `python -m fr3_husky_nn_policy.export_ppo_actor <ckpt> models/dual_fr3_lift_sim2real100hz_model4999.npz --output-activation tanh` (torch tanh(mean)과 최대 차 2.6e-6) |
| 관측 36 | 오른팔 q − ready 9(관절 7 + 손가락 2), qdot 9, 큐브 중심 3, 목표 3, 목표 quat [0,0,0,1], 이전 행동 8 |
| 행동 8 | 팔: q_target = q + clip(0.1·a, ±0.1 rad), 20 Hz, Isaac PD. 그리퍼: a[7] < 0이면 닫기 |
| 노드 | `ppo_liftcube_sim2real_policy_node` (launch `dual_fr3_lift_sim2real_policy.launch.py`, config `dual_fr3_lift_sim2real100hz.yaml`) |

### 예전 v3 노드(`ppo_liftcube_policy_node`)와 다른 점

- 제어기의 Isaac PD 경로(`isaac_relative_control`)는 `robot_name: dual`만 받고, dual은 그리퍼를 다루지 않는다. 그래서:
  - 팔: dual goal로 14개 목표를 보낸다. **왼팔은 시작할 때 각도에 고정**, 오른팔만 정책이 움직인다.
  - 그리퍼: 노드가 franka action `/right_franka_gripper/grasp`(width 0, speed 0.1 m/s, force 30 N, epsilon 0.08)와 `/right_franka_gripper/move`(width 0.08, speed 0.1 m/s)를 직접 보낸다. 제어기의 GripperGrasp/GripperOpen과 같은 값이다. 열고 닫는 명령이 바뀔 때만 보낸다(히스테리시스 0.1).
- 보낸 시각, accepted, result를 모두 로그에 남긴다(아래 "로그").
- 비전 끊김 처리는 없다. `/object_pose`가 0.2 s 넘게 안 오면 세션을 멈춘다(`object_pose_timeout_s`).
- 관절 상태는 box·캐비닛 노드와 같다. 팔은 `/dual_fr3/joint_states`(ros2_control, 1 kHz)에서 받는다.
  - `/joint_states`는 joint_state_publisher가 팔과 그리퍼를 합쳐 30 Hz로만 내므로 쓰지 않는다.
  - 손가락은 `/right_franka_gripper/joint_states`에서 받는다.

## 관측과 실로봇 출처

관측 36개는 모두 실로봇에서 얻는다. 그리퍼와 큐브 위치는 sim과 실물의 값이 다르다.

| 번호 | 관측 | sim (학습) | 실로봇 출처 |
|---|---|---|---|
| 0~6 | 오른팔 관절 q − ready 자세 | 잡음 0.005 rad | `/dual_fr3/joint_states` (FR3 엔코더) |
| 7~8 | 손가락 q − 0.04 | 잡음 0.005 | `/right_franka_gripper/joint_states` (폭/2) |
| 9~15 | 오른팔 관절 속도 | 잡음 0.05 rad/s | `/dual_fr3/joint_states` velocity |
| 16~17 | 손가락 속도 | 잡음 0.05 | 그리퍼 폭을 노드가 미분. franka_gripper는 측정 속도를 내지 않는 것으로 판단(실물 미확인) |
| 18~20 | 큐브 중심 위치 (base) | 실제 위치 + 잡음 1 cm, 지연·끊김 없음 | ArUco `/object_pose` |
| 21~23 | 목표 위치 | 무작위 명령 | 노드 파라미터 `target_position` |
| 24~27 | 목표 quat | [0, 0, 0, 1] 고정 | 상수 |
| 28~35 | 이전 행동 8 | | 노드가 기억 |

### sim과 달라지는 곳

| | sim | 실물 |
|---|---|---|
| 그리퍼 (관측 7~8, 16~17) | implicit PD(stiffness 2e3)라 약 0.1 s에 닫힘. 명령 지연 없음 | action goal → libfranka. 0.1 m/s로 움직임(손가락 하나 0.05 m/s). 8 cm → 5 cm 큐브까지 0.3 s 이상에 명령 지연이 더해짐 |
| 큐브 위치 (관측 18~20) | 실제 위치 + 잡음 1 cm, 매 step 새 값 | ArUco 처리 지연이 있음. 쥐면 손이 마커를 가릴 수 있음 |
| 큐브 방향 | 관측 안 함, yaw 0만 학습 | 정책이 yaw를 못 보므로 큐브를 base 축에 나란히 놓아야 함 |
| 큐브 질량 | 0.175~0.325 kg | 90 g |
| 힘·접촉 | 관측 안 함 | 잡았는지는 손가락 폭으로만 앎 |

## 이번 실험에서 볼 것 (pick-and-place에 넣을 값)

이 실험의 목적은 아래 값을 재서 pick-and-place 학습(`Isaac-PickPlace-Cube-DualFR3-UMI-v0`)의 그리퍼 모델, 비전 지연과 끊김에 넣는 것이다.

| 볼 것 | 어디서 | 쓰임 |
|---|---|---|
| 그리퍼 명령 지연: send → accepted → 손가락이 움직이기 시작 → result | `liftcube_log_summary`의 gripper 표 (`gripper_events.csv`, `trace.csv`의 `finger_width_m`) | sim 그리퍼에 명령 지연과 닫힘 속도를 넣음 |
| 닫힘 속도: 폭이 줄어드는 기울기, 큐브를 쥔 뒤 폭 | `trace.csv`의 `finger_width_m`, `obs_16`·`obs_17` | sim 손가락 속도 제한(현재 거의 즉시) |
| 닫기 명령에서 큐브가 들리기까지 걸린 시간 | 요약의 `lift start` 줄 | 정책이 손가락이 다 닫히기 전에 드는지 확인. 그렇다면 sim 그리퍼를 늦춰야 함 |
| 열기 명령 지연 (정책이 열 때만) | gripper 표의 `open` 줄 | 놓기(place) 단계 지연 |
| 큐브 pose 지연 | 요약의 `cube age` (`object_stamp_age_ms`는 카메라 stamp부터, `object_pose_age_ms`는 노드가 받은 뒤부터) | 비전 지연 DR 범위 (box 값과 비교) |
| 큐브 pose 끊김 | 쥔 뒤 `cube pose is stale` 종료 여부, `object_pose_age_ms` 최댓값 | 끊김 확률과 길이, 노드의 "마지막 값 유지"가 필요한지 |
| 정지한 큐브의 위치 흔들림 | 요약의 `cube noise` 줄 (처음 1 s, 손이 닿기 전) | 관측 잡음(현재 1 cm)이 맞는지 |
| 손가락 속도 관측이 실제로 나오는지 | `obs_16`, `obs_17`이 닫는 동안 0이 아닌지 | 미분이 제대로 되는지 |
| tick 주기, 관절 상태 나이, 제어기 추종 오차 | 요약의 `tick`, `joint err` | 20 Hz 정책 루프가 학습과 같은지 |

- shadow 실행(4번)에서는 그리퍼 명령을 보내지 않는다. 그리퍼 지연을 빼고 큐브 pose 지연과 흔들림만 본다.
- 그리퍼 지연만 따로 보려면 정책 없이 `ros2 action send_goal /right_franka_gripper/grasp ...`를 보내고 `ros2 topic echo /right_franka_gripper/joint_states`로 폭을 볼 수도 있다.
  - 이때는 정책 노드 로그가 남지 않으니, 시간은 `ros2 bag record /right_franka_gripper/joint_states`로 남긴다.

## 준비

1. **손끝을 기본 Franka 팁으로 바꾼다.** 이 정책은 기본 팁(TCP가 link7에서 0.2104 m)으로 학습했다. UMI 팁은 7.4 cm 더 길다.
2. 큐브: 50 × 50 × 100 mm, 세워서 놓는다.
3. ArUco 노드가 `/object_pose`(`geometry_msgs/PoseStamped`, `frame_id: base`)로 **큐브 중심**을 낸다. 책상 위에 선 큐브라면 z가 약 0.395다.
   - 마커 윗면 pose를 그대로 내면 z가 약 0.445가 되어 시작 검사에서 걸린다.

## 큐브 놓는 곳

| | 값 (base frame) |
|---|---|
| 큐브 중심 x | **0.45 ~ 0.53** (학습 0.37 ~ 0.53. 실제 책상 앞 모서리가 x 0.40이라 앞쪽은 못 씀) |
| 큐브 중심 y | −0.37 ~ −0.13 (가운데 −0.25) |
| yaw | **0**: 큐브 면을 base 축에 나란히. 학습에 yaw 무작위화가 없었다 |
| 추천 첫 위치 | (0.48, −0.25) |
| 목표 (기본) | (0.45, −0.30, 0.50) = 책상 위 15.5 cm. 범위 x 0.35~0.55, y −0.45~−0.15, z 0.45~0.60 |

노드 시작 검사 범위는 x 0.36~0.54, y −0.38~−0.12, z 0.365~0.425다(학습 범위 ±1 cm).

## 1. 터미널 1: 제어기 + 정책 노드 (shadow)

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_lift_sim2real_policy.launch.py shadow_mode:=true launch_move_group:=true
```

- 기본값은 `load_gripper:=true`(그리퍼 action server 필요), `with_realsense:=false`다.
- 목표를 바꾸려면 노드를 띄운 뒤 `ros2 param set /ppo_liftcube_sim2real_policy_node target_position "[0.45, -0.30, 0.55]"`를 쓴다(다음 start부터 적용).
- 로그 확인: `Loaded ...dual_fr3_lift_sim2real100hz_model4999.npz (36->8 tanh ...), mode=SHADOW`.

## 2. 터미널 2: ArUco

큐브 들기 때 쓰던 ArUco 노드를 띄운다. 다음 명령으로 확인한다.

```bash
ros2 topic hz /object_pose
ros2 topic echo /object_pose --once
```

`frame_id`가 `base`이고 z가 0.395 근처면 된다.

## 3. 터미널 3: ready 자세, 그리퍼 열기

```bash
ros2 run fr3_husky_task_manager move_to_joint        # 기본값이 학습 ready 자세 (오른팔 0.5236 -0.7854 0 -2.3562 0 1.5708 0.7854)
ros2 action send_goal /right_franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
```

큐브를 위 표의 위치에 놓는다.

## 4. shadow로 한 번 확인

```bash
ros2 service call /ppo_liftcube_sim2real_policy_node/start_policy std_srvs/srv/Trigger {}
```

- 명령은 보내지 않고, 8 s 동안 추론과 로그만 남긴다.
- 그리퍼는 `would_send` 이벤트로만 남는다.
- 끝나면 `ros2 run fr3_husky_nn_policy liftcube_log_summary`로 큐브 위치, tick, 지연을 본다.

시작이 거부되면 메시지대로 고친다.

| 메시지 | 고칠 것 |
|---|---|
| `cube ... outside the training start region` | 큐브 위치. z가 0.44 근처면 ArUco가 큐브 중심이 아닌 마커 pose를 내는 것 |
| `right arm is not at the training ready pose` | `move_to_joint` |
| `open the right gripper first` | 3번의 Move |
| `cube pose is stale` | ArUco 노드, 마커 가림 |
| `franka gripper grasp/move action servers are not ready` | `load_gripper:=true`인지, 그리퍼 노드가 떴는지 |

## 5. 실제 실행

터미널 1을 끄고 다음 명령으로 다시 띄운다.

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_lift_sim2real_policy.launch.py shadow_mode:=false launch_move_group:=true
```

3번 순서(ready 자세, 그리퍼 열기, 큐브 놓기)를 반복한 뒤:

```bash
ros2 service call /ppo_liftcube_sim2real_policy_node/start_policy std_srvs/srv/Trigger {}
ros2 service call /ppo_liftcube_sim2real_policy_node/stop_policy std_srvs/srv/Trigger {}    # 멈출 때
```

- 정책은 8 s 뒤 제어기가 끝낸다.
- 끝나도 그리퍼는 그대로 쥐고 있다. 큐브를 받친 뒤 3번의 Move로 연다.
- 다음 시도 전에 `move_to_joint`로 ready 자세로 돌아간다.
- 정책은 관절을 sim처럼 약 2 rad/s까지 움직인다(행동이 한 step에 0.1 rad까지). 처음에는 비상정지를 손에 쥔다.

## 로그

세션마다 `fr3_husky_nn_policy/data/dual_fr3_liftcube_sim2real100hz/<시각>/`에 남는다(`log_root`로 바꿀 수 있다).

| 파일 | 내용 |
|---|---|
| `metadata.json` | 모델, 파라미터, 큐브 시작 위치, 목표, 끝난 이유 |
| `trace.csv` | 20 Hz 한 줄씩: tick 주기, 관절·큐브 pose 나이, 추론 시간, 관측 36, 행동 8, 오른팔 목표 7, 그리퍼 명령(`gripper_closed_cmd`)과 손가락 폭(`finger_width_m`), 큐브 quat |
| `gripper_events.csv` | 그리퍼 goal마다 `send`, `accepted`, `result`(성공 여부, 그때 손가락 폭) |

요약:

```bash
ros2 run fr3_husky_nn_policy liftcube_log_summary            # 가장 최근 세션
ros2 run fr3_husky_nn_policy liftcube_log_summary <run_dir>
```

명령마다 다음 지연을 ms로 출력한다:
- send → accepted
- send → 손가락이 처음 2 mm 움직인 때(20 Hz trace 기준, ±50 ms)
- send → result

sim 그리퍼(implicit PD, stiffness 2e3)는 약 0.1 s에 닫힌다. 이 차이가 pick-and-place 학습에 넣을 지연이다.

## 주의

- 실물 큐브(90 g)는 학습 질량(0.175~0.325 kg)보다 가볍다. 들기는 더 쉬울 수 있지만 쥘 때 밀려날 수 있다.
- 학습 마찰은 0.4~1.2(정지), 큐브 크기는 ±5 mm 무작위였다.
- 큐브 pose 관측 잡음은 학습 때 1 cm였고, 지연과 끊김은 없었다. 쥔 뒤 마커가 손에 가려 0.2 s 넘게 끊기면 세션이 멈춘다. 이게 자주 나오면 `object_pose_timeout_s`를 늘리거나 마커를 옮긴다.
- 왼팔은 시작 각도에 고정된다. 왼팔 그리퍼는 건드리지 않는다.
