# dual FR3 box lift 토크 모드 비교 실험 (hover 시작, 정책 7개)

2026-11-23 작성. 토크 모드 box lift 정책 7개를 실물에서 **hover 시작**으로 같은 조건에서 10회씩 돌려 성공률을 비교한다. KRoC 지연 sim2real 원고의 실물 결과다.

- 시작은 pregrasp 표가 아니라 **hover 자세**다. 팔은 박스 위치와 상관없이 고정 자세(판 중심이 박스 윗면보다 약 15 cm 위)에서 시작하고, 정책이 내려와서 잡는다. 박스만 옮기면 되므로 MoveIt 자세를 회차마다 바꾸지 않는다.
- 위치 모드는 이번 실험에서 하지 않는다(토크 모드 `command_interface:=effort`).
- 7개 정책 모두 hover 관절 자세가 같다. 정책을 바꿔도 2번(hover 이동) 명령은 그대로다.

# 빠른 실행 (v2 → realdrop)

터미널마다 하는 일:

| 터미널 | 하는 일 | 언제 다시 띄우나 |
|---|---|---|
| T1 | 컨트롤러 + MoveIt + 정책 노드(inference), launch 하나 | 정책을 바꿀 때, reflex로 로봇이 멈췄을 때 |
| T2 | 충돌 임계값, hover 이동, 시작 검사 | — |
| 비전 PC | `box_tf_publisher` | — |
| T3 | 정책 시작·정지 | — |
| T4 | bag 기록 | 회차마다 |

**모든 터미널 공통 (처음에 한 번씩)**

```bash
source ~/jw_ws/install/setup.bash
cd ~/jw_ws/src/fr3_husky
```

- pull한 뒤에는 먼저 빌드한다: [0. 준비](#0-준비-한-번).
- 시작과 정지는 **따로** 보낸다. 같이 붙여 넣으면 시작하자마자 정지된다.

## 1. T1: 컨트롤러 + 정책 노드 켜기

정책 launch 하나가 컨트롤러, MoveIt, 정책 노드를 같이 띄운다.

**v2**

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/dual_fr3_boxlift_pregrasp_wide_rollalign_curriculum_v2_model5999.npz \
  box_size_m:="[0.310, 0.410, 0.280]" start_pose:=hover goal_z_m:=0.20 \
  launch_move_group:=true \
  box_pose_topic:=/box_tf box_position_offset_m:="[0.0, 0.0, -0.03]" \
  log_task_name:=dual_fr3_boxlift_torque_eval \
  shadow_mode:=false
```

**realdrop** (v2 회차를 다 끝낸 뒤 T1을 Ctrl+C로 끄고 이것을 띄운다)

```bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/dual_fr3_boxlift_pregrasp_wide_rollalign_curriculum_realdrop_model5999.npz \
  box_size_m:="[0.310, 0.410, 0.280]" start_pose:=hover goal_z_m:=0.20 \
  launch_move_group:=true \
  box_pose_topic:=/box_tf box_position_offset_m:="[0.0, 0.0, -0.03]" \
  log_task_name:=dual_fr3_boxlift_torque_eval \
  shadow_mode:=false
```

- 다른 정책은 `model_path`의 `_v2_` 자리만 바꾼다: `nodelay`, `novisiondelay`, `nojointdelay`, `nocmddelay_s43`, `realdrop_sysid` ([정책 7개](#정책-7개)).
- 로그에 run 이름(`curriculum_..._v2` 또는 `..._realdrop`)과 `start pregrasp_table (3956 table rows) + hover, using start_pose hover`가 나오는지 본다.
- **그날 처음 띄우는 정책(v2)은 마지막 줄을 `shadow_mode:=true`로 바꿔 한 번** 1회차를 해 본다. 시작이 받아지고(`hover start`) 로그의 관절 목표 변화가 step당 0.1 rad 이하이며 판이 박스 쪽으로 내려가는 목표면, T1을 끄고 `shadow_mode:=false`로 다시 띄운다.
- **정책을 바꿀 때:** 박스를 치운다 → T3에서 정책이 멈춰 있는지 확인(`stop`) → T1 Ctrl+C → 새 정책으로 T1을 띄운다 → **2번 충돌 임계값을 다시 보낸다** → `ros2 topic hz /box_tf`로 비전이 다시 들어오는지 본다.

## 2. T2: 충돌 임계값 (T1을 띄울 때마다)

T1이 다 뜬 뒤 보낸다. 정책을 바꾸느라 T1을 다시 띄웠을 때도 다시 보낸다.

```bash
ros2 service call /service_server/set_force_torque_collision_behavior franka_msgs/srv/SetForceTorqueCollisionBehavior \
  "{lower_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], upper_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], \
    lower_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0], upper_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]}"
```

- 응답이 `success=True`인지 보고, 시각을 기록지에 적는다.
- **모든 정책에서 같은 값으로 둔다.** 이유와 기본값으로 할 때의 처리는 [충돌 임계값](#충돌-임계값)을 본다.

## 3. 비전 PC: 박스 pose

비전 PC에서 `box_tf_publisher`를 띄운다([dual_fr3_box_experi_step.md](dual_fr3_box_experi_step.md) B). T1이 떠 있어야 카메라 TF를 받는다. 로봇 PC에서 `ros2 topic hz /box_tf`로 들어오는지 본다.

## 4. 한 회차

### 4-1. 박스 치우고 팔을 hover 자세로 (T2)

박스를 팔이 움직이는 길 밖으로 먼저 치운다.

```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

- 정책이 끝나면 팔이 들어 올린 자세에 있으므로 매 회차 보낸다.
- 대신 `ros2 run fr3_husky_nn_policy boxlift_start_helper --move`를 쓰면 확인을 받고 같은 자세로 보낸다.

### 4-2. 박스 놓고 시작 검사 (T2)

박스를 회차 위치([시험 조건](#시험-조건-정책마다-같은-10회))에 놓는다. 긴 변이 좌우, 판 아래로 앞에서 낮게 밀어 넣는다.

```bash
ros2 run fr3_husky_nn_policy boxlift_start_helper --seconds 8
```

- `--seconds 8`: 기본 3 s로 하면 비전 PC의 `/box_tf`를 찾기(DDS discovery) 전에 끝나 `no box pose on /box_tf in 3 s`가 나올 수 있다(11-23, `/box_tf`는 30 Hz로 나오고 있었는데도 그랬다).

- 다음 세 가지를 확인한다.
  - `hover ... inside` (박스가 hover 학습 범위 안)
  - `start PASS now (hover)`
  - `policy input x, y`가 회차 위치 ±1 cm
- 아니면 박스를 옮기고 다시 실행한다.

### 4-3. bag 켜기 (T4)

이름의 `v2_01`을 정책·회차에 맞게 바꾼다(예: `realdrop_07`).

```bash
mkdir -p ~/jw_ws/bags && ros2 bag record -o ~/jw_ws/bags/boxeval_v2_01_$(date +%m%d_%H%M%S) \
  /box_tf /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state \
  /left_franka_robot_state_broadcaster/external_wrench_in_base_frame \
  /right_franka_robot_state_broadcaster/external_wrench_in_base_frame
```

### 4-4. 정책 시작 (T3)

```bash
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
```

- 응답에 `hover start, goal [...]`가 나오면 시작한 것이다.
- T1에 `Session result appended`가 나올 때까지(시작 후 6 s) 손대지 않는다.

### 4-5. 정책 정지 (T3, 따로 보낸다)

정지하면 조임이 풀려 박스가 떨어진다. **박스 아래를 손으로 받친 뒤** 보낸다.

```bash
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

### 4-6. 마무리

- T4 Ctrl+C (bag 끝).
- 기록지에 눈으로 본 것을 적는다(미끄럼, 기울기, 박스 회전, reflex).
- 다음 회차는 4-1부터.

### 5회차 뒤: 목표 z 바꾸기 (T3)

6~10회차는 목표 z 0.30이다. 정책이 멈춰 있을 때(IDLE)만 받는다.

```bash
ros2 param set /ppo_boxlift_policy_node goal_z_m 0.30
```

- T1을 다시 띄우면(정책을 바꾸면) launch 값 0.20으로 돌아간다. 다음 정책의 1회차는 0.20이 맞다.

### 시작이 거부될 때

응답 message에 이유가 나온다.

| message | 할 일 |
|---|---|
| `arms are ... rad from the hover pose` | 4-1 hover 이동을 다시 |
| `box x=... outside the trained hover start range` | 박스를 옮긴다 |
| `not resting on the floor` | 비전 z를 본다(오프셋 −3 cm가 들어갔는지) |

- 6 s(학습 에피소드) 이후에도 정책은 계속 돈다. 결과는 6 s 시점 박스 pose로 기록된다.
- `fault: joint state is stale`이 나오면 로봇이 멈춘 것이다(reflex). 회차는 실패로 두고 Desk에서 에러를 푼다. 같은 회차를 한 번 더 할지 기록지에 적는다(집계에서는 둘 다 남긴다). T1을 다시 띄웠으면 2번 충돌 임계값도 다시 보낸다.

---

# 상세

## 정책 7개

모두 같은 task 계보(`...-Pregrasp-Wide-RollAlign-Curriculum`), 같은 관측 56D, 같은 hover 자세·박스 범위·목표 범위다. 차이는 학습 때 넣은 지연·끊김·관절 동역학뿐이다. 파일은 `fr3_husky_nn_policy/models/dual_fr3_boxlift_pregrasp_wide_rollalign_curriculum_<P>_model5999.npz`이다.

| `<P>` | 학습에서 다른 점 | sim 성공 (10 cm, 지연 전부 있음) |
|---|---|---|
| `v2` | 기준: 비전 지연 0~150 ms + 끊김, 관절 상태 0~10 ms, 명령 0~10 ms 랜덤화 | 98.8 % (hover만 99.2 %) |
| `nodelay` | 지연 랜덤화 전부 끔 | 89.1 %, 놓침 48 |
| `novisiondelay` | 비전 지연·끊김만 끔 | 89.5 %, 놓침 29 |
| `nojointdelay` | 관절 상태·명령 지연만 끔 | 88.3 %, 놓침 14 |
| `nocmddelay_s43` | 명령 지연만 끔 (seed 43) | 95.7 %, 놓침 2 |
| `realdrop` | v2 + 비전 끊김을 실물 노드에 맞춤(0.05~0.30 s, 끝까지 끊김 없음) | RealDrop 환경 100 % |
| `realdrop_sysid` | realdrop + 팔 armature·마찰을 11-17 실물 토크 probe 식별값으로 | SysID 환경 100 %, 놓침 0 |

- sim 숫자는 dual_fr3_lab `docs/box_delay_ablation_results.md`(512 env, 표·hover 시작 섞음), `docs/dual_fr3_joint_dynamics.md` §7이다. 실물 숫자와 섞지 않는다.

**정책 순서 (7개 전부 할 때):** `v2` → `nodelay` → `novisiondelay` → `nojointdelay` → `nocmddelay_s43` → `realdrop` → `realdrop_sysid` → `v2` 다시 3회(1, 3, 8회차 조건)

- 핵심 대비(v2 vs 지연 끈 정책)를 먼저 한다. 시간이 모자라면 뒤의 둘은 다른 날 해도 된다.
- 마지막 v2 3회는 그날 로봇·비전 상태가 처음과 같은지 보는 확인용이다.
- v2와 realdrop 둘만 할 때도 끝에 v2 3회 확인을 붙이면 두 정책 사이에 로봇 상태가 바뀌었는지 볼 수 있다.

## 시험 조건 (정책마다 같은 10회)

박스는 긴 변(0.41 m)이 좌우, yaw 0. 목표 x·y는 박스 시작 위치(수직 들기, `goal_xy_reference:=box_start`, offset 0), 목표 z는 박스 중심 base 높이다.

| 회차 | 박스 중심 (x, y) [m, base] | 목표 z |
|---|---|---|
| 1 | P1 (0.45, 0.00) | 0.20 |
| 2 | P2 (0.42, −0.05) | 0.20 |
| 3 | P3 (0.48, +0.05) | 0.20 |
| 4 | P4 (0.42, +0.05) | 0.20 |
| 5 | P5 (0.48, −0.05) | 0.20 |
| 6~10 | P1~P5 같은 순서 | 0.30 |

- hover 학습 범위는 x 0.40~0.50, y −0.10~0.10, yaw ±0.1 rad(±5.7°)다. P1~P5는 비전 흔들림(±1.7 cm)을 생각해 안쪽에 잡았다.
- 박스는 helper가 보여 주는 `policy input x, y`가 목표 위치 ±1 cm 안에 오도록 놓는다.

## 0. 준비 (한 번)

코드와 모델을 받고 빌드한다.

```bash
source ~/jw_ws/install/setup.bash
cd ~/jw_ws/src/fr3_husky && git pull
ls fr3_husky_nn_policy/models/dual_fr3_boxlift_pregrasp_wide_rollalign_curriculum_*_model5999.npz   # 7개
cd ~/jw_ws && colcon build --symlink-install --packages-select fr3_husky_nn_policy && source install/setup.bash
cd ~/jw_ws/src/fr3_husky
```

## 충돌 임계값

- 캐비닛 실험(11-07 이후)과 같은 값이다. 5499 실물 10회 중 2회는 기본 임계값에서 판이 바닥에 닿아 `cartesian_reflex`로 멈췄다(`dual_fr3_boxlift_5499_report.md` §5.2). 이번에는 정책 차이만 보려고 올린다.
- 관절 토크 감지는 사실상 꺼지고 100 N 힘 기준만 남는다. 비상정지를 손에 둔다.
- 기본값으로 하기로 하면 빠른 실행 2번을 빼고, reflex 정지는 실패와 따로 센다. 어느 쪽이든 모든 정책을 같게 한다. 정한 값과 시각을 기록지에 적는다.
- 서비스는 T1의 franka 하드웨어가 띄운다. T1을 다시 띄우면(정책을 바꿀 때 포함) 새 연결이라 값이 남는지 확인하지 않았으므로 다시 보낸다.

## 결과

- 세션 요약: `experi_result/dual_fr3_boxlift_torque_eval/results.csv`(회차마다 한 줄, `model`, `start_pose` 열)
- 세션 로그: `data/dual_fr3_boxlift_torque_eval/<세션>/`(`trace.csv` 20 Hz, `metadata.json`)
- bag: `~/jw_ws/bags/boxeval_<P>_<회차>_<시각>/`

**판정**(5499 보고서와 같은 기준): 6 s 시점 박스 중심-목표 3D 거리 < 10 cm이고 기울기 < 15°면 성공. 5 cm 기준도 같이 본다. 6 s 전에 끝난 회차(fault, 작업자 정지)는 실패다. 판 접촉은 실물에서 잴 수 없어 뺀다.

**집계:** `scripts/tools/summarize_boxlift_real.py`(dual_fr3_lab과 같은 파일, 표준 라이브러리만 씀). 저장소 루트에서 실행한다.

```bash
python3 scripts/tools/summarize_boxlift_real.py experi_result/dual_fr3_boxlift_torque_eval/results*.csv \
  --data_root data/dual_fr3_boxlift_torque_eval --trials --md experi_result/dual_fr3_boxlift_torque_eval/summary.md
```

- 정책별 성공률(10 cm, 5 cm), 3D 오차·기울기 중앙값, 들었다 놓친 횟수(박스가 5 cm 넘게 올랐다가 6 s에 5 cm 아래), fault 수를 낸다.
- 5499 결과(`experi_result/dual_fr3_boxlift_real/etri_box_lift_80%_results.csv`, `--start_pose any --since 20261107_1937`)로 돌리면 보고서와 같은 8/10, fault 2가 나온다.
- `results.csv`가 옛 노드의 열 구성으로 이미 있으면, 새 행은 옆의 `results_<해시>.csv`로 간다(열이 어긋나지 않게). 위 명령의 `results*.csv`가 둘 다 읽는다.

## MuJoCo로 먼저 보기 (선택)

같은 노드·hover 시작을 MuJoCo에서 본다. 박스 위치는 `mujoco_hover_box:="[x, y, yaw_deg]"`다. MuJoCo는 컨트롤러와 노드를 한 launch로 띄운다(터미널 하나).

```bash
# 터미널 1
export ROS_DOMAIN_ID=87
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/dual_fr3_boxlift_pregrasp_wide_rollalign_curriculum_v2_model5999.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  start_pose:=hover mujoco_hover_box:="[0.45, 0.0, 0.0]" goal_z_m:=0.20 \
  use_mujoco:=true shadow_mode:=false launch_rviz:=false mujoco_box_friction:=1.2

# 터미널 2 (export ROS_DOMAIN_ID=87 먼저): 시작, 몇 초 본 뒤 정지는 따로
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- realdrop은 `_v2_`를 `_realdrop_`로 바꾼다. 다시 하려면 터미널 1을 끄고 다시 띄운다(reset 없음).
- MuJoCo는 판 접촉 모델이 Isaac과 달라 성공률을 그대로 비교하지 않는다. 노드·hover 자세·시작 검사가 도는지만 본다.

## 노드 변경 (2026-11-23)

- `boxlift_core`: manifest `start.hover`(dual_fr3_lab `export_boxlift_actor.py`가 -Curriculum run에 쓰는 고정 hover 자세, 박스 x·y·yaw 범위, 관절 노이즈)를 읽는다. `check_start_conditions(..., start_pose="hover")`는 팔이 hover 자세에서 관절마다 노이즈 0.01 + `hover_joint_margin_rad`(0.02) 안, 박스가 hover 범위 ± `start_position_margin_m`·`start_yaw_margin_rad` 안, 바닥에 놓였는지를 본다. 기본값 `start_pose="manifest"`는 예전과 같다.
- `ppo_boxlift_policy_node`: 파라미터 `start_pose`(manifest | hover), `hover_joint_margin_rad`. IDLE일 때 `ros2 param set`으로 바꿀 수 있다. hover가 없는 체크포인트에 hover를 주면 시작 때 거부한다. 결과 행·metadata에 `start_pose`를 남기고, RViz `~/start_range_markers`에 hover 박스 범위(주황)를 더 그린다.
- `boxlift_start_helper`: 노드가 hover면 박스가 hover 범위 안인지와 hover `move_to_joint` 명령을 보여 주고, 노드와 같은 hover 검사를 한다.
- launch: `start_pose`, `log_task_name`(비우면 예전처럼 `dual_fr3_boxlift`/`dual_fr3_boxlift_real`), MuJoCo `mujoco_hover_box`.
- `append_session_result`: 기존 CSV의 열 구성이 다르면 `results_<해시>.csv`에 새로 쓴다.
- 확인(5070, 로봇 없이): `test_boxlift_core.py`·`test_boxlift_logger.py` 52개 통과. shadow 노드 + 가짜 관절·박스 토픽으로 hover 시작 수락, 6 s 결과 행(`start_pose` hover), `start_pose manifest`로 바꾸면 거부, 잘못된 값 거부, helper `start PASS now (hover)`. 7개 npz 모두 hover 검사 통과. launch의 MuJoCo hover 인자 생성 확인. 컨트롤러·MoveIt과 같이 띄운 것은 로봇 PC에서 처음이다.
