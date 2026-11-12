# dual FR3 box lift 실로봇 실험 순서 (Pregrasp-Wide, 긴 변 좌우)

# 빠른 실행

정책은 **GoalXY 5499**(2026-11-07 실물 10회 중 8회, [실물 결과](#실물-결과-goalxy-5499-2026-11-07)), 시작 자세는 후보 A(표 줄 1965)다. 정책 설명은 [5499 보고서](dual_fr3_boxlift_5499_report.md), 다른 정책은 [실로봇 후보](#실로봇-후보-2026-11-05), 절차 설명은 아래 [상세](#상세)에 있다.
목표 x·y를 옮기려면 [목표 x/y를 옮기는 체크포인트를 쓸 때](#목표-xy를-옮기는-체크포인트를-쓸-때)를 본다.

- 정책을 바꿀 때는 `model_path`만 바꾼다. 모든 후보가 같은 시작 자세 표를 쓰므로 MoveIt 자세, `box_size_m`, `box_position_offset_m`은 그대로다.

**모든 터미널 공통 (처음에 한 번씩)**

```bash
source ~/jw_ws/install/setup.bash
cd ~/jw_ws/src/fr3_husky
```

- 처음 한 번, 그리고 pull한 뒤에는 빌드한다: [0. 준비](#0-준비-한-번) 2.
- 시작과 정지는 **따로** 보낸다. 같이 붙여넣으면 시작하자마자 정지된다.

## A. MuJoCo (sim-to-sim)

`ROS_DOMAIN_ID=87`로 실로봇과 분리한다. **모든 MuJoCo 터미널에서** export한다.

| 터미널 | 명령 |
|---|---|
| 1. MuJoCo + 컨트롤러 + 정책 노드 | 아래 명령. 뷰어가 뜨고 `Object pose publisher started`가 보일 때까지 기다린다 |
| 2. 시작 | `ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}` |
| 2. 정지 (4~5 s 본 뒤) | `ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}` |
| 다시 하기 | 터미널 1을 Ctrl+C로 끄고 다시 띄운다 (reset 없음) |

```bash
# 터미널 1
export ROS_DOMAIN_ID=87
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_5499.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  use_mujoco:=true shadow_mode:=false launch_rviz:=false \
  mujoco_start_row:=1965 goal_z_m:=0.15 \
  mujoco_box_friction:=1.2
```

- 다른 시작 자세: `mujoco_start_row`를 2790, 2166, 371, 1257 중 하나로 바꾼다.
- 비전처럼 노이즈·지연이 섞인 박스 pose로 보려면 `mujoco_vision_sim:=true`를 붙인다. 실로봇처럼 박스를 3 cm 높게 보게 하려면 `mujoco_vision_bias_m:=[0.0,0.0,0.03]`도 붙인다.
- 결과 로그: `data/dual_fr3_boxlift/<시각>/trace.csv` ([결과 보기](#3-결과-보기)).

## B. 실로봇 (지금은 shadow까지)

`ROS_DOMAIN_ID`는 `.bashrc`의 23을 그대로 쓴다. 명령 모드는 비전 `/box_pose`가 붙은 뒤에 한다.

**박스를 아무 데나 놓고 시작 자세 고르기 (자동):** 로봇 launch와 비전이 떠 있으면

```bash
ros2 run fr3_husky_nn_policy boxlift_start_helper          # 박스 pose → 가장 맞는 표 줄과 move_to_joint 명령, 지금 시작 검사 결과
ros2 run fr3_husky_nn_policy boxlift_start_helper --move   # 같은 것 + 확인 후 팔 이동 (박스를 먼저 뺀다)
```

- 떠 있는 정책 노드에서 모델, 박스 토픽, z 보정, 허용 범위를 읽어 노드와 같은 기준(`check_start_conditions`)으로 판정한다.
- 순서: 박스 놓기 → helper로 줄 확인 → 박스 빼기 → `--move`(또는 출력된 명령) → 박스 다시 놓기 → helper로 `start PASS` 확인 → `start_policy`.

| 순서 | 터미널 | 할 일 |
|---|---|---|
| 1 | 1 | 컨트롤러 + MoveIt + 정책 노드 (shadow): 아래 명령 |
| 2 | 2 | 팔을 ready → 시작 자세로 (**박스 없이**): 아래 명령 |
| 3 | — | 박스를 후보 A 위치에 놓는다: 로봇 쪽 면이 O'에서 0.293 m 앞, 중심선 위, 긴 변이 좌우 ([3. 박스 놓기](#3-박스-놓기)) |
| 4 | 3 | 박스 pose publish (비전 전까지 고정값): 아래 명령. 끄지 않고 계속 켜 둔다 |
| 5 | 4 | 시작 → 응답에 `pre-grasp table row 1965`가 나오면 통과. 몇 초 본 뒤 정지 |
| 6 | — | 로그 확인: 관절 목표 변화 ≤ 0.1 rad, 판이 박스 쪽으로 가는 목표, fault 없음 |

```bash
# 터미널 1 (shadow: 로봇에 명령을 보내지 않는다). 박스 pose는 비전 /box_tf, 높이 -3 cm 보정
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$PWD/fr3_husky_nn_policy/models/facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_5499.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  shadow_mode:=true launch_move_group:=true goal_z_m:=0.15 \
  box_pose_topic:=/box_tf box_position_offset_m:="[0.0, 0.0, -0.03]"

# 터미널 2: ready 자세, 그다음 후보 A 시작 자세
ros2 run fr3_husky_task_manager move_to_joint
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0012 1.6314 -0.1775 -1.3488 -1.0002 1.6352 0.5559 --right-target-positions -0.0102 1.6209 0.1723 -1.3833 1.0213 1.6466 0.9907

# 터미널 3: 비전 대신 고정값을 쓸 때만 (터미널 1에서 box_pose_topic, box_position_offset_m를 빼고 띄운다)
# 박스 중심 pose (z = -(바닥~팔 장착면 높이) + 0.140. 0.392 m면 -0.252)
ros2 topic pub -r 30 /box_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: 0.448, y: -0.001, z: -0.252}, orientation: {x: 0.0, y: 0.0, z: -0.0065, w: 1.0}}}"

# 터미널 4: 시작
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
```

```bash
# 터미널 4: 몇 초 본 뒤 정지 (따로 보낸다)
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- **비전 `/box_tf`** (2026-11-04): 비전 PC에서 `box_tf_publisher`를 띄운다. 로봇 launch(터미널 1)가 떠 있어야 비전 노드가 카메라 TF(`base → top_azure_rgb_link`, `config/TF_base2cam.yaml`)를 받는다.
  - 박스를 바닥에 놓아도 비전 z가 약 3 cm 높게 나온다(박스 아랫면 들뜸 + 캘리브레이션). `box_position_offset_m:="[0.0, 0.0, -0.03]"`로 정책 입력에서만 뺀다. 비전 쪽 `/box_tf`는 보정하지 않는다(이중 보정 방지). 회전은 보정하지 않는다(들 때 박스 기울기를 정책이 봐야 한다).
  - 시작 검사 허용은 비전 흔들림 때문에 넓혀 두었다: 위치 ±2 cm(`start_position_margin_m` 0.01), 높이 ±5 cm(`rest_z_tolerance_m` 0.05). `config/dual_fr3_boxlift.yaml`.
- 고정값을 쓸 때 박스 z는 실측한 base 높이로 맞춘다. 학습값 −0.265에서 ±0.03을 벗어나면 시작이 거부된다. 박스 x·y는 ±1 cm, yaw는 ±3° 안이어야 한다.
- 시작이 거부되면 응답 message에 가장 가까운 표 줄과 벗어난 양이 나온다 ([5. 정책 시작과 정지](#5-터미널-4-정책-시작과-정지)).
- 명령 모드(`shadow_mode:=false`)는 [6. 명령 모드](#6-명령-모드)를 보고, 비전이 붙은 뒤에 한다.

---

# 상세

2026-11-03 작성, 11-04 갱신. 학습한 시작 자세로 팔을 먼저 보내고, 박스를 놓은 뒤 RL 정책을 켜서 조이기와 들기를 본다.

- 팔 이동(MoveIt, `move_to_joint`)과 RL 노드는 따로 실행한다. 둘은 서로 모른다.
- 박스 pose는 Azure Kinect 비전으로 받는다. shadow 확인은 손으로 놓은 위치를 고정 publish해도 된다(4번).
- 좌표계와 박스 놓는 위치: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)
- 실로봇 전에 같은 노드로 MuJoCo에서 돌려 보는 절차는 맨 아래 [Isaac → MuJoCo sim-to-sim](#isaac--mujoco-sim-to-sim)에 있다.

## 실물 결과: GoalXY 5499 (2026-11-07)

- 모델 `facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_5499.npz`, 명령 모드, 시작은 pregrasp 표(`start_mode` pregrasp_table), 에피소드 4 s.
- 데이터: `experi_result/video/etri_box_lift_80%_results.csv`(13세션, 첫 세션은 두 줄), 세션 로그 `data/dual_fr3_boxlift_real/<세션>/`, 영상 `experi_result/video/etri_box_lift_80%.webm`.
- 노드에는 자동 성공 판정이 없다. 로거가 남긴 끝 상태(비전 박스 pose)에 **학습 성공 기준**을 적용했다:
  박스 중심-목표 3D 거리 < 10 cm, 기울기 < 15°. 학습은 여기에 양 판 접촉과 0.5 s 유지가 더 붙지만, 실물에서는 판 접촉을 잴 수 없어 뺐다.
- 집계는 마지막 10회(19:37~19:47)다. 앞의 3회(19:24~19:32)는 목표 z 0.15에서 맞춘 회차다(5.2 cm 성공, 12.3 cm 작업자 정지, 5.6 cm 성공).

**10회 중 8회 성공 (80 %)**

| # | 세션 | 목표 (x, y, z) | 3D 오차 (cm) | 기울기 (°) | 결과 | 메모 |
|---|---|---|---|---|---|---|
| 1 | 193733 | (0.45, 0, 0.2) | 4.6 | 7.8 | 성공 | |
| 2 | 193839 | (0.48, 0, 0.2) | 6.3 | 7.8 | 성공 | |
| 3 | 193950 | (0.42, 0, 0.2) | 2.7 | 7.8 | 성공 | |
| 4 | 194058 | (0.45, +0.03, 0.2) | 3.9 | 7.1 | 성공 | |
| 5 | 194207 | (0.45, −0.03, 0.2) | 5.3 | 7.7 | 성공 | |
| 6 | 194309 | (0.45, 0, 0.3) | 3.8 | 7.0 | 성공 | |
| 7 | 194405 | (0.48, 0, 0.3) | 54.5 | – | **실패** | 0.55 s에 `fault: joint state is stale`, 박스 안 들림 |
| 8 | 194516 | (0.42, 0, 0.3) | 54.3 | – | **실패** | 0.55 s에 `fault: joint state is stale`, 박스 안 들림 |
| 9 | 194626 | (0.45, +0.03, 0.3) | 3.0 | 6.7 | 성공 | |
| 10 | 194728 | (0.45, −0.03, 0.3) | 3.9 | 7.8 | 성공 | |

- 성공 8회: 3D 오차 평균 4.2 cm(2.7~6.3), 기울기 6.7~7.8°, 4 s를 끝까지 들고 있었다. 박스 중심은 시작 z −0.24에서 목표 높이 근처(z 0.21~0.28)까지 올라갔다.
- 실패 2회: 0.4 s쯤부터 `/dual_fr3/joint_states`가 들어오지 않아(나이 12 → 62 → 112 ms) 노드가 0.55 s에 멈췄다.
  그때까지 정책 명령(스텝당 최대 0.09~0.10 rad, 관절 추종 오차 0.06~0.10 rad)은 성공한 6번과 거의 같았다.
  들다가 놓친 것이 아니라 로봇 쪽이 멈춘 것으로 보인다(Franka reflex나 통신일 수 있으나 로그로는 원인을 모름). 둘 다 목표 z 0.3, x 0.48/0.42 회차다.
- 큐브 pick-place와 판정 방식이 다르다: 큐브는 노드가 학습 기준(xy 2.5 cm)으로 자동 판정, 박스는 로그 끝 상태를 사후 판정(10 cm, 15°)한다.

## 실로봇 후보 (2026-11-05)

- 파일은 모두 `fr3_husky_nn_policy/models/`에 있다. 시작 자세 후보(A~E)는 `model/dual_fr3_boxlift_pregrasp_wide_rollalign_<…>_start_poses.yaml`이고, 모든 정책이 같은 시작 자세 표(3956줄)를 쓰므로 내용이 같다.
- **바꾸는 것은 `model_path` 하나다.** MoveIt 시작 자세, `box_size_m`, `box_position_offset_m`, `goal_z_m`은 그대로 둔다.
- 11-05에 세운 순서(6697 → LowMuFlat 7200 → 7100 → GoalXY 4700)는 쓰지 않았다. 6697은 실물에서 판이 미끄러지며 박스가 돌았고(결과 판정 기록 없음), 11-07에는 GoalXY 5499로 시험했다.
- 비교하려면 정책마다 같은 박스 위치에서 5회 이상 한다.
- 실험이 끝나면 `data/dual_fr3_boxlift/<시각>/`을 `data/dual_fr3_boxlift_real/`로 옮기고 bag도 남긴다(미끄럼·판 높이 분석용).

| 순서 | `model_path` (`fr3_husky_nn_policy/models/`) | 시작 자세 후보 (`model/`) |
|---|---|---|
| (11-05) | `facegrip_squeezelift14_model_6697.npz` | `…_facegrip_squeezelift_model6697_start_poses.yaml` |
| 1 | `facegrip_squeezelift_lowmuflat_model_7200.npz` | `…_facegrip_squeezelift_lowmuflat_model7200_start_poses.yaml` |
| 2 | `facegrip_squeezelift_lowmuflat_model_7100.npz` | `…_facegrip_squeezelift_lowmuflat_model7100_start_poses.yaml` |
| 3 | `facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_4700.npz` | `…_facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model4700_start_poses.yaml` |
| 3' | `facegrip_boxdrop_flatsqueeze_lowmusoftlift_model_4500.npz` | `…_facegrip_boxdrop_flatsqueeze_lowmusoftlift_model4500_start_poses.yaml` |

sim 비교 (Isaac, 1024 env, 4 s, DR·외란. 미끄럼은 512 env `sim_slip_probe`):

| | 6697 | LowMuFlat 7200 | LowMuFlat 7100 | GoalXY 4700 | LowMuSoftLift 4500 | FlatSqueeze 3998 |
|---|---|---|---|---|---|---|
| 실물 | 미끄럼 + 회전, 판정 기록 없음 | 안 함 | 안 함 | 안 함 | 1회, 판정 기록 없음 | 12회 중 1회 |
| 학습 | SqueezeLift | 6697 + 접촉 마찰 0.3~0.85, 503 iter | 같은 run, 403 iter | 4500 + goal x/y, 200 iter | 3998 + 접촉 마찰 0.21~0.64, 들기 가속 제한, 502 iter | FlatSqueeze |
| 성공 기본 / 낮은 마찰 (접촉 0.3~0.85) | 99.5 / 92.5% | 100 / 99.7% | 100 / 99.9% | 100 / 99.9% | 100 / 100% | 100 / 99.8% |
| Kp × 0.7 / × 0.5 | 82.7 / 6.9% | 98.8 / 49.3% | 98.2 / 51.0% | 99.9 / 88.8% | 100 / 87.7% | 99.7 / 65.1% |
| 박스 pose 지연 0~150 ms + 끊김 | 96.0% | 94.8% | 95.9% | 학습에 포함 | 학습에 포함 | 학습에 포함 |
| 들 때 최대 기울기 median / p90 | 7.8 / 10.5° | 8.2 / 10.5° | 10.2 / 13.2° | 6.6 / 9.8° | 6.3 / 10.8° | 3.6 / 6.0° |
| 학습 마찰: 미끄럼 (0.25→0.6 s) / 0.6 s 판 중심 − 박스 중심 | 2.3 / +10.8 cm | 0.5 / +9.7 cm | 0.4 / +9.2 cm | −0.2 / −1.3 cm | 0.0 / +1.5 cm | 0.1 / +5.8 cm |
| 접촉 마찰 0.25: 미끄럼 median / 최대 기울기 > 20° | 8.8 cm / 50% | 6.3 cm / 17% | 6.4 cm / 22% | 0.5 cm / 4% | 1.4 cm / 7% | 4.4 cm / 25% |

- 7200·7100은 6697의 파지(판이 박스 중심 위 약 10 cm)를 그대로 두고, 낮은 마찰·약한 Kp·접촉 마찰 0.25에서 6697보다 낫다. 실물 시험은 안 했다.
  7200은 들 때 기울기와 마찰 0.25 기울기가 7100보다 작고, 7100은 goal 정확도(2.3 vs 3.5 cm)가 조금 낫다.
- 4700·4500은 sim에서 미끄럼이 가장 적고, 판이 박스 중심 높이에서 잡는다. 같은 run의 GoalXY 5499를 11-07 실물에 올려 10회 중 8회 성공했다([5499 보고서](dual_fr3_boxlift_5499_report.md)).
- 쓰지 않는 것: 3998(실물 12회 중 11회 실패), 4997(판을 박스 중심 아래까지 내리고 관절 한계 여유 0.05 rad. 5499도 같은 경향이지만 11-07 실물에서 통했다), LowMuFlat 7696(박스 yaw 33°, export 안 함), 2999·3700·4699·5698(이전 단계).

### ready 자세 시작 후보: LMSL 5800 (4090, 2026-11-05)

- `fr3_husky_nn_policy/models/dual_fr3_boxlift_ready_wide_j1mirror_fgbd_flatsqueeze_lowmusoftlift_model5800.npz`
  - 학습: `dual_fr3_lab` `...-Light-Box410-Wide-J1Mirror-FaceGrip-BoxDrop-FlatSqueeze-LowMuSoftLift`(ready 자세에서 접근부터 들기까지 전부 RL).
  - 결과: `dual_fr3_lab/docs/dual_fr3_lift_sim2real.md`의 "box ready 시작" 장.
- 위 후보와 다른 점:
  - **MoveIt 시작 자세가 없다.** 팔을 ready 자세(`move_to_joint` 인자 없이, manifest `default_position`과 같음)에 두고 박스를 놓은 뒤 바로 `start_policy`한다. `boxlift_start_helper`는 표가 없다고만 알려 준다.
  - 박스 범위: 중심 x 0.40~0.50, y ±0.10, yaw ±10°.
  - goal x/y를 박스와 따로 학습했다(x 0.40~0.50, y ±0.10). `goal_xy_offset_m`으로 옮기며 들기를 줄 수 있고, 시작 박스 + offset이 범위 밖이면 시작이 거부된다.
  - 학습 접촉 마찰 0.2~0.65(박스 0.1~0.8, 판 0.3~0.5), 박스 크기 ±1 cm.
  - MuJoCo로는 못 돌린다. 지금 launch(`mujoco_start_row`)는 pregrasp 표가 있는 정책만 띄운다.
- 실물 결과 없음. 접근 동작도 정책이 하므로 shadow로 먼저 본다.

## (이전) 실로봇 정책: FaceGrip 3700 먼저 (2026-11-04)

지금은 위 [실로봇 후보](#실로봇-후보-2026-11-05)를 쓴다. 아래는 기록으로 남긴다.


- **실로봇은 `dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_model3700.npz`로 한다.** 시작 자세는 RollAlign 후보(맨 아래 표의 RollAlign 줄)와 같다.
- RollAlign 2999는 후보에서 뺐다.
  - MuJoCo: 들기 시작 0.4 s에 박스가 판 사이에서 조이는 축(y)으로 약 55° 돌고, goal까지 못 든다(접촉 모델을 바꿔도 같다).
  - Isaac: 박스 회전 속도 제한(`max_angular_velocity` 30 deg/s, 설정 실수)을 풀면 성공 100% → 79%, 들 때 기울기 p90 42°.
- FaceGrip 3700
  - MuJoCo elliptic: 6회 모두 goal 근처(+0.11~0.13)까지 들고 놓치지 않았다. 들 때 최대 기울기 32~41°.
  - Isaac 제한 해제: 성공 99.7%, 기울기 p90 16.5°.
- 남은 약점: 들면서 판이 박스 면을 타고 올라간다(2 s에 판 중심이 박스 중심 위 Isaac +10.5 cm, MuJoCo +17 cm). 회전 속도 제한을 고친 물리에서 이어 학습 중이다(dual_fr3_lab).
- **지금 실로봇에서 할 수 있는 것은 shadow(1~5번)까지다.** 명령 모드(6번)는 비전 `/box_pose`가 붙은 뒤에 한다.

## 목표 위치(goal)와 RViz 표시 (2026-11-05)

### 목표가 정해지는 방식

- 목표 = (시작 박스 x + `goal_xy_offset_m[0]`, 시작 박스 y + `goal_xy_offset_m[1]`, `goal_z_m`), 회전 없음. 모두 base 좌표계, m.
- 시작 박스 x/y는 `start_policy`를 받은 순간 정책이 보는 박스 위치(`box_position_offset_m` 적용 후)다. 실행 중에는 바뀌지 않는다.
- `goal_z_m`, `goal_xy_offset_m`은 `start_policy` 때마다 다시 읽는다. 세션 사이(IDLE)에 `ros2 param set`으로 바꾸면 되고, launch를 다시 켤 필요가 없다.
- 노드는 학습 범위를 NPZ manifest에서 읽어 범위 밖 값을 거부한다.
  - z: `goal.z_range_base` (지금 모델들 0.15~0.35)
  - x/y는 체크포인트가 목표를 뽑은 방식에 따라 둘 중 하나다.
    - `goal.xy_offset_range_base`: 시작 박스 기준 오프셋 범위. 수직 들기 모델은 `[0, 0]`이다. 이 항목이 없는 예전 체크포인트도 `[0, 0]`만 허용한다.
    - `goal.xy_range_base`: 목표 x/y의 절대 범위(base). 학습 때 목표를 박스와 상관없이 이 범위에서 뽑았다(GoalXY 모델). 시작 박스 + 오프셋이 이 범위 안이어야 한다. 그래서 같은 오프셋도 박스 위치에 따라 거부될 수 있고, 박스가 범위 밖(예: x 0.39)에 있으면 오프셋 0도 거부된다.
- 거부되는 때
  - `ros2 param set`: 형식이 틀렸거나, z가 범위 밖이거나, 어떤 학습 시작 위치에서도 학습 범위에 닿지 않는 오프셋이면 바로 실패한다. 세션이 PENDING/RUNNING이면 goal 값 변경 자체를 거부한다.
  - `start_policy`: 지금 박스 위치로 계산한 목표가 학습 범위 밖이면 이유와 함께 거부한다. 성공하면 응답에 목표가 찍힌다.

### 목표 x/y를 옮기는 체크포인트를 쓸 때

지금 받은 모델 중 GoalXY 두 개(`models/facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_{4700,5499}.npz`)가 목표 x/y를 옮길 수 있다. 학습 범위는 목표 x 0.40~0.50, y −0.10~0.10 (base)이다. 시작 자세 후보는 `model/dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model{4700,5499}_start_poses.yaml`(후보 A~E와 같은 박스 다섯 위치)다.

launch는 오프셋 0으로 켜고, 시도마다 터미널에서 바꾼다. x +는 로봇에서 멀어지는 쪽, y +는 로봇 왼쪽이다. 값은 소수점으로 쓴다. `[0, 0.05]`처럼 정수가 섞이면 타입 오류로 거부된다.

```bash
ros2 param set /ppo_boxlift_policy_node goal_xy_offset_m "[0.0, 0.05]"
ros2 param set /ppo_boxlift_policy_node goal_z_m 0.25
# RViz 'Box-lift goal'이 옮겨진 자리에 초록이면 시작 가능, 빨강이면 start_policy가 거부한다
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
```

- 처음 값은 launch 인자(`goal_xy_offset_m:="[0.03, -0.02]"`, `goal_z_m:=0.25`)로 줄 수도 있다.
- `boxlift_start_helper`도 지금 박스 위치로 목표를 계산한다. 거부될 목표면 `REFUSED by start_policy: ...` 줄을 찍는다.
- MuJoCo(`use_mujoco:=true`) 뷰어의 초록 목표 박스는 launch 인자로 준 처음 값만 반영한다. `ros2 param set`은 노드 목표와 RViz만 바꾼다.

#### 목표를 범위 중앙 기준으로 주기: `goal_xy_reference` (2026-11-07)

기본(`box_start`)은 목표 = 시작 박스 x/y + offset이라, 박스를 놓을 때마다 목표가 같이 움직인다. `goal_xy_reference:=range_center`로 띄우면 목표 = **학습한 목표 범위의 중앙 + offset**이다. 박스를 어디에 놓든 같은 offset이면 같은 목표다.

| 모델 | 목표 범위 (base) | 중앙 | `range_center`일 때 offset 범위 |
|---|---|---|---|
| GoalXY 4700/5499 | x 0.40~0.50, y −0.10~0.10 | (0.45, 0.00) | x −0.05~+0.05, y −0.10~+0.10 |

- 시작 박스 표의 범위(x 0.397~0.500, y −0.107~0.108)도 중앙이 (0.449, 0.001)이라 목표 범위 중앙과 거의 같다.
- `goal.xy_range_base`가 없는 모델(목표가 시작 박스 기준으로만 학습됨)에서는 `range_center`를 거부한다.
- launch 인자 `goal_xy_reference:=range_center`로 주거나, IDLE에서 `ros2 param set /ppo_boxlift_policy_node goal_xy_reference range_center`로 바꾼다. offset 범위를 넘으면 `ros2 param set`이 바로 거부된다.
- `boxlift_start_helper`, RViz 목표 박스, `metadata.json`(`parameters.goal_xy_reference`)에 모두 반영된다.

모델의 학습 범위 확인:

```bash
cd ~/jw_ws/src/fr3_husky/fr3_husky_nn_policy
python3 -c "import numpy as np,json,sys; g=json.loads(str(np.load(sys.argv[1])['manifest_json']))['goal']; print('xy offset:', g.get('xy_offset_range_base', 'none (0 only)'), ' xy absolute:', g.get('xy_range_base', 'none'), ' z:', g['z_range_base'])" models/<새모델>.npz
```

launch 로그의 `Loaded ...` 줄에도 `trained offset x [...], y [...]` 또는 `trained absolute goal x [...], y [...]`가 나온다.

### 학습 PC: manifest의 goal 항목 (dual_fr3_lab)

`dual_fr3_lab/scripts/tools/export_boxlift_actor.py`가 학습 command 설정을 보고 둘 중 하나를 쓴다(2026-11-05부터).

수직 들기(`VerticalLiftPoseCommand`):

```json
"goal": {
  "command_class": "...VerticalLiftPoseCommand",
  "z_range_base": [0.15, 0.35],
  "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
  "xy_offset_range_base": {"x": [0.0, 0.0], "y": [0.0, 0.0]}
}
```

GoalXY(`UniformPoseCommand`, 박스와 상관없이 목표를 뽑는 학습):

```json
"goal": {
  "command_class": "isaaclab.envs.mdp.commands.pose_command:UniformPoseCommand",
  "z_range_base": [0.15, 0.35],
  "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
  "xy_range_base": {"x": [0.4, 0.5], "y": [-0.1, 0.1]}
}
```

- `xy_offset_range_base`는 시작 박스 기준 오프셋(m), `xy_range_base`는 목표 절대 좌표(m)의 축별 (low, high)다. 둘 다 있으면 노드가 manifest를 거부한다.
- 노드 쪽 파싱: `BoxLiftManifest.goal_xy_offset_range` / `goal_xy_range`. 검사: `boxlift_core.check_goal_settings`(박스와 무관한 값 검사, `ros2 param set`), `boxlift_core.make_goal`(시작 박스까지 넣은 검사, `start_policy`).
- 오프셋을 원형(반지름)으로 뽑는 학습은 아직 형식이 없다. 그런 학습을 만들면 exporter와 노드 파싱·검사를 같이 바꾼다.

### RViz 표시

launch가 띄우는 RViz(`fr3_husky_controller/rviz/fr3.rviz`)에 다음이 들어 있다.

| Displays 이름 | 토픽 | 색 | 내용 |
|---|---|---|---|
| Box-lift goal | `/ppo_boxlift_policy_node/goal_marker` | 초록 (빨강) | 시작 전: 지금 박스 위치와 지금 goal 파라미터로 계산한 목표(박스를 따라 움직임). 빨강이면 학습 범위 밖이라 `start_policy`가 거부한다. 실행 중: `start_policy` 순간 고정된 목표 |
| Box-lift policy box | `/ppo_boxlift_policy_node/box_marker` | 주황 | 정책이 보는 박스(비전 + `box_position_offset_m`), 20 Hz |
| Vision box (raw /box_tf) | `/box_tf/marker` | 초록(비전 노드) | 기본 꺼짐. 비전 원래 값 |

실행 중 주황 박스가 초록 박스 안으로 들어가면 목표에 도달한 것이다. `boxlift_start_helper`도 `goal x … y … z …` 줄로 목표를 보여 준다. 세션 `metadata.json`에는 실제 목표가 `goal_base`, 넣은 오프셋이 `parameters.goal_xy_offset_m`으로 남는다.

## 실행 전 확인 (실로봇, MuJoCo 공통)

1. **pull 후 다시 빌드한다(0번).** launch 인자 `box_size_m`이 새로 생겼다. 빌드하지 않으면 launch가 이 인자를 모른다.
2. **launch에 `box_size_m:="[0.310, 0.410, 0.280]"`를 준다.** 빼면 기본값이 410 앞뒤 박스라 정책 로드가 거부된다.
3. **팔은 쓰는 정책의 후보 자세로 보낸다(2번).** RollAlign과 Pregrasp는 j7이 0.25 rad 달라서, 다른 정책 후보로 보내면 시작 검사에서 거부된다.
4. **박스 pose를 끊기지 않게 넣는다.** 두 정책 모두 매 step 실시간 박스 pose를 보고 든다.
   - `/box_pose`: 박스 **중심**(윗면 마커 아님), base frame.
   - 명령 모드는 비전 pose로만 한다. 고정 publish로 들면 sim 성공 0~8%다(4번 주의).
   - 0.30 s 넘게 안 들어오면 노드가 fault로 멈추고, 조임이 풀려 박스가 떨어진다. 이 정책은 비전 끊김을 학습하지 않았다(끊김 학습 `-Wide-RollAlign-BoxDrop`은 dual_fr3_lab에서 준비 중).
5. **박스는 학습 조건과 같게:** 310(앞뒤) × 410(좌우) × 280 mm, 0.4~0.6 kg. MuJoCo에 박스를 넣을 때도 같다.
6. **놓친 박스는 다시 잡지 않는다.** 학습에서 박스를 놓친 상태를 거의 겪지 않았다. MuJoCo나 실물에서 미끄러져 놓치면 그대로 끝난다.
7. **판을 V자로 기울여 든다(RollAlign 약 25°, FaceGrip 11~16°).** 판 아래 모서리로 받치는 선 접촉이라 접촉 모델이나 실물 마찰에 따라 미끄러질 수 있다. 종이 박스라면 모서리 눌림을 본다.
8. **`dyros_robot_controller`는 jw_ws 복사본을 쓴다(0번 2).** 다른 workspace 라이브러리가 올라오면 컨트롤러가 configure에서 죽을 수 있다.

## 박스 방향

**긴 변(410 mm)이 로봇 좌우(y)**다. 판은 박스 양 끝의 310 × 280 mm 면을 잡고, 410 mm 방향으로 조인다.

- 박스 frame: x = 310 mm 변(앞뒤), y = 410 mm 변(좌우, 판이 조이는 방향), z = 280 mm.
- 노드의 `box_size_m`은 `[0.310, 0.410, 0.280]`이다(1번 launch 인자). 기본값 `[0.410, 0.310, 0.280]`이면 정책 로드가 거부된다.

## 정책 두 개

시작 자세만 다르다. 두 표는 같은 박스 위치에서 만들었고, 관절 j1~j6이 같고 **j7(손목)만 약 0.25 rad 다르다**.

| | RollAlign (후보 제외, 위 "실로봇 정책") | Pregrasp |
|---|---|---|
| 정책 | `model/dual_fr3_boxlift_pregrasp_wide_rollalign_model2999.npz` | `model/dual_fr3_boxlift_pregrasp_wide_model2999.npz` |
| 시작 자세 후보 | `model/dual_fr3_boxlift_pregrasp_wide_rollalign_model2999_start_poses.yaml` | `model/dual_fr3_boxlift_pregrasp_wide_model2999_start_poses.yaml` |
| 학습 task (`dual_fr3_lab`) | `...-Light-Pregrasp-Wide-RollAlign-v0` | `...-Light-Pregrasp-Wide-v0` |
| 시작 때 판 | 판 모서리가 박스 모서리와 나란 (0°) | 판이 면 안에서 약 16° 돌아감 |
| sim 성공 (1024 env, DR·외란) | 100% | 100% |
| 성공까지 | 1.40 s | 1.65 s |
| 끝 goal 거리 / 기울기 | 2.4 cm / 2.9° | 3.7 cm / 3.4° |
| 들 때 박스 최대 기울기 | 12.0° (p90 14.0) | 9.3° (p90 11.9) |
| 끝 박스 yaw | 1.5° (p90 2.9) | 2.0° (p90 4.8) |
| 판 최대 힘 | 20 N | 18 N |
| 들 때 좌우 판 높이 차 | 0.5 cm | 2.3 cm |
| 판 V자 기울기 최대 (0.5~0.75 s) | 약 25° | 약 16° |
| FR3 한계 여유 최소 | 0.064 rad | 0.056 rad |
| 팔 움직임 | 양팔 대칭 (주로 j2·j4) | **비대칭**: 오른팔이 j1·j3·j5를 1.0~1.4 rad씩 돌림 |

- 경로는 모두 `~/jw_ws/src/fr3_husky/` 아래다(husky-rcu PC).
- 둘 다 3000 iter, 박스 310 × 410 × 280 mm, 0.4~0.6 kg, 박스 중심 x 0.40~0.50 m.

### 판 면 파지 후보: RollAlign-FaceGrip model_3700 (2026-11-04)

RollAlign 2999에서 판-면 각도 벌점을 넣어 700 iter 이어 학습했다. 들 때 판 V자를 줄인 정책이다. MuJoCo나 실물에서 2999와 비교해 덜 미끄러지는지 본다.

- 정책: `model/dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_model3700.npz`
- **시작 자세는 RollAlign과 같다.** 표가 같으므로 RollAlign 후보(2번, "MoveIt 시작 자세 후보"의 RollAlign 줄)를 그대로 쓴다. 후보 파일 `..._facegrip_model3700_start_poses.yaml`도 같은 줄을 가리킨다.
- launch는 `model_path`만 바꾼다. `box_size_m`은 같다.
- sim 비교 (1024 env, DR·외란):

  | | RollAlign 2999 | FaceGrip 3700 |
  |---|---|---|
  | 성공 | 100% | 100% |
  | 끝 goal 거리 | 2.4 cm | 3.4 cm (조금 더 높이 든다) |
  | 판 V자 기울기 0.5 / 0.75 s | 24° / 26° | 16° / 11° |
  | 들고 있을 때(1.5 s) 판 기울기 | 6.6° | 4.4° |
  | 판 최대 힘 | 20 N | 17 N |

- 학습 task: `dual_fr3_lab` `...-Light-Pregrasp-Wide-RollAlign-FaceGrip-v0`. 자세한 결과는 `dual_fr3_lab/docs/dual_fr3_lift_sim2real.md`의 "판 면 파지".
- 노드 설정: `fr3_husky_nn_policy/config/dual_fr3_boxlift.yaml`.
- 정책 제어: 20 Hz, 관절 목표 = 측정 q + ±0.1 rad, base frame.
- RollAlign 계열(2999, FaceGrip 3700)을 Pregrasp보다 먼저 쓰는 이유:
  - 양팔이 대칭으로 움직이고, 좌우 판 높이가 맞는다.
  - Pregrasp는 오른팔이 크게 돌아가는 자세로 든다(sim 영상에서 오른팔 팔꿈치가 옆으로 눕는다).
- RollAlign 계열 안에서는 FaceGrip 3700을 쓴다(위 "실로봇 정책").
- 정책을 바꾸면 2번 `move_to_joint`도 그 정책의 후보로 다시 보낸다. j7 차이 0.25 rad가 시작 검사 허용 0.03 rad보다 크다.

## 0. 준비 (한 번)

1. 시작 검사는 설정을 바꿀 필요가 없다.
   - 노드가 npz 매니페스트의 `start.mode`(`pregrasp_table`)를 보고, 팔과 박스가 표의 한 줄에 맞는지 본다. 표는 npz 안에 있다.
   - 허용 범위: 관절마다 0.03 rad, 박스 x·y ±1 cm, yaw ±0.05 rad.
   - `ready_tolerance_rad`는 이 정책에서 쓰지 않는다.
2. 빌드 (pull한 뒤에도 다시 한다):

   ```bash
   cd ~/jw_ws
   colcon build --symlink-install --allow-overriding mujoco_ros_hardware \
     --packages-select mujoco_ros_hardware fr3_husky_description fr3_husky_controller fr3_husky_nn_policy
   source install/setup.bash
   ```

   - husky-rcu PC 기준이다. `mujoco_ros_hardware`와 `dyros_robot_controller`는 js_ws에서 복사한 jw_ws 복사본을 쓴다(아래).
   - 실로봇 터미널은 `.bashrc`의 `ROS_DOMAIN_ID=23`을 그대로 쓴다. MuJoCo를 같이 띄울 때만 다른 domain으로 바꾼다.
   - **`dyros_robot_controller`** (2026-11-04 정리, husky-rcu에서 이미 했다)
     - 문제: `jw_ws`는 jh_ws/ros2_ws 위에 올라가 있는데 `fr3_husky_controller`는 js_ws 헤더로 빌드됐다. 실행할 때 jh_ws의 `libdyros_robot_controller.so`가 올라와 헤더와 라이브러리가 달랐다. 그 결과 `dual_fr3_action_controller` configure에서 `munmap_chunk(): invalid pointer`로 죽었다(박스 크기만 바꿔도 죽고 안 죽고가 바뀌었다).
     - 해결: js_ws/etri_ws의 `dyros_robot_controller`(커밋 안 된 pinocchio 4 수정 포함)를 `~/jw_ws/src`로 복사해 같이 빌드한다. 이제 source만 하면 jw_ws 라이브러리가 올라온다.
     - 확인: `ldd ~/jw_ws/install/fr3_husky_controller/lib/libfr3_husky_controller.so | grep dyros`가 `~/jw_ws/install/dyros_robot_controller`를 가리켜야 한다.
     - build 폴더를 지우고 다시 빌드할 때만 js_ws와 같은 cmake 인자가 필요하다. 없으면 `/opt/openrobots`의 다른 pinocchio 헤더가 섞여 빌드가 실패한다.

       ```bash
       cd ~/jw_ws
       colcon build --symlink-install --packages-select dyros_robot_controller --cmake-args \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -Dpinocchio_DIR=/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio \
         -Deigenpy_DIR=/opt/ros/humble/lib/x86_64-linux-gnu/cmake/eigenpy \
         -Dcoal_DIR=/opt/ros/humble/lib/x86_64-linux-gnu/cmake/coal
       colcon build --symlink-install --packages-select fr3_husky_controller \
         --cmake-args -Ddyros_robot_controller_DIR=$HOME/jw_ws/install/dyros_robot_controller/share/dyros_robot_controller/cmake
       ```

3. 바닥 표시: [dual_fr3_box_real_setup.md](dual_fr3_box_real_setup.md)의 "Wide 박스 놓는 방법"대로 표시한다.
   - O'(두 팔 장착점 중점을 바닥에 내린 점)를 찍는다.
   - O'에서 로봇 정면 방향으로 중심선을 긋는다.
   - 쓸 후보의 "로봇 쪽 면 x" 위치에 가로선을 긋는다(후보 A는 O'에서 0.293 m).
4. base 높이 확인: 바닥에서 팔 장착면까지 높이를 잰다.
   - sim 학습값은 0.405 m, 실물 URDF는 0.392 m다.
   - 4번의 박스 z가 이 값에 달려 있다: z = −(base 높이) + 0.140.

## 1. 터미널 1: 컨트롤러 + move_group + box lift 노드 (shadow)

```bash
source ~/jw_ws/install/setup.bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$HOME/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_model3700.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  shadow_mode:=true launch_move_group:=true goal_z_m:=0.15
```

- 다른 정책은 `model_path`만 바꾼다: RollAlign `..._wide_rollalign_model2999.npz`, Pregrasp `..._wide_model2999.npz`.
- `box_size_m`: 실물 박스 (앞뒤, 좌우, 높이). 정책의 학습 박스와 ±5 mm 안에서 맞아야 노드가 뜬다.
- `shadow_mode:=true`: 추론과 로그만 하고 로봇에는 명령을 보내지 않는다. 처음에는 반드시 이것으로 한다.
- `launch_move_group:=true`: 2번의 `move_to_joint`가 쓰는 MoveIt을 같이 띄운다.
- `goal_z_m`: 박스 중심을 올릴 높이(base frame)다. 학습 범위 0.15~0.35에서 가장 낮은 0.15로 시작한다(바닥에서 41.5 cm 들기).
- MuJoCo로 먼저 확인할 때는 맨 아래 [Isaac → MuJoCo sim-to-sim](#isaac--mujoco-sim-to-sim) 절차를 따른다(박스와 시작 자세가 자동으로 놓인다).

## 2. 터미널 2: 팔을 시작 자세로 (박스 없이)

```bash
source ~/jw_ws/install/setup.bash
# (1) ready 자세
ros2 run fr3_husky_task_manager move_to_joint
# (2) 시작 자세: 후보 A (박스 중심 x 0.448, y 0), FaceGrip 3700 / RollAlign 정책 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0012 1.6314 -0.1775 -1.3488 -1.0002 1.6352 0.5559 --right-target-positions -0.0102 1.6209 0.1723 -1.3833 1.0213 1.6466 0.9907
```

- Pregrasp 정책이면 맨 아래 후보 표의 Pregrasp 줄을 쓴다(j7만 다르다).
- **박스를 놓기 전에** 움직인다. `MoveToJoint`는 박스를 모르므로, 박스가 먼저 있으면 판이 박스를 지나는 경로가 나올 수 있다.
- 기본 속도·가속도 배율은 0.1이다.
- 도착하면 판 중심이 바닥에서 약 19 cm(박스 중심 + 5 cm)에 선다. 박스가 놓일 자리의 양 끝 면에서 4 cm 바깥이다.
  - RollAlign: 판 모서리가 수평·수직이라 바닥에서 약 12~26 cm를 덮는다.
  - Pregrasp: 판이 약 16° 돌아가 있어 약 10~28 cm를 덮는다.

## 3. 박스 놓기

- 후보의 박스 위치에 놓는다. 후보 A라면:
  - 로봇 쪽 면(310 mm 변)이 O'에서 0.293 m 앞이다.
  - 박스 중심이 중심선 위(y 0)다. 양 끝 면은 y +0.204, −0.206이다.
  - 410 mm 변이 중심선에 수직이다(yaw 0). 즉 긴 변이 좌우다.
- 로봇 앞쪽(+x)에서 두 판 사이로 밀어 넣는다. 판과 박스 끝 면 사이가 양쪽 4 cm씩이다.
- 허용 오차는 학습 노이즈만큼이다: 위치 ±1 cm, yaw ±3°. 노드가 시작할 때 이 범위를 검사한다.
  - 넓혀야 하면 `start_position_margin_m`, `start_yaw_margin_rad`만큼 더해진다. 학습 분포 밖이 되므로 결과 해석에 주의한다.

## 4. 터미널 3: 박스 pose publish (비전 대신 고정값)

```bash
source ~/jw_ws/install/setup.bash
# 후보 A, base 높이 0.392 m일 때: z = -0.392 + 0.140 = -0.252, yaw -0.7 deg
ros2 topic pub -r 30 /box_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: 0.448, y: -0.001, z: -0.252}, orientation: {x: 0.0, y: 0.0, z: -0.0065, w: 1.0}}}"
```

- 박스 **중심** pose를 base frame으로 보낸다. 박스 x축은 310 mm 변(앞뒤), y축은 410 mm 변(좌우) 방향이다.
- 노드 시작 검사(`start_policy` 때 한 번):
  - 팔과 박스가 표의 같은 줄에 맞아야 한다. 관절마다 ±0.03 rad, 박스 x·y ±1 cm, yaw ±0.05 rad. 2번에서 보낸 후보 줄과 같은 줄이 잡혀야 한다(응답 message에 줄 번호가 나온다).
  - 박스 z가 학습값 −0.265에서 ±0.03 안이어야 한다.
- **주의: 고정값은 박스가 들려도 바뀌지 않는다.** 정책은 매 step 박스 위치·자세를 관측하므로, 들기 시작한 뒤의 관측이 실제와 달라진다.
  - shadow 확인에는 문제없다.
  - **명령 모드로 들지 않는다 (2026-11-04 sim 확인).** sim에서 박스 관측을 에피소드 시작 값으로 고정하면(= 이 고정 publish) 두 정책 모두 실패했다.
    - RollAlign 성공 0%, Pregrasp 8%.
    - 박스는 들지만 goal을 약 20 cm 지나쳐 들고, 박스가 25~56° 기운다. 판이 아닌 부분이 박스에 닿는 경우도 57%였다.
    - `/box_pose`를 비전(Azure Kinect) 값으로 바꾼 뒤에 명령 모드로 간다. 박스 관측을 고정해 학습하는 `-Wide-RollAlign-FrozenBox`는 2026-11-04에 중단했다(비전이 있어 필요 없음).
- 박스 pose가 0.30 s 넘게 안 들어오면 노드가 fault를 낸다(`box_pose_timeout_s`). publish를 계속 켜 둔다.

## 5. 터미널 4: 정책 시작과 정지

```bash
source ~/jw_ws/install/setup.bash
ros2 service call /ppo_boxlift_policy_node/start_policy std_srvs/srv/Trigger {}
# 정지
ros2 service call /ppo_boxlift_policy_node/stop_policy std_srvs/srv/Trigger {}
```

- 시작이 거부되면 응답 message에 이유가 나온다.
  - 예: `not at a trained pre-grasp start: nearest table row 1965 (...) has box x/y 0.015 m off (allowed 0.010)`, 박스 z가 바닥이 아니다.
  - 가장 가까운 표 줄과, 팔·박스가 각각 얼마나 벗어났는지 나온다. 팔이 벗어났으면 `move_to_joint`를 다시, 박스가 벗어났으면 박스를 다시 놓는다.
  - 팔 j7만 0.25 rad쯤 벗어났다면 다른 정책의 후보로 보낸 것이다.
- 시작되면 응답 message와 `metadata.json`(`start_table_row`)에 잡힌 줄 번호가 남는다.
- 세션마다 로그 폴더(`metadata.json`, `trace.csv`)가 생긴다(`log_root`, 비우면 기본 data 경로).
- shadow에서 확인할 것:
  - 정책 출력 크기(관절 목표 변화 ≤ 0.1 rad).
  - 첫 몇 step 동안 판이 박스 쪽으로 다가가는 방향의 목표가 나오는지.
  - fault가 없는지.

## 6. 명령 모드

> 비전 없이 고정 publish로는 하지 않는다(4번 주의). sim에서 성공 0~8%였다.

1. 터미널 1을 끄고 `shadow_mode:=false`로 다시 띄운다.
2. 2~5번을 반복한다.

```bash
source ~/jw_ws/install/setup.bash
ros2 launch fr3_husky_nn_policy dual_fr3_boxlift_policy.launch.py \
  model_path:=$HOME/jw_ws/src/fr3_husky/model/dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_model3700.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  shadow_mode:=false launch_move_group:=true goal_z_m:=0.15
```

- Isaac에서 시작 후 약 1.4~1.65 s 안에 조여서 goal 높이까지 든다. 판 최대 힘은 17~21 N이다.
- 학습 에피소드는 4 s다. `max_duration_s: 0.0`이면 `stop_policy`까지 계속 돈다.
- **정지하면 조임이 풀려 박스가 떨어진다**(아직 hold 모드 없음). 박스 아래를 받치거나 낮은 goal로 시험한다.
- 알려진 거동:
  - 두 정책 모두 들 때 판을 V자로 기울여 아래 모서리로 받친다. RollAlign 약 25°, Pregrasp 약 16°.
  - Pregrasp는 오른팔이 j1·j3·j5를 1 rad 넘게 돌려 팔꿈치를 옆으로 눕힌다. 주변에 사람이나 물체가 없게 한다.

---

## MoveIt 시작 자세 후보

- `dual_fr3_lab/scripts/tools/pregrasp_start_poses.py`로 각 정책 npz 안의 학습 표에서 뽑았다.
  - 요청한 박스 위치에 가장 가까운 표 줄을 고른다. 거리 기준은 max(|dxy| / 1 cm, |dyaw| / 0.05 rad)다.
  - 두 정책의 표는 줄 번호, 박스 pose, j1~j6이 같다. **j7만 다르다.**
  - 다른 위치가 필요하면: `python scripts/tools/pregrasp_start_poses.py <npz> <yaml> --box X Y YAW_DEG ...`
- 박스 중심 z는 sim 기준 −0.265다. 실물 base 높이가 다르면 4번의 z를 그에 맞춘다.
- 관절 순서는 각 팔 j1~j7, 단위는 rad다.
- sim 확인: 다섯 줄에서 노이즈 없이 500 env, 질량·마찰 DR과 외란을 켜고 돌렸다.
  - RollAlign: 성공 100%, 성공까지 1.40 s, FR3 한계 여유 최소 0.129 rad.
  - Pregrasp: 성공 100%, 성공까지 1.65 s, FR3 한계 여유 최소 0.112 rad.

| 후보 | 표 줄 | 박스 중심 x, y [m], yaw | 로봇 쪽 면 x [m] | `/box_pose` orientation z | 관절 여유 [rad] | j7 L / R (RollAlign) | j7 L / R (Pregrasp) |
|---|---|---|---|---|---|---|---|
| **A** (첫 시험) | 1965 | 0.448, −0.001, −0.7° | 0.293 | −0.0065 | 0.102 | 0.5559 / 0.9907 | 0.8009 / 0.7834 |
| B | 2790 | 0.419, −0.003, −0.5° | 0.264 | −0.0045 | 0.140 | 0.6206 / 0.9367 | 0.8056 / 0.7865 |
| C | 2166 | 0.477, +0.002, 0.0° | 0.322 | −0.0003 | 0.065 | 0.4975 / 1.0713 | 0.8160 / 0.7725 |
| D | 371 | 0.447, +0.047, +0.9° | 0.292 | 0.0079 | 0.096 | 0.5443 / 0.9783 | 0.8154 / 0.7853 |
| E | 1257 | 0.451, −0.052, +0.9° | 0.296 | 0.0082 | 0.069 | 0.6068 / 1.0668 | 0.7945 / 0.7396 |

- 관절 여유는 Margin 한계(FCI − 0.05 rad)까지 거리다.
- `/box_pose` orientation은 x 0, y 0, w 1.0에 위 z를 넣는다. 박스를 정확히 yaw 0으로 놓았다면 z 0으로 보내도 허용 범위(±2.9°) 안이다.

### A (박스 0.448, −0.001)

```bash
# RollAlign, FaceGrip 3700 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0012 1.6314 -0.1775 -1.3488 -1.0002 1.6352 0.5559 --right-target-positions -0.0102 1.6209 0.1723 -1.3833 1.0213 1.6466 0.9907
# Pregrasp
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0012 1.6314 -0.1775 -1.3488 -1.0002 1.6352 0.8009 --right-target-positions -0.0102 1.6209 0.1723 -1.3833 1.0213 1.6466 0.7834
```
`/box_pose` position x 0.448, y −0.001, orientation z −0.0065

### B (박스 0.419, −0.003)

```bash
# RollAlign, FaceGrip 3700 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0175 1.5936 -0.1633 -1.4554 -1.0029 1.6619 0.6206 --right-target-positions -0.0326 1.5915 0.1577 -1.4697 1.0069 1.6623 0.9367
# Pregrasp
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0175 1.5936 -0.1633 -1.4554 -1.0029 1.6619 0.8056 --right-target-positions -0.0326 1.5915 0.1577 -1.4697 1.0069 1.6623 0.7865
```
`/box_pose` position x 0.419, y −0.003, orientation z −0.0045

### C (박스 0.477, +0.002)

```bash
# RollAlign, FaceGrip 3700 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0073 1.6684 -0.1932 -1.2504 -1.0067 1.618 0.4975 --right-target-positions 0.0148 1.6629 0.1924 -1.2574 1.016 1.6234 1.0713
# Pregrasp
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0073 1.6684 -0.1932 -1.2504 -1.0067 1.618 0.816 --right-target-positions 0.0148 1.6629 0.1924 -1.2574 1.016 1.6234 0.7725
```
`/box_pose` position x 0.477, y 0.002, orientation z −0.0003

### D (박스 0.447, +0.047)

```bash
# RollAlign, FaceGrip 3700 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0854 1.6376 -0.1598 -1.3198 -0.9496 1.5957 0.5443 --right-target-positions 0.0705 1.6189 0.1932 -1.4058 1.0702 1.6852 0.9783
# Pregrasp
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions 0.0854 1.6376 -0.1598 -1.3198 -0.9496 1.5957 0.8154 --right-target-positions 0.0705 1.6189 0.1932 -1.4058 1.0702 1.6852 0.7853
```
`/box_pose` position x 0.447, y 0.047, orientation z 0.0079

### E (박스 0.451, −0.052)

```bash
# RollAlign, FaceGrip 3700 (같은 자세)
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0718 1.6059 -0.1967 -1.432 -1.1059 1.7056 0.6068 --right-target-positions -0.0806 1.6643 0.1649 -1.2464 0.9171 1.5643 1.0668
# Pregrasp
ros2 run fr3_husky_task_manager move_to_joint --left-target-positions -0.0718 1.6059 -0.1967 -1.432 -1.1059 1.7056 0.7945 --right-target-positions -0.0806 1.6643 0.1649 -1.2464 0.9171 1.5643 0.7396
```
`/box_pose` position x 0.451, y −0.052, orientation z 0.0082

## 이전 정책: 410 mm 앞뒤 (Pregrasp box410 model_2999)

긴 변을 앞뒤로 놓고 학습한 정책이다. 실물 박스를 90° 돌려 놓아야 해서 Wide 정책으로 바꿨다.

- 정책: `model/dual_fr3_boxlift_pregrasp_box410_model2999.npz`.
- 시작 자세 후보: `model/dual_fr3_boxlift_pregrasp_box410_model2999_start_poses.yaml`. 이 문서의 이전 판(commit ef5ae52)에 A~E 명령이 있다.
- launch에 `box_size_m:="[0.410, 0.310, 0.280]"`(기본값)을 쓴다.

---
---

# Isaac → MuJoCo sim-to-sim

Isaac에서 학습한 정책을 MuJoCo에서 inference한다. 실로봇과 **같은 노드, 같은 npz, 같은 컨트롤러**를 쓰고, 로봇과 박스만 MuJoCo로 바꾼다.

- 2026-11-04부터 Wide 정책(310 앞뒤 × 410 좌우)을 MuJoCo에서 돌릴 수 있다. MuJoCo 박스 크기는 launch 인자 `box_size_m`을 따른다.
- 아래 명령은 Wide FaceGrip 3700 기준이다. 이전 정책(box410 model_2999) 결과는 맨 아래 "이전 정책 결과"에 남겼다.

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
| `/box_pose` | 비전(Azure Kinect). shadow 확인은 `ros2 topic pub` 고정값도 됨 | MuJoCo가 GT를 30 Hz로 publish. **손으로 publish하지 않는다** |
| 들기 중 박스 관측 | 비전은 박스를 따라감. 고정값은 실제와 다름 | 실제 박스를 따라감 |
| 중력 | 로봇은 FCI가 보상 | 로봇 body는 `gravcomp`, 박스만 중력을 받음 |
| base 높이 | 실측 필요 | 학습값 0.405 m (`mujoco_mount_z:=0.2727`) |

## 0. 준비 (한 번)

1. MuJoCo 하드웨어 패키지: 박스 pose publisher가 들어간 `mujoco_ros_hardware`(`boxlift-object-pose` 브랜치)가 workspace에 있어야 한다.
   - husky-rcu PC: `~/jw_ws/src/mujoco_ros_hardware` (js_ws 버전을 복사해 overlay로 빌드했다). `dyros_robot_controller`도 같은 방식이다(실로봇 0번 2).
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
  model_path:=$PWD/model/dual_fr3_boxlift_pregrasp_wide_rollalign_facegrip_model3700.npz \
  box_size_m:="[0.310, 0.410, 0.280]" \
  use_mujoco:=true shadow_mode:=false launch_rviz:=false \
  mujoco_start_row:=1965 goal_z_m:=0.15 \
  mujoco_box_friction:=1.2
```

- `mujoco_start_row`: npz 안 pre-grasp 표의 줄 번호다. 팔 초기 관절각과 박스 x, y, yaw를 이 줄에서 읽는다.
  - Wide RollAlign / FaceGrip 후보: A 1965, B 2790, C 2166, D 371, E 1257 (실로봇 후보 A~E와 같은 줄).
  - box410 model_2999 후보: A 1919, B 3788, C 3925, D 720, E 2228 (`box_size_m`은 기본값).
- 마찰 모델은 launch 기본값이 elliptic, `impratio 10`이다. MuJoCo 기본(`mujoco_cone:=pyramidal mujoco_impratio:=1`)은 잡은 접촉이 계속 조금씩 미끄러져서(creep) 정책과 상관없이 박스를 놓친다(아래 "Wide 결과").
- `shadow_mode:=true`로 띄우면 추론과 로그만 하고 팔은 움직이지 않는다. 배포 경로만 볼 때 쓴다.
- 뷰어 창이 뜬다. launch 로그에 다음 두 줄이 보이면 준비된 것이다.
  - `[dual_fr3_boxlift] MuJoCo start: table row 1965, box (0.4482, -0.0009, yaw -0.74 deg), size [0.31, 0.41, 0.28]`
  - `Object pose publisher started: body=box frame=base topic=/box_pose (30.0 Hz)`
- 뷰어의 초록 반투명 박스가 goal이다(학습과 같게 x, y = 박스 시작 위치 + `goal_xy_offset_m`, z = `goal_z_m`, 회전 없음).

| launch 인자 (MuJoCo 전용) | 기본값 | 의미 |
|---|---|---|
| `box_size_m` (공통 인자) | `[0.410, 0.310, 0.280]` | 노드의 박스 검사와 MuJoCo 박스 크기. Wide는 `[0.310, 0.410, 0.280]` |
| `goal_xy_offset_m` (공통 인자) | `[0.0, 0.0]` | 목표 x/y = 시작 박스 x/y + 이 값. manifest `goal.xy_offset_range_base` 안이어야 한다([목표 위치](#목표-위치goal와-rviz-표시-2026-11-05)) |
| `mujoco_start_row` | 1919 | 시작 표 줄 (팔 + 박스). Wide는 1965 등 위 후보 |
| `mujoco_mount_z` | 0.2727 | Husky base_link 위 팔 장착면 높이. 0.2727 = 바닥에서 0.405 m(학습), 0.26 = 기존 URDF/MJCF(0.392 m) |
| `mujoco_box_mass` | 0.5 | 박스 질량 [kg] (학습 0.4~0.6) |
| `mujoco_box_friction` | 0.8 | 박스와 판의 마찰 (학습 박스 마찰 0.4~1.2) |
| `mujoco_box_condim` | 4 | 박스 접촉 차원 (3 미끄럼, 4 +비틀림, 6 +구름) |
| `mujoco_contact_timeconst` | 0.02 | 박스와 판의 solref time constant [s]. MuJoCo 기본 0.02, 작을수록 단단함 |
| `mujoco_cone` | elliptic | 마찰 cone. `pyramidal`은 MuJoCo 기본(잡기에서 creep) |
| `mujoco_impratio` | 10 | 마찰/법선 impedance 비. MuJoCo 기본은 1 |
| `mujoco_vision_sim` | false | true면 MuJoCo 정답 pose는 `/box_pose_gt`로 보내고, `box_pose_vision_sim` 노드가 노이즈·지연·끊김을 섞어 `box_pose_topic`으로 낸다(비전 흉내) |
| `mujoco_vision_delay_s` | 0.03 | 비전 흉내: 촬영→수신 지연 [s] |
| `mujoco_vision_bias_m` | `[0.0, 0.0, 0.0]` | 비전 흉내: base 기준 위치 오프셋 [m]. 실로봇처럼 박스를 3 cm 높게 보려면 `[0.0,0.0,0.03]` |
| `mujoco_vision_drop_probability` | 0.0 | 비전 흉내: 프레임을 안 보낼 확률 |

- 비전 흉내 노이즈(노드 파라미터 기본값)는 실로봇 `/box_tf` 실측(박스 정지, 4 s)이다: 위치 std x 3.4, y 0.7, z 1.9 mm, 회전 std 0.3°. stamp는 정답 pose의 stamp(= 촬영 시각)를 그대로 둔다.

- 손목 D435i는 boxlift launch에서 항상 뺀다(`with_realsense:=false`). 학습과 SysID 모델에 없고, link8 끝 관성이 달라지기 때문이다. Azure Kinect(Husky 고정)는 남긴다.

## 2. 터미널 2: 정책 시작과 정지

```bash
source ~/jw_ws/install/setup.bash
export ROS_DOMAIN_ID=87
# 확인: 박스가 학습 위치에 있는지 (z -0.265 근처)
ros2 topic echo --once /box_pose
# 시작 -> 응답 message에 "pre-grasp table row 1965"
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

## 4. Wide 결과 (2026-11-04, husky-rcu)

후보 A~E 다섯 줄, 마찰 0.8 / 1.2, 질량 0.5 kg, condim 4, 조건마다 1회, 4.5 s. elliptic은 A, C, D 세 줄. goal z +0.15.

| 정책, 접촉 모델 | 1.4 s z | 4 s z | 최대 기울기 | 4 s 기울기 | 떨어뜨림 |
|---|---|---|---|---|---|
| RollAlign 2999, pyramidal | −0.11 ~ −0.07 | 대부분 −0.1 이하 | 54~98° | 0~90° | 4/10 |
| RollAlign 2999, elliptic | −0.08 ~ −0.07 | −0.11 ~ +0.05 | 52~57° | 18~36° | 0/6 |
| FaceGrip 3700, pyramidal | +0.04 ~ +0.13 | 놓친 6회는 −0.265 | 29~97° | 18~23° (안 놓친 4회) | 6/10 |
| **FaceGrip 3700, elliptic** | **+0.08 ~ +0.12** | **+0.11 ~ +0.13** | 32~41° | 12~19° | **0/6** |

- pyramidal에서 놓치는 것은 MuJoCo 기본 마찰 모델의 creep 탓이 크다. 마찰 숫자(0.8, 1.2)는 이미 Isaac 실제 접촉(박스와 판의 평균, 0.45~0.85)보다 크다.
- RollAlign은 접촉 모델과 상관없이 0.4 s에 박스가 조이는 축(y)으로 약 55° 돌아 못 든다.
- FaceGrip도 들면서 판이 박스 면을 타고 올라간다. 판 중심(TCP)은 박스 중심 대비 시작 +5 cm에서 2.0 s에 +17 cm가 된다(Isaac +10.5 cm). 이때 판 위쪽이 박스 윗면(+14 cm) 위로 나와, 판마다 접촉점 1~3개로 위쪽 모서리를 잡고 박스가 21~26° 기운 채 매달린다.
- 들고 있는 상태(2.0 s)에서 박스 y축 회전이 시작되는 토크(목표 관절각 고정, 0.5 Nm/s로 증가, 2° 판정)
  - FaceGrip pyramidal 마찰 1.2: 0.27 Nm
  - FaceGrip elliptic: 0.26~0.56 Nm
  - 판 하나가 미는 힘은 14 N이다.
  - 측정 스크립트(오프라인 MuJoCo로 세션을 재현한 뒤 토크를 건다)는 아직 repo에 넣지 않았다.
- 배포 경로: 5개 줄 모두 시작 검사 통과, fault 없음.

### 비전 흉내 (FaceGrip 3700, elliptic, 2026-11-04)

후보 B(2790), A(1965), C(2166) × 마찰 0.8/1.2, 조건마다 1회. 값은 정답 pose(`/box_pose_gt`) 기준이다.

| 박스 pose 입력 | 1.4 s z | 4 s z | 최대 기울기 | 4 s 기울기 | 떨어뜨림 |
|---|---|---|---|---|---|
| 정답 (B만, A·C는 위 표) | +0.09 | +0.12 | 34~37° | 13~14° | 0/2 |
| 노이즈 + 지연 30 ms | +0.04 ~ +0.12 | +0.10 ~ +0.13 | 34~44° | 9~23° | 0/6 |
| 노이즈 + 지연 + z +3 cm | +0.06 ~ +0.12 | +0.10 ~ +0.11 | 32~56° | 9~32° | 0/6 |

- 실측 수준의 노이즈와 지연으로는 놓치지 않는다. B는 들기가 느려진다(1.4 s z +0.09 → +0.05).
- z +3 cm로 보면 정책이 박스를 3 cm 덜 든다고 믿고 덜 든다: 4 s z가 1~1.5 cm 낮다. C에서는 기울기가 커진다(최대 56°, 끝 32°). 실로봇 명령 모드 전에 비전 z 오프셋을 맞추는 게 낫다.

## 이전 정책 결과 (box410 model_2999)

Isaac 기준값(box410 model_2999, 후보 다섯 줄 sim 확인): 성공 100%, 성공까지 1.4 s, 끝 기울기 4.7°, 박스 yaw 0.4°, 최대 판 힘 16 N.

### 첫 결과 (2026-11-03, husky-rcu, 후보 A, 마찰 0.8, 질량 0.5 kg, 1회)

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
  - `gravity_z:=-9.81`, `mobile_mount_z`, `with_box:=true`, `box_x/box_y/box_yaw`(표 줄), `box_size_x/y/z`(`box_size_m`), `box_mass`, `box_friction`, `plate_friction`(= 박스 마찰. MuJoCo는 두 geom 중 큰 마찰을 쓴다), `box_condim`(기본 4), `contact_timeconst`(박스와 판 solref, 기본 0.02), `cone`, `impratio`.
  - `goal_z`(= `goal_z_m`): `base` 아래에 충돌 없는 초록 반투명 박스로 goal을 그린다.
  - `reach_markers:=false`: Reach task의 목표 구(흰색, 파란색, 빨간색)를 뺀다.
- 팔 초기 자세: `fr3_action_controller.launch.py`의 `mujoco_initial_joint_positions`가 `robot_description`의 ros2_control `initial_value`를 바꾼다(MuJoCo일 때만).
- 박스 pose: `mujoco_object_pose_body:=box`이면 `mujoco_ros_hardware`가 박스 body pose를 `base` 기준 `PoseStamped`로 `/box_pose`에 30 Hz로 publish한다. stamp는 publish 시각이다.
