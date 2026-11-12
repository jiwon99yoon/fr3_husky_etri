# 듀얼 FR3 판 박스 들기: GoalXY 5499 보고서 (2026-11-10)

실로봇 시험 기록이 남은 정책 `facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_5499.npz`(이하 5499)가 무엇이고 어떻게 학습했는지,
sim2real을 위해 무엇을 했는지, sim·실물 수치와 다른 정책과의 차이를 정리한다.
실물 시험 절차와 회차별 표는 [dual_fr3_box_experi_step.md](dual_fr3_box_experi_step.md)의 "실물 결과: GoalXY 5499"에 있다.

## 요약

- 5499 = task `Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Pregrasp-Wide-RollAlign-FaceGrip-BoxDrop-FlatSqueeze-LowMuSoftLift-GoalXY-v0`의 5499 iter.
  처음부터 학습한 FaceGrip-BoxDrop(0 → 2999) 위에 판 평행·최소 조임(FlatSqueeze), 낮은 마찰·부드러운 들기(LowMuSoftLift), 목표 x·y 이동(GoalXY)을 차례로 이어 학습했다(총 5500 iter, 약 5.4억 transition).
- 관측 56차원(관절 각·속도, 박스 위치·자세, 목표, 직전 행동), 행동 14차원(양팔 관절 목표의 상대 변화, 스텝당 ±0.1 rad), 20 Hz.
  actor는 지연·끊김이 섞인 박스 pose를, critic은 실제 박스 pose를 본다(비대칭 actor-critic).
- 접근은 MoveIt이 하고(판이 박스 면 4 cm 밖), 정책은 조이기 ~ 들기 ~ 유지 4 초만 맡는다.
- **실물 10회 중 8회 성공**(2026-11-07, 목표 x·y·z를 바꿔 가며). 성공 8회는 3D 오차 평균 4.2 cm, 기울기 6.7~7.8°로 4 s를 끝까지 들었다.
  실패 2회는 0.4 s쯤부터 `/dual_fr3/joint_states`가 끊겨 노드가 0.55 s에 멈춘 경우다(박스를 들기 전).
- sim: 목표 x·y 이동 과제 98.6%, 낮은 접촉 마찰(0.21~0.64) 99.4%, 접촉 마찰 0.2에서도 3 cm 넘게 미끄러진 에피소드 0%.
- 앞선 정책의 실물 실패는 모두 **들기 순간 판이 박스 면을 타고 미끄러지고, 이어서 박스가 잡는 축 둘레로 도는 것**이었다(FlatSqueeze 3998 12회 중 11회, SqueezeLift 6697 11-05 시험).
  5499 계열은 이 실패를 sim에서 재현하는 마찰(접촉 0.22~0.25)까지 낮춘 DR과 들기 가속·미끄럼·yaw 벌점으로 학습했다.

## 1. 작업과 시스템

| 항목 | 값 |
|---|---|
| 로봇 | FR3 두 대(Husky 위), 각 link7에 140 × 140 × 25 mm 판. 그리퍼 없음 |
| 박스 | 310(앞뒤) × 410(잡는 방향) × 280 mm 빈 상자, 실측 501~561 g |
| base frame | 두 팔 장착점의 중점, x 앞, y 왼쪽, z 위. 바닥 z −0.405 m(박스 중심 정지 높이 −0.265) |
| 시작 | MoveIt(`move_to_joint`)으로 시작 자세 표 한 줄로 이동. 판이 박스 면 4 cm 밖, 판 중심이 박스 중심보다 5 cm 위, 판 모서리 정렬 |
| 정책 구간 | 조이기 ~ 들기 ~ 유지, 학습 에피소드 4 s(80 step) |
| 목표 | 박스 중심 base x 0.40~0.50, y ±0.10, z 0.15~0.35 m(학습 때 박스 위치와 따로 뽑음). 실물은 `goal_z_m`, `goal_xy_reference`, offset 파라미터로 지정 |
| 박스 pose | Azure Kinect 비전 `/box_tf`, z −0.03 m 보정(`box_position_offset_m`) |
| 추론 | `ppo_boxlift_policy_node`(NumPy, 20 Hz) → `RunPolicyControl` → C++ 컨트롤러 1 kHz PD(Kp 80, Kd 4) 토크 |

## 2. 체크포인트 계보

모두 4096 env × 24 step/iter, 같은 관측·행동이라 앞 단계 checkpoint에서 이어 학습했다. 이어 학습은 N → N + 999(LowMuSoftLift는 502 iter에서 고름)로 끝나서 번호가 5499다.

| 단계 | task (`…-Light-Pregrasp-Wide-RollAlign` 뒤) | iter | 바꾼 것 | 이유 |
|---|---|---|---|---|
| 1 | `-FaceGrip-BoxDrop-Scratch` | 0 → 2999 (scratch) | 판-면 각도 벌점, 박스 pose 지연·끊김, critic만 실제 박스 pose | 실물 비전 지연(stamp 나이 35~90 ms)과 끊김. 판을 V자로 젖혀 모서리로 받치는 파지 금지 |
| 2 | `-FaceGrip-BoxDrop-FlatSqueeze` | 2999 → 3998 | 더 기운 판 기준 평행(−20), 판 면 앞뒤 중앙(−20), 판 법선 조임 16 N 부족분(−0.5) | 1단계 2999는 실물에서 판은 평행하게 댔지만 판-박스가 미끄러졌다(조임 13.4 N) |
| 3 | `-…-FlatSqueeze-LowMuSoftLift` | 3998 → 4500 | 접촉 마찰 0.21~0.64, 박스 크기 ±1 cm, 들기 가속 1.5 m/s² 초과(−20), 판-박스 미끄럼(−10), 박스 yaw 변화(−10) | 3998이 실물 12회 중 11회 들기 순간 미끄러진 뒤 박스가 돌았다. 같은 미끄럼이 sim 접촉 마찰 0.22~0.25에서 나왔다 |
| 4 | `-…-LowMuSoftLift-GoalXY` | 4500 → 5499 | 목표 x·y를 박스와 따로 뽑기 | 수직 들기뿐 아니라 옮기며 들기 |

- 총 5500 iter = 약 5.4억 transition. Isaac Sim 6.0.1, Isaac Lab 3.0.0b2, rsl-rl 5.0.1.
- run(`dual_fr3_lab/logs/rsl_rl/` 아래, task 디렉터리 `dual_fr3_plate_lift_sim2real_100hz_light_pregrasp_wide_rollalign_…`):
  `…facegrip_boxdrop_scratch/2026-11-04_20-08-54_facegrip_boxdrop_scratch`, `…facegrip_boxdrop_flatsqueeze/2026-11-05_00-45-16_flatsqueeze_from_fgbd2999`,
  `…facegrip_boxdrop_flatsqueeze_lowmusoftlift/2026-11-05_04-14-11_lowmusoftlift_from_fs3998`, `…facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy/2026-11-05_05-20-47_goalxy_from_sl4500/model_5499.pt`.
- 자세한 단계별 결과: `dual_fr3_lab/docs/dual_fr3_lift_sim2real.md` "LowMuSoftLift", "goal xy를 박스와 따로", "GoalXY 결과" 절.

## 3. MDP

### 관측 56차원 (비대칭 actor-critic)

| index | 항목 | 크기 | 내용 | actor (배포) | critic (학습만) |
|---|---|---|---|---|---|
| 0~13 | `joint_pos` | 14 | 관절각 − 기본 자세(ready). 순서 왼팔 j1~7, 오른팔 j1~7 | + N(0, 0.005 rad) | 정확 |
| 14~27 | `joint_vel` | 14 | 관절 속도 | + N(0, 0.05 rad/s) | 정확 |
| 28~30 | `object_position` | 3 | 박스 중심 위치(base) | + N(0, 1 cm), 지연·끊김 | 정확 |
| 31~37 | `target_object_position` | 7 | 목표 위치 + 쿼터니언 (0, 0, 0, 1) | 정확 | 정확 |
| 38~51 | `actions` | 14 | 직전 행동(정책 출력 a, 에피소드 시작 0) | 정확 | 정확 |
| 52~55 | `object_orientation` | 4 | 박스 쿼터니언(base, xyzw, w ≥ 0). 180° 대칭은 배포 쪽에서 맞춤 | + N(0, 0.02), 지연·끊김 | 정확 |

- 박스 pose 지연·끊김(actor만): 에피소드마다 0~3 step(0~150 ms) 지연을 뽑아 고정하고, 끊기지 않은 step마다 2% 확률로 0.1~1.0 s 끊긴다(그동안 마지막 값 유지). 에피소드의 10%는 0.05~2 s 사이 임의 시점부터 끝까지 끊긴다.
- 판 접촉력, 마찰, 질량 같은 특권 정보는 actor·critic 모두 보지 않는다(critic도 마찰·질량을 몰라 value loss가 크다, 7절).

### 행동 14차원

- 정책 출력 a ∈ (−1, 1)(tanh). 관절마다 Δq = clip(0.1 · a, ±0.1 rad).
- q_target = q_측정 + Δq(`policy_step`, 매 step 측정값 기준), FCI 관절 한계 − 0.05 rad로 자름. 다음 step(50 ms)까지 유지.
- 팔 PD: Kp 80, Kd 4, 토크 한계 87 Nm(j1~4) / 12 Nm(j5~7), 토크 변화율 1000 Nm/s. 실물 C++ 컨트롤러와 같은 값.
- 결과적으로 관절당 PD 토크 상한은 80 × 0.1 = 8 Nm이다. 이게 조임 힘의 상한을 정한다.

### 보상 (5499 학습 때 최종 weight)

들기와 목표 추종 보상에 게이트를 곱하는 구조다: `r = w_reach·r_reach + g·(w_lift·r_lift + w_goal·r_goal) + w_succ·r_succ + Σ w_k·r_k`.
게이트 g = G · q · f_level · f_plate(G: 두 판 모두 1 N 넘게 누름, q: 접촉력·판 자세가 면 법선과 맞는 정도(더 나쁜 판), f_level: 박스가 45° 기울면 0, f_plate: 판 외 접촉력 10 N이면 0).

| 묶음 | 항 (weight) |
|---|---|
| 과제 | 접근 `reaching_object` (6 → 9, 팔별 판-면 거리 tanh(σ 0.3) × 평행도의 min), 들기 `lifting_object` (15, 박스 10 cm 이상), `lift_progress` (8), 목표 추종 (16, σ 0.3) + 정밀 (5, σ 0.05), 성공 (20, 0.5 s 유지 후 매 step, yaw 오차로 최대 절반 감액), 조임 품질 `squeezing_object` (2) |
| 판 자세·조임 | `plate_face_tilt` 더 기운 판의 판-면 각도 (−20), `plate_face_offcenter` 판 중심의 면 앞뒤 중앙 오차 (−20), `min_squeeze` 판 법선 힘 16 N 부족분 (−0.5), `excessive_squeeze` 100 N 초과분 (−0.05), `plates_misalignment` (−4) |
| 부드러운 들기 | `box_lift_accel` 양 판 접촉 중 박스 위 가속 1.5 m/s² 초과분 (−20), `plate_box_slip` 판 중심점의 박스 대비 접선 속도 + 0.1 m × 상대 각속도 (−10), `box_yaw_drift` 에피소드 시작 대비 박스 yaw 변화 (−10) |
| 박스·대칭 | `box_upright` (−2 → −3), `plate_height_symmetry` (−8), `plate_approach_symmetry` (−20), `one_sided_contact` (0 → −5) |
| 안전·부드러움 | 판 외 접촉 부위 수 (−1), FR3 관절 한계 여유 (−1), `action_rate` (−0.1), `joint_vel` (−0.1) |

- `→` 값과 −1e-4에서 시작하는 항은 커리큘럼이 들기를 배운 뒤 최종 값으로 바꾼다. 부드러운 들기 세 항은 success EMA 3 뒤 2400 env-step(100 iter)에 걸쳐 올린다. 외란(×1 → ×3)·q의 판 자세 factor·yaw 항도 성공률에 따라 켜진다.
  이어 학습하면 커리큘럼이 처음부터 다시 돈다. 5499 run 끝에서는 모든 항이 최종 값이었다(TensorBoard `Curriculum/*`).
- 학습 마지막 50 iter 평균 초당 기여(`Episode_Reward/*`): 들기 11.8, 성공 9.3, 목표 추종 9.2 + 1.0, 접근 7.1, 들기 진행도 6.6, 조임 품질 1.7 / 판 기울기 −2.3, 미끄럼 −1.8, 최소 조임 −1.2, 앞뒤 중앙 −0.9, 들기 가속 −0.8, yaw −0.5, 나머지 10항 합 −1.2.
  과제 +46.8 /s, 벌점 −8.7 /s, 에피소드 return 약 152.
- 종료: 시간(4 s), 박스가 작업 영역 밖.
- 성공 판정(학습·sim 평가): 박스-목표 거리 < 10 cm, 박스 기울기 < 15°, 양 판 접촉이 0.5 s(10 step) 연속. 실물은 판 접촉을 잴 수 없어 끝 상태의 거리·기울기만 본다.

### PPO

actor·critic MLP 256-128-64(ELU, 따로), tanh-squashed Gaussian(평균 bound 5), lr 1e-4 adaptive(KL 0.01), γ 0.98, λ 0.95,
entropy 0.006, clip 0.2, value loss clip, 5 epoch × 4 minibatch, 24 step/env, 4096 env.

## 4. sim2real을 위해 한 것

### 4.1 5499 학습에 들어간 것

| 항목 | sim 설정 | 실물 / 근거 |
|---|---|---|
| 제어 계약 | 20 Hz, 측정 관절 기준 상대 목표, ±0.1 rad, 50 ms 유지 | 배포 노드·컨트롤러와 같은 식, 관측·행동 parity 테스트 통과(`data/boxlift_obs_parity_261103/report.md`) |
| 팔 액추에이터 | explicit PD 80/4, 토크 한계 87/12 Nm, 토크 변화율 1000 Nm/s | 실물 C++ 컨트롤러 값 |
| 관절 수동 동역학 | armature 0.1, viscous 0.003, 건마찰 0.2 Nm | 실물·MuJoCo·Isaac 3자 sysID(2026-09)로 맞춘 값 |
| PD 게인 DR | Kp, Kd × 0.85~1.15 | 실물 추종 오차 대비 |
| 관절 한계 | sim 한계와 목표 clamp를 FCI 한계 − 0.05 rad로, 한계 근처 벌점 | 실물 reflex 정지 방지 |
| 박스 | 실측 크기 ±1 cm, 질량 0.4~0.6 kg | 빈 상자 501~561 g |
| 접촉 마찰 | 박스 0.1~0.8, 판 0.3~0.5 → 접촉(평균) 0.21~0.64 | 실물 미끄럼이 sim 접촉 마찰 0.22~0.25 상당(3998 bag 분석) |
| 시작 분포 | 실물과 같은 시작 자세 표(3956줄, 박스 x 0.40~0.50, y ±0.11, yaw ±6.6°) + 박스 ±1 cm·±0.05 rad, 관절 ±0.01 rad 노이즈 | 실물도 같은 표 줄로 MoveIt 이동 |
| 관측 노이즈·비전 | 위 관측 표, 박스 pose 지연 0~150 ms·끊김 | 비전·엔코더 잡음, 실물 비전 stamp 나이 35~90 ms |
| 비대칭 critic | critic만 노이즈·지연 없는 박스 pose | |
| 외란 | 박스에 질량 비례 힘·토크(±0.625 N, ±0.125 Nm, 0.4~0.8 s마다, 성공률에 따라 ×1 → ×3) | 접촉 변동 대비 |
| 접촉 품질 보상 | 판 면 파지(V자 금지), 면 앞뒤 중앙, 최소 조임 16 N, 판 외 접촉 벌점 | sim에서만 통하는 모서리 받치기·약한 조임 차단 |
| 부드러운 들기 | 들기 가속 상한, 판-박스 미끄럼, 박스 yaw 변화 벌점 | 실물 실패(들기 순간 미끄럼 → 회전) |
| 좌표계 | Isaac root = 실물 base(URDF로 대조) | 관측 위치가 같은 frame |
| 배포 안전장치 | shadow 모드, 시작 조건 검사(표 줄·박스 위치), manifest 검사(박스 크기·관측 순서·관절 순서·goal 범위), 관절·박스·명령 timeout | |

### 4.2 5499에 없는 것

| 항목 | 영향 |
|---|---|
| 관절 상태·명령 지연 DR | 실측 관절 상태 나이 < 1 ms라 영향 작다. 다만 실물 실패 2회는 `/joint_states`가 끊겨 노드가 멈춘 경우다 |
| critic의 마찰·질량·크기 입력 | critic이 미끄러질지 예측하지 못해 value loss가 크다(40~100, FlatSqueeze 0.6) |
| 판 잡는 높이 벌점 | 오래 학습하면 판을 박스 중심 아래로 내려 잡는다(5499 −3.1 cm) |

### 4.3 sim에서 맞추기 어렵거나 못 하는 것

| 항목 | 이유 | 대응 |
|---|---|---|
| 실물 접촉 마찰·법선력 | 판-상자 마찰을 측정하지 않았다. bag 분석상 마찰 × 법선력이 원래 학습 범위(0.45~0.85)의 절반쯤 | 낮은 마찰 DR(5499에 들어감), 판 고무 패드(하드웨어) |
| 상자 변형 | 빈 상자 면이 조임에 휠 수 있다. 강체 sim으로 표현 불가 | 조임 여유, 천천히 들기 |
| 접촉 모델 차이 | 같은 조임에서 박스 회전 시작 토크가 Isaac이 MuJoCo의 2~3배(0.80~0.93 vs 0.26~0.56 Nm) | MuJoCo sim2sim으로 확인 |
| 물리 주기 | sim PD 100 Hz(dt 0.01), 실물 1 kHz | explicit PD + 토크 변화율 제한으로 근사 |
| 비전 오차 구조 | 실물 박스 z가 2.8 cm 높게 보임 | 노드 오프셋 보정 |
| 놓친 뒤 다시 잡기 | 바닥에서 다시 잡는 상태가 시작 분포와 너무 달라 RL이 못 배움 | 노드에서 낙하 감지 → MoveIt 복귀 → 재시작이 현실적 |
| 정지 시 hold | 정책을 멈추면 컨트롤러가 측정 관절을 유지해 조임이 풀린다 | 미구현(박스 받치고 시험) |

## 5. 결과

### 5.1 sim (Isaac)

평가: 1024 env, 학습 노이즈·질량·마찰 DR 그대로, 외란 1.0, 80 step(4 s). GoalXY 열은 이 task Play, 나머지는 수직 goal(LowMuSoftLift Play, 기본 마찰 Wide-RollAlign Play).
미끄럼은 `sim_slip_probe.py`(512 env, 박스 0.1 + 판 0.4 / 0.3 → 접촉 마찰 0.25 / 0.2).

| ckpt | GoalXY 성공 / 끝 goal | 수직 낮은 마찰 / 기본 마찰 | 들 때 기울기 median / p90 | 판 중심 − 박스 중심 / 판 높이 차 median, p90 | 관절 여유 | μ 0.25 미끄럼 median / p90, 기울기 > 20° | μ 0.2 미끄럼 median / p90, > 3 cm, 기울기 > 20° |
|---|---|---|---|---|---|---|---|
| FlatSqueeze 3998 | 84.4% / 2.6 cm | 92.3 / 100% | 4.5 / 12.0° | +6.1 / 1.1, 2.6 cm | 0.127 | 4.1 / 13.1 cm, 21% | 14.8 / 18.4 cm, 95% |
| LowMuSoftLift 4500 | 95.5 / 5.0 | 98.9 / 100 | 6.3 / 10.8 | +1.5 / 2.5, 4.7 | 0.096 | 1.4 / 4.0, 7% | 5.8 / 11.3, 74%, 12% |
| GoalXY 4700 | 97.9 / 4.2 | 97.8 / 100 | 6.6 / 9.8 | −1.1 / 1.3, 3.3 | 0.092 | 0.5 / 2.5, 4% | 3.4 / 6.8, 57%, 16% |
| **GoalXY 5499** | **98.6 / 3.8** | **99.4 / 100** | 9.1 / 12.3 | −3.1 / 1.7, 3.4 | 0.050 | −1.5 / −0.7, 2% | −0.8 / 0.6, **0%**, 11% |

- 학습할수록 천천히 들고(들기 초반 판 상승 16 → 3 cm, 최대 가속 5 → 2.5 m/s²), 낮은 마찰에서 미끄럼이 거의 없어진다.
- 대신 잡는 높이가 계속 내려간다(판 중심이 박스 중심 위 +6.1 → −3.1 cm). 5499는 판을 박스 중심 약간 아래에서 잡고, 관절 여유가 sim 한계(0.05 rad)까지 줄었다.

### 5.2 sim2real (실물, 2026-11-07)

- 명령 모드, 시작은 pregrasp 표, 에피소드 4 s. 노드에는 자동 성공 판정이 없어, 로거가 남긴 끝 상태(비전 박스 pose)에 학습 성공 기준(3D 거리 < 10 cm, 기울기 < 15°)을 적용했다.
- 집계는 마지막 10회(19:37~19:47)다. 앞의 3회(19:24~19:32)는 목표 z 0.15에서 맞춘 회차다(5.2 cm 성공, 12.3 cm 작업자 정지, 5.6 cm 성공).
- 데이터: `experi_result/dual_fr3_boxlift_real/etri_box_lift_80%_results.csv`, 세션 로그 `data/dual_fr3_boxlift_real/20261107_19*/`, 영상 `experi_result/video/etri_box_lift_80%.webm`.

| # | 세션 | 목표 (x, y, z) | 3D 오차 (cm) | 기울기 (°) | 결과 |
|---|---|---|---|---|---|
| 1 | 193733 | (0.45, 0, 0.2) | 4.6 | 7.8 | 성공 |
| 2 | 193839 | (0.48, 0, 0.2) | 6.3 | 7.8 | 성공 |
| 3 | 193950 | (0.42, 0, 0.2) | 2.7 | 7.8 | 성공 |
| 4 | 194058 | (0.45, +0.03, 0.2) | 3.9 | 7.1 | 성공 |
| 5 | 194207 | (0.45, −0.03, 0.2) | 5.3 | 7.7 | 성공 |
| 6 | 194309 | (0.45, 0, 0.3) | 3.8 | 7.0 | 성공 |
| 7 | 194405 | (0.48, 0, 0.3) | 54.5 | – | 실패: 0.55 s `fault: joint state is stale` |
| 8 | 194516 | (0.42, 0, 0.3) | 54.3 | – | 실패: 0.55 s `fault: joint state is stale` |
| 9 | 194626 | (0.45, +0.03, 0.3) | 3.0 | 6.7 | 성공 |
| 10 | 194728 | (0.45, −0.03, 0.3) | 3.9 | 7.8 | 성공 |

- 성공 8회: 3D 오차 평균 4.2 cm(2.7~6.3), 기울기 6.7~7.8°, 박스 중심이 시작 z −0.24에서 목표 높이 근처(z 0.21~0.28)까지 올라가 4 s를 끝까지 들었다.
- 실패 2회: 0.4 s쯤부터 `/dual_fr3/joint_states` 나이가 12 → 62 → 112 ms로 늘어 노드가 0.55 s에 멈췄다. 그때까지 정책 명령(스텝당 최대 0.09~0.10 rad, 관절 추종 오차 0.06~0.10 rad)은 성공한 회차와 거의 같았다.
  들다가 놓친 것이 아니라 로봇 쪽이 멈춘 것으로 보인다(Franka reflex나 통신일 수 있으나 로그로는 원인을 모름).

### 5.3 다른 정책의 실물 결과

| 정책 | 실물 결과 | 출처 |
|---|---|---|
| **GoalXY 5499** | **10회 중 8회 성공** (실패 2회는 joint state 끊김) | 2026-11-07, 위 표 |
| FlatSqueeze 3998 | 12회 중 11회 들기 순간 미끄럼(3.1~7.8 cm) + 박스 회전(최대 40~70°) | 2026-11-05, `data/dual_fr3_boxlift_real`, bag 12개 |
| SqueezeLift 6697 (다른 계보) | 판이 미끄러지며 박스가 돎. 결과 판정 기록 없음 | 2026-11-05, 세션 14개(trace만), bag 2개(미끄럼 4.5~4.7 cm, 최대 기울기 24°·85°) |
| FaceGrip-BoxDrop 2999 | 판은 평행하게 대지만 판-박스가 미끄러짐 | 2026-11-05 관찰 |
| LowMuSoftLift 4500 / 4997 | 4500 1회, 4997 5회 돌림. 결과 판정 기록 없음 | 2026-11-05, `data/dual_fr3_boxlift_real` |

- 실물 bag 분석(3998 12회, 6697 2회):
  - 조임은 sim과 같다. 들고 있을 때 j1 토크 L −5.7~−7.9, R +4.3~+6.1 Nm(sim 6.9~7.3), j1·j3 PD 오차 0.08~0.099 rad(sim 0.086~0.093).
  - 실패 순서: 0.25 → 0.6 s 동안 판이 박스보다 3~8 cm 더 올라감(sim 0.1 cm) → 판이 박스 중심 위 7~13 cm까지 올라감 → 박스가 잡는 축(y) 둘레로 20~70° 회전.
  - sim에서 같은 미끄럼은 접촉 마찰 0.22~0.25에서 나온다. 즉 실물 마찰 × 조임 힘이 원래 학습 범위의 절반쯤이다. 이것이 3단계 LowMuSoftLift의 근거다.
- 실측 지연(11-05 세션 median / p95): 정책 주기 50.0 / 50.2 ms, 관절 상태 나이 0.2~0.9 / ≤ 1.2 ms, 추론 0.2 ms, 박스 pose stamp 나이 35~91 / 40~103 ms(두 PC 시계 차이가 섞였을 수 있음).

## 6. 분석: 5499가 앞 정책과 다른 점

| | FlatSqueeze 3998 | SqueezeLift 6697 | **GoalXY 5499** |
|---|---|---|---|
| 판 중심 − 박스 중심 (sim, 들고 있을 때) | +5.8~6.1 cm | +10.8 cm | **−3.1 cm** |
| 학습 접촉 마찰 | 0.45~0.85 | 0.45~0.85 | **0.21~0.64** |
| 학습 마찰에서 들 때 미끄럼 (0.25→0.6 s) | 0.1 cm | 2.3 cm | 5499는 안 잼 (4500 0.0, 4700 −0.2 cm) |
| 접촉 마찰 0.25 미끄럼 median / 기울기 > 20° | 4.4 cm / 25% | 8.8 cm / 50% | −1.5 cm / 2% |
| 박스 pose 지연·끊김 학습 | 있음 | 없음 | 있음 |
| 실물 | 12회 중 1회 | 미끄럼 + 회전 | 10회 중 8회 |

- **잡는 높이.** 3998·6697은 판이 박스 중심보다 위를 잡아, 판이 조금만 미끄러져도 박스 무게가 잡는 축 둘레로 박스를 돌린다. 5499는 박스 중심 높이(약간 아래)에서 잡아 회전시키는 토크가 작다.
- **낮은 마찰에서 학습.** 실물 상당 마찰(0.22~0.25)이 학습 범위 안이고, 들기 가속·미끄럼 벌점으로 천천히 든다. sim에서 마찰 0.2에도 3 cm 넘는 미끄럼이 없다.
- **sim 지표만으로는 실물 순위를 못 고른다.** 3998은 기본 마찰 sim 100%였지만 실물 1/12였다. 실물 실패 방식(미끄럼 → 회전)을 sim에서 재현하는 조건(낮은 마찰)으로 평가한 뒤에야 순위가 실물과 맞았다.
- 주의: 실물 시험은 10회이고, 성공은 끝 상태의 거리·기울기로만 판정했다(판 접촉·유지 시간은 못 잼).

## 7. 5499의 한계와 다음 단계

- 판을 박스 중심보다 3 cm 낮게 잡는다. 박스가 판-판 축 위에 선 역진자에 가까워, 들 때 기울기가 p90 12.3°로 4500(10.8°)보다 크다.
- 관절 여유가 sim 한계(0.05 rad)에 닿는다. 실물에서 관절 한계 reflex 위험이 있다. 중간 checkpoint 4700(여유 0.092 rad, 판 −1.1 cm)이 대안이다.
- critic이 마찰·질량을 몰라 value loss가 크다 → critic에 μ, 질량, 크기를 넣는다.
- 정책을 멈추면 조임이 풀린다(hold 모드 없음). 실물 실패 2회의 `/joint_states` 끊김 원인을 확인해야 한다.
- 다음: 판 잡는 높이 벌점, critic 특권 입력, 판-박스 마찰 실측, MuJoCo sim2sim으로 후보 순위 확인.

## 부록: 파일

| 무엇 | 경로 |
|---|---|
| 배포 NPZ | `fr3_husky_nn_policy/models/facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy_model_5499.npz` (sha256 manifest 안) |
| 학습 run | `dual_fr3_lab/logs/rsl_rl/dual_fr3_plate_lift_sim2real_100hz_light_pregrasp_wide_rollalign_facegrip_boxdrop_flatsqueeze_lowmusoftlift_goalxy/2026-11-05_05-20-47_goalxy_from_sl4500` (`params/env.yaml`, `agent.yaml`) |
| 학습 기록 | `dual_fr3_lab/docs/dual_fr3_lift_sim2real.md` |
| 실물 데이터 | `data/dual_fr3_boxlift_real/`, `data/dual_fr3_boxlift_real_bags/`, `experi_result/dual_fr3_boxlift_real/` |
| 실험 절차 | `docs/dual_fr3_box_experi_step.md` |
