# Box-lift 배포 관측/행동 일치 테스트 (2026-11-03)

실로봇 배포 노드를 작성하기 전에, 넘겨받은 배포 사양(56D 관측, 14D 행동, 목표 관절각 처리)을
numpy로 구현했을 때 Isaac 학습 정책의 입출력과 정확히 같은지 확인했다. 노드 코드는 아직 없다.
여기서 쓴 관측 조립식(`build_obs`)과 행동 처리식(`process_action`)이 노드가 구현해야 할 기준이다.

> **박스 변경 (2026-11-03 결정)**
> - 실로봇 배포와 앞으로의 학습은 모두 **새 박스 410 × 310 × 280 mm, 0.4~0.6 kg**으로 한다.
> - 이 테스트의 정책(`model_7498`)은 기존 박스(555 × 475 × 275 mm)로 학습했으므로 **배포 대상이 아니다**.
> - 관측/행동 사양과 검증 절차는 박스와 무관하므로 그대로 유효하다.
> - 새 박스로 학습한 체크포인트가 나오면 아래 "재현" 절차로 다시 검증한다.
> - 박스별 조건은 아래 "박스 조건" 절에 정리했다.

## 대상

| 항목 | 값 |
|---|---|
| Task | `Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Margin-Play-v0` (관측 노이즈 off) |
| Checkpoint | `dual_fr3_lab/logs/rsl_rl/dual_fr3_plate_lift_sim2real_100hz_light/2026-11-03_00-14-53_margin005_from5999/model_7498.pt` |
| Checkpoint sha256 | `309c9d1799d9ecc8a7629b788f1da65f6bdc0ce63078f143395ab951dd6dd5b8` |
| Actor NPZ | `actor.npz` (`export_ppo_actor.py --output-activation tanh`, 56→256→128→64→14, ELU) |
| Trace | `trace.npz`: 16 env × 200 policy step (에피소드 2개씩), 물리 0.01 s × decimation 5 = 20 Hz |
| Isaac Lab | 3.0.0b2, rsl-rl-lib 5.0.1 |

## 결과: 전 항목 통과

Isaac 관측과 행동을 배포 쪽 numpy 구현과 비교했다. 오차는 모두 float32 반올림 수준이다(`summary.json`).

| 검사 | max abs err |
|---|---|
| 팔 관절 순서 = 왼팔 j1~7, 오른팔 j1~7 | 일치 |
| 기본 자세 (넘겨받은 값) | 5e-8 |
| 목표 관절각 한계 (FCI −0.05 rad) | 1e-7 |
| 행동: scale 0.1, clip ±0.1, `policy_step` | 일치 |
| 관측 항목 순서/크기: joint_pos 14, joint_vel 14, object_position 3, target 7, actions 14, object_orientation 4 | 일치 |
| 관측 재구성: 관절 각도/속도 (이름으로 선택) | 1e-7 / 0 |
| 관측 재구성: 박스 위치 (base 기준) | 2e-7 |
| 관측 재구성: 목표 (시작 박스 x,y + z + `(0,0,0,1)`) | 0 |
| 관측 재구성: 직전 행동 (에피소드 시작 0) | 0 |
| 관측 재구성: 박스 쿼터니언 (base 기준, xyzw, w≥0) | 3e-7 |
| NPZ actor vs Isaac 행동 (Isaac 관측 입력) | 1.2e-6 |
| NPZ actor vs Isaac 행동 (재구성 관측 입력) | 2.1e-6 |
| 목표 관절각 = clip(q + clip(0.1a, ±0.1), 한계) vs Isaac이 유지한 목표 | 1.2e-7 |

넘겨받은 사양은 그대로 맞다. 노드는 아래 식만 구현하면 된다.

```
obs = [q − q_default (14), qd (14), p_box^base (3), (x0, y0, z_goal, 0, 0, 0, 1) (7), a_prev (14), quat_box^base xyzw, w≥0 (4)]
a   = tanh(MLP(obs))
q_target = clip(q(t_k) + clip(0.1·a, ±0.1), 한계)   # 50 ms 동안 유지 (policy_step)
```

## 확인 중 발견한 것

1. **학습 env의 목표가 에피소드 마지막 step에서 다시 샘플링된다.**
   - command `resampling_time_range = (5.0, 5.0)`인데 에피소드 길이도 5.0 s다. 그래서 매 에피소드 마지막 step(episode step 99)에서 목표가 바뀐다.
   - 바뀐 목표의 x,y는 그 순간의 박스 x,y이고, z는 새로 샘플링된다. 3200개 중 16개가 해당한다.
   - 에피소드가 끝나는 step이라 학습 영향은 작다. 배포에서는 사양대로 목표를 고정하면 된다.
   - 이 16개는 목표/actor 검사에서 빼고 따로 집계했다.
   - 학습 쪽에서 깔끔하게 하려면 `resampling_time_range`를 에피소드 길이보다 길게 두면 된다.
2. **`dual_fr3_lab/docs/DUAL_FR3_LIFT_PLATE_SIM2REAL.md:117`의 목표 쿼터니언 `(1,0,0,0)`(wxyz)은 틀렸다.**
   - 실제 학습 관측은 `(0,0,0,1)`(xyzw, Isaac Lab 3.0)이다.
3. **박스 시작 상태:**
   - 위치: x 0.45~0.60, y ±0.1
   - yaw: **±0.1 rad (±5.6°)**
   - 스폰 직후 박스 중심 z(base 기준)는 −0.245다. 바닥에서 2.25 cm 위에서 떨어뜨리기 때문이다.
   - 바닥에 안착하면 z는 −0.2675다. base 높이가 0.405 m라서다.

## 박스 조건

박스 좌표축 규약: 원점은 박스 중심, x는 앞뒤, y는 판이 잡는 방향, z는 위쪽이다(학습 env 정의).

| 항목 | 기존 박스 (이 테스트의 정책) | **새 박스 (배포, 앞으로의 학습)** |
|---|---|---|
| 크기 x × y × z | 0.555 × 0.475 × 0.275 m | **0.410 × 0.310 × 0.280 m** (축 배정 확정) |
| 판이 잡는 폭 (y) | 0.475 m | **0.310 m** (사용자 확인, 2026-11-03) |
| 질량 | 0.5~1.0 kg (명목 0.75) | **0.4~0.6 kg** |
| 크기 랜덤화 | 없음 (scale 1.0) | 학습 설정에서 정할 것 |
| 반높이 (윗면 마커 → 박스 중심) | 0.1375 m | **0.140 m** |
| 바닥 안착 시 중심 z (base 기준, sim base 높이 0.405 m) | −0.2675 m | −0.265 m |
| 같은 값, 실물 URDF base 높이 0.392 m 기준 | −0.2548 m | −0.252 m |
| 박스-판 마찰 | 박스 0.4~1.2, 판 0.5 | 학습 설정에서 정할 것 |
| 시작 위치 / yaw | x 0.45~0.60, y ±0.10, yaw ±0.1 rad | 학습 설정에서 정할 것 (잡는 폭이 줄어 바뀔 수 있음) |
| 목표 z (base 기준) | 0.15~0.35 m | 학습 설정에서 정할 것 |

**새 박스에 대해 확인할 것**
- 축 배정은 x 410 mm(앞뒤), y 310 mm(판이 잡는 방향), z 280 mm(높이)로 확정했다. 학습 설정의 `size`도 `(0.410, 0.310, 0.280)` 순서여야 한다.
- 크기는 실물을 실측한 값을 학습에 넣는다. 크기 랜덤화를 끄면 다른 크기의 박스는 재학습이 필요하다.
- 비전 노드가 쓰는 마커 → 박스 중심 오프셋을 0.1375 m에서 0.140 m로 바꿔야 한다. 마커를 윗면 중앙에 붙이는 경우다.
- 판 접촉 위치가 달라지므로 판 장착(0.132 m, 45°)과 기본 자세가 새 박스 폭에서도 맞는지 학습 쪽 결과로 확인한다.

## 민감도: 배포에서 하기 쉬운 실수가 행동을 얼마나 바꾸나

정상 관측에 실수 하나씩만 넣고 actor 출력 변화를 쟀다. 행동 범위는 ±1이고, Δq = 0.1·a다.

| 실수 | 평균 \|Δa\| | 최대 \|Δa\| | 최대 Δq 변화 |
|---|---|---|---|
| 목표 쿼터니언을 wxyz `(1,0,0,0)`로 넣음 | **0.30** | 1.84 | 0.18 rad |
| 박스 yaw 180° 뒤집힌 쿼터니언 (대칭 처리 누락) | **0.42** | 1.89 | 0.19 rad |
| 박스 z +1.3 cm (실물 URDF base 높이 0.392 m vs sim 0.405 m) | 0.020 | 0.18 | 0.018 rad |
| 박스 x,y +1 cm (학습 노이즈 1σ) | 0.009 | 0.10 | 0.010 rad |

- 쿼터니언 순서와 180° 대칭 처리는 틀리면 정책이 사실상 다른 행동을 낸다. 노드와 인식 쪽 단위 테스트로 반드시 막아야 한다.
- base 높이 차이(1.3 cm)는 학습 노이즈 1σ의 약 2배 영향이다. 실측해서 박스 z를 보정하거나, 오차를 감안해야 한다.

## 참고 통계 (노드 설계용)

16 env × 200 step 기준이다.

- 행동 포화(|a| > 0.99): 1.5% / 목표 관절각이 한계에 잘리는 비율: 1.5% / 최대 관절 속도: 1.97 rad/s
- **컨트롤러 거부 위험 (policy_step):**
  - 상황: 컨트롤러는 명령을 받는 순간 `|목표 − 현재 q| ≤ max_policy_target_delta_rad(0.11)`를 검사한다. 정책 시점의 q와 컨트롤러 수신 시점 사이 지연을 L로 두고, `|Δq − qd·L| > 0.11`이 나오는 비율을 sim 궤적으로 추정했다.

    | L | 거부되는 step 비율 | 거부가 1번이라도 나는 에피소드 비율 |
    |---|---|---|
    | 10 ms | 0% | 0% |
    | 20 ms | 0.12% | 12% |
    | 30 ms | 1.3% | 47% |
    | 50 ms | 5.1% | 97% |

  - 의미: 거부되면 컨트롤러가 abort하고 halt한다. halt는 현재 위치를 붙잡기 때문에 쥐는 힘이 사라진다.
  - 할 일: joint_states 나이와 전달 지연을 실측하고, policy_step 경로의 검사 기준을 정해야 한다.
- 버티는지 여부:
  - 에피소드 끝(5 s)에 박스가 들려 있는 경우: 31/32
  - 목표 z ±3 cm 도달: 20/32. 도달까지 걸린 시간은 중앙값 약 70 step(3.5 s)이다.
  - 도달 후 끝까지 ±3 cm 안에 머문 경우: 9/32
  - 즉 5 s 안에서 계속 들고는 있지만, 목표 높이 근처에서 계속 조정한다. 5 s 이후 동작은 학습되지 않았다.

## 재현

```bash
# 1) actor export (torch 필요: Isaac env)
cd ~/etri_ws/src/fr3_husky_etri
PYTHONPATH=fr3_husky_nn_policy ~/anaconda3/envs/isaac_601_300b2/bin/python -m fr3_husky_nn_policy.export_ppo_actor \
  <checkpoint> data/boxlift_obs_parity_261103/actor.npz --output-activation tanh

# 2) Isaac trace (headless에서는 EULA 환경변수가 필요하다)
cd ~/dual_fr3_lab
OMNI_KIT_ACCEPT_EULA=YES ~/anaconda3/envs/isaac_601_300b2/bin/python scripts/tools/dump_boxlift_deploy_trace.py \
  --task Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Margin-Play-v0 \
  --checkpoint <checkpoint> --num_envs 16 --steps 200 \
  --output ~/etri_ws/src/fr3_husky_etri/data/boxlift_obs_parity_261103/trace.npz --headless

# 3) 검증 (numpy만 필요)
cd ~/etri_ws/src/fr3_husky_etri
~/anaconda3/envs/cRobotics/bin/python scripts/tools/verify_boxlift_obs_parity.py \
  --trace data/boxlift_obs_parity_261103/trace.npz --actor data/boxlift_obs_parity_261103/actor.npz \
  --output-dir data/boxlift_obs_parity_261103
```

체크포인트를 바꾸면 1)~3)을 다시 돌린다. actor sha256과 trace의 checkpoint sha256이 같아야 통과한다.

## 추가 (2026-11-03): 매니페스트 포함 export

- `actor_manifest.npz`: `actor.npz`와 가중치가 같고(비교 확인), 학습 run의 `params/env.yaml`에서 뽑은 `manifest_json`이 추가로 들어 있습니다.
  - 만든 도구: `dual_fr3_lab/scripts/tools/export_boxlift_actor.py <run>/model_XXXX.pt <out>.npz`
  - `params/`가 옆에 없으면 export를 거부합니다.
- 배포 core `fr3_husky_nn_policy/boxlift_core.py`가 위 기록 trace를 그대로 재현합니다.
  - 확인 테스트: `test/test_boxlift_core.py::test_core_reproduces_recorded_isaac_policy_io` (obs 1e-5, action 1e-4, held target 1e-5)
- `model_7498`은 기존 박스(555×475×275)로 학습한 모델이라, 새 박스 0.410×0.310×0.280으로 로드하면 거부됩니다. 의도한 동작입니다.
