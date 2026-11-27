# FR3 양팔 SysID 실험 (2026-11-26): ready·중간·hover·pregrasp에서 마찰과 응답

## 한눈에 보기

**무엇을 하나:** 토크 모드(PD Kp 80, Kd 4, 정책과 같은 경로)에서 양팔 관절을 하나씩 움직인다. 양팔 `robot_state`를 1 kHz bag으로 남긴다.

| 기호 | 시험 | 도구 | 나오는 것 |
|---|---|---|---|
| S | 등속 왕복 + 천천히 출발 + 사인 (9/29와 같은 설정) | `fr3_sysid_node` | c, F_d, c_v, 방향별 F_s, A |
| P | 계단 + 사인 + chirp (11-17과 같은 설정) | `fr3_command_response_probe` | 응답·지연, 빠른 움직임 검증용 데이터 |
| C | 짧은 등속 왕복 (j1, j2, j4) | `fr3_sysid_node` | 그 자세의 c |
| T | hover에서 j2, j4를 진폭만 줄여 등속 왕복 | `fr3_sysid_node` | 같은 속도에서 왕복 길이에 따라 마찰이 바뀌는지 |

**자세:** R0(9/29 ready) → M(중간) → H(hover) → G(pregrasp) → 마지막에 다시 R0.

**시간:** 약 2시간. **로봇 쪽 코드 수정은 없다.**

**결과물** (경로의 `261126`은 실험 날짜. 다른 날 하면 8절 명령의 날짜를 모두 바꾼다)

| 종류 | 위치 |
|---|---|
| bag (1 kHz) | `~/jw_ws/bags/sysid_261126/<label>_<시각>/` |
| sysid CSV | `~/jw_ws/src/fr3_husky/data/sysid_dual_261126/<label>/real/` |
| probe CSV | `~/jw_ws/src/fr3_husky/data/command_response/real/<시각>_..._<label>/` |

- 11-25 계획서([fr3_joint_friction_261125_experi_step.md](fr3_joint_friction_261125_experi_step.md))를 대신한다. 11-26에 로봇 없이 분석한 결과(1-1)를 보고 아래처럼 바꿨다.
  - **press 대신 sysid 노드의 등속 왕복을 쓴다.**
    - + 방향과 − 방향 토크의 평균이 c이고, 차이의 절반이 마찰이다. 그래서 c와 마찰이 바로 갈린다.
    - 반환점에서 명령 토크가 10 ms마다 0.01~0.05 Nm씩 올라간다. press 계단(0.12 Nm)보다 출발 문턱이 촘촘히 나온다.
  - **ready는 9/29 자세(j1 ±0.52)로 한다.** 9/29 양팔·판 데이터와 바로 비교할 수 있다.
  - **중간 자세(M)와 pregrasp(G)는 짧은 c 시험만, hover에서는 진폭 시험(T)을 더 한다.**
- 개념(제어기와 plant 나누기, 토크 정의)은 [control.md](control.md)와 dual_fr3_lab `docs/dual_fr3_joint_dynamics.md`에 있다.

---

## 1. 왜 하나

### 1-1. 지금까지

- **지금 sim 값(SysID 학습 환경)의 출처**
  - 데이터: 11-17 오른팔 한 팔, Franka Hand, 다른 base.
  - 방법: Isaac **100 Hz**에서 관절별 CMA-ES로 fit했다.
  - 1 kHz에서는 fit한 적이 없다.
- **11-26 로봇 없이 확인한 것 (9/29 양팔·판 데이터, 11-05 박스 bag)**
  - 식별값은 왼팔과 판 EE에도 통한다. 왼팔은 오른팔보다 오차가 약 10 % 클 뿐이다.
  - 마찰은 움직인 이력에 따라 다르다. 한 방향으로 길게 갈 때가 짧게 왕복할 때보다 20~50 % 크다.
  - 멈췄다 출발하는 문턱은 직전에 움직인 방향 쪽으로 약 0.34 Nm 치우쳐 있다.
  - c는 자세에 따라 다르다. j2가 ready에서 −0.35 Nm, 작업 자세(j2 ≈ 1.6)에서 +0.55 Nm였다.

### 1-2. 가설과 판정 방법

**가설 1: 같은 관절의 최적값이 조건에 따라 바뀐다. 그래서 고정값 하나로는 다 못 맞추고, 학습 때 DR이 필요하다.**
- **조건:** 자세, 팔, 시간·온도, 움직임 이력, 시뮬레이터(Isaac과 MuJoCo).
- **판정 방법:** "조건 i에서 맞춘 값으로 조건 j를 재생한 오차" 표를 만든다.
  - 대각선 밖이 **반복 하한**(같은 자세를 두 번 했을 때 R0 1회차 대 2회차)보다 확실히 크면 맞다.
  - 그 차이가 DR 범위의 근거다.
- **값이 아니라 오차로 판정한다.** A, c_v, F_d는 서로 대신할 수 있다. 그래서 조건마다 값이 달라도 재생 오차는 같을 수 있다.
- **시뮬레이터가 다르면** 마찰 모델 모양부터 다르다. Isaac은 F_s, F_d, c_v로 나뉘고, MuJoCo는 `frictionloss` 하나다. 같은 데이터·같은 비용으로 각각 fit한다. 이것만으로 나오는 결론은 "값은 sim마다 따로 맞춰야 하는 등가값"이다. DR의 근거는 각 sim 안에서 남는 오차와 조건 변동이다.

**가설 2: 100 Hz와 1 kHz에서 잘 맞는 A, c_v 등이 다르다.**
- 같은 bag을 Isaac 100 Hz와 1 kHz로 재생해서 fit하고, 서로 값을 바꿔 넣은 오차를 본다.
- 재생은 두 가지로 한다.
  - **PD 없이:** 실물 τ_J,d를 그대로 넣는다. plant 적분 몫만 본다.
  - **PD 넣고:** 실물 명령으로 sim 주기에 맞춰 PD를 돌린다. 학습 sim과 같은 조건이다.
- 11-25에 11-17 값으로 PD 없이 재생했을 때는 1 kHz 9.05, 100 Hz 9.27 mrad로 거의 같았다. 차이가 난다면 PD 토크를 10 ms 동안 그대로 두는 쪽에서 날 것으로 본다.
- 이 영향은 관성이 작은 손목(j5~j7)에서 크다. K_d·dt/(M_jj + A)가 j6에서 100 Hz 약 0.3, j2에서 약 0.02다.

### 1-3. 같이 얻는 것

- **응답·지연:** P의 계단 도달 시간, 사인 이득과 위상, chirp 주파수 응답. ready와 hover, 양팔이다. 11-17은 한 팔뿐이었다.
- **명령 경로 지연:** `/policy_joint_command` 발행 시각과 τ_J,d가 바뀐 시각의 차이. 11-17은 한 팔 1.9 ms였다. 이번에 양팔(Husky)에서 분포와 꼬리(p99, 최대)를 본다.
- **joint_states 나이:** header 시각과 받은 시각의 차이.

---

## 2. 기록하는 값

양팔 `/left_franka_robot_state_broadcaster/robot_state`, `/right_…/robot_state`(`franka_msgs/FrankaRobotState`, 1 kHz)에 다 들어 있다.

| 값 | 필드 | 쓰는 곳 |
|---|---|---|
| τ_J,d (= 우리 τ_cmd) | `desired_joint_state.effort` | **재생 입력.** 11-17에 우리 PD 식으로 다시 계산한 값과 0.014 Nm 차이, 반영 1.9 ms로 확인했다 |
| τ_J (관절 토크 센서) | `measured_joint_state.effort` | 확인용(τ_J − τ_J,d ≈ 중력 + 남은 손실), 링크 쪽 fit 비교 |
| q, q̇ | `measured_joint_state.position / velocity` | 비교 대상, 재생 구간의 시작 상태 |
| θ, θ̇ (모터 쪽, 링크 단위) | `measured_joint_motor_state` | 관절 비틀림. 11-05에 j1~j4 약 2.2만 Nm/rad로 나왔다 |
| 외력 추정 | `tau_ext_hat_filtered`, `o_f_ext_hat_k` | 접촉 구간 거르기 |
| EE 질량 | `inertia_ee` | Desk 설정 확인. 판이면 0.1324 kg |

- 각 팔 토픽의 배열은 14칸이다. 그 팔의 7관절이 0~6칸에 들어 있다.
- **bag 크기:** 11-05 박스 bag은 약 9.6 MB/s였다. 시험 시간을 다 합치면 약 100분이라 **약 55 GB**다. 시작 전에 디스크를 확인한다(7절).
- **sysid CSV는 노드 주기(10 ms)로만 남는다.** 1 kHz 재생은 bag으로 한다. 그래서 **모든 시험에서 bag을 켠다.** dry run만 예외다.

---

## 3. 자세

| 이름 | 왼팔 | 오른팔 | 이유 |
|---|---|---|---|
| **R0** ready | `-0.5236 -0.7854 0 -2.3562 0 1.5708 0.7854` | `0.5236 -0.7854 0 -2.3562 0 1.5708 0.7854` | 9/29 sysid 자세(측정 q0와 몇 mrad 차이). 노드의 기본 양팔 자세와 같다 |
| **M** 중간 | `-0.2033 0.0576 -0.0682 -2.1578 -0.5026 1.5392 0.6362` | `0.2033 0.0576 0.0682 -2.1578 0.5026 1.5392 0.9346` | R0와 H의 관절값 가운데. c가 j2를 따라 매끄럽게 이어지는지 본다 |
| **H** hover | `0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869` | `-0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839` | 박스 정책의 hover 시작 자세([dual_fr3_box_torque_eval_experi_step.md](dual_fr3_box_torque_eval_experi_step.md) 4-1과 같다) |
| **G** pregrasp | `0.0543 1.6190 -0.1589 -1.3667 -0.9633 1.6154 0.5753` | `0.0322 1.6089 0.1799 -1.4290 1.0418 1.6718 0.9569` | 11-05 실물 정책이 시작한 자세(bag `030707`, 정지 구간 평균). c가 가장 컸던 자세 |

**관절 한계 검사 (노드: 한계에서 0.02 rad 안쪽)**

| 자세 | 시험 | 가장 가까운 한계까지 남는 양 |
|---|---|---|
| R0 | S (±0.15, 손목 ±0.075) / P (±0.12) | 0.55 / 0.58 rad |
| H | S / P / T | 0.77 / 0.80 rad |
| M | C (±0.05) | 0.85 rad |
| G | C (±0.05) | 0.15 rad |
| G | S를 한다면 (±0.15) | 0.05 rad. 통과는 한다 |

모두 통과한다. G에서 짧은 시험만 하는 이유는 관절 한계가 아니라 **판과 책상 사이 간격** 때문이다.

**안전**
- **박스는 치운다.** 판과 손이 아무것에도 닿지 않아야 한다. 접촉력은 τ_J,d에 없어서, 닿은 구간은 재생이 틀린다.
- **H:** S에서 j2를 +0.15 rad 움직이면 판이 약 9 cm 내려간다. 판 아래 여유를 본다. 한 번에 관절 하나만 움직이므로, 두 판 사이는 최대 약 8 cm 가까워진다.
- **G:** 판이 책상 가까이 있다. C는 ±0.05 rad라 판이 약 3 cm 움직인다. **H에서 G로 내려갈 때와 G에서 올라올 때 판이 책상에 닿지 않는지 눈으로 본다.** G에서 R0로 바로 가지 않고 H를 거친다.

---

## 4. 시험

### S: sysid 노드 rich 프로파일 (9/29와 같은 설정)

- 관절 하나씩, 양팔 14관절을 왼팔 j1~j7, 오른팔 j1~j7 순서로 자동으로 돈다. 나머지 13관절은 q0에 붙잡혀 있다.
- 관절마다 하는 것:
  - 등속 왕복: 진폭 ±0.15 rad, 속도 0.05 / 0.10 / 0.15 rad/s, 가감속 0.3 s, 정지 0.4 s.
  - 왕복 종류는 같은 방향으로 나가기, 방향 바꿔 돌아오기, 반대쪽으로 나가기, 다시 돌아오기 네 가지다.
  - 그다음 Hann 창 사인 0.5 / 1 Hz, ±0.05 rad, 3주기.
- 손목(j5~j7)은 진폭이 절반이다.
- 명령 100 Hz. 관절당 약 42 s, **14관절 약 12분**.
- 파라미터는 노드 기본값 그대로 둔다. 9/29 launch의 기본값과 같다.

### P: probe sweep (11-17과 같은 설정)

- 팔 하나씩, j1~j7.
  - 계단: ±0.02 / 0.05 / 0.10 rad.
  - 사인: 0.5 / 1 / 2 / 3 Hz.
  - chirp: 0.2 → 5 Hz.
- 명령 20 Hz로 정책과 같은 경로다. trial 21개, **팔당 약 10분**.
- 계단이 목표를 한 번에 바꾼다. 정책이 한 step에 바꾸는 최대 0.20 rad의 절반 수준이다.

### C: 짧은 c 시험

- 관절: j1, j2, j4, 양팔 6개 (sweep 번호 0, 1, 3, 7, 8, 10).
- 등속 왕복만 한다. 진폭 ±0.05 rad, 속도 0.05 → 0.10 → 0.05 → 0.10 rad/s(두 번 반복). 사인은 없다.
- **약 3분.**
- `wrist_excitation_scale:=1.0`을 꼭 준다.
  - 노드는 시험하지 않는 손목용 궤적도 미리 만든다.
  - 손목 배율 0.5면 진폭이 0.025 rad가 된다. 그러면 0.10 rad/s의 가감속 거리(0.03 rad)보다 짧아서 노드가 시작할 때 거부한다.

### T: 왕복 길이 시험 (hover)

- 관절: j2, j4, 양팔 4개 (sweep 번호 1, 3, 8, 10).
- C와 같은 설정이다(진폭 0.05 rad, 속도 0.05 / 0.10).
- 같은 자세의 S(진폭 0.15 rad, 같은 속도)와 비교한다. 속도는 같고 한 방향으로 가는 길이만 다르다.
- **약 2분.**

---

## 5. 순서

| # | 자세 | 시험 | label | 시간 | 보는 것 |
|---|---|---|---|---|---|
| 0 | | 준비: 컨트롤러, 임계값, 확인, R0, dry run | | 15분 | |
| 1 | R0 | S | `r0_S1` | 12분 | **전원 켠 직후.** 기준, 9/29와 비교(2달), 시간·온도 기준 |
| 2 | R0 | S | `r0_S2` | 12분 | **반복 하한**(가설 1의 기준선). 1번 끝나자마자 |
| 3 | R0 | P 오른팔 | `r0_P_R` | 10분 | 응답·지연, 가설 2 |
| 4 | R0 | P 왼팔 | `r0_P_L` | 10분 | 팔 차이 |
| 5 | M | C | `m_C` | 3분 | 자세에 따른 c |
| 6 | H | S | `h_S` | 12분 | 자세 의존 (가설 1) |
| 7 | H | P 오른팔 | `h_P_R` | 10분 | 작업 자세의 응답·지연 |
| 8 | H | P 왼팔 | `h_P_L` | 10분 | |
| 9 | H | T | `h_T` | 2분 | 이력 의존 (6번과 비교) |
| 10 | G | C | `g_C` | 3분 | 작업 자세 c (11-05 bag 추정값 +0.55와 비교) |
| 11 | R0 (H를 거쳐) | S | `r0_S3` | 12분 | **시간·온도** (1번과 비교). 켠 지 1.5시간 이상 지나서 |

- 자세 이동까지 더하면 약 2시간이다. 11번 전에 1.5시간이 안 됐으면 기다린다.
- 시간이 모자라면 10 → 5 → 4 → 8 순으로 뺀다. **1, 2, 6, 11은 꼭 한다.** 3·7도 가능하면 한다.

---

## 6. 터미널

| 터미널 | 용도 |
|---|---|
| T1 | 컨트롤러. 세션 내내 켜 둔다 |
| T2 | 짧은 명령: 임계값, 자세 이동, 시험 시작·정지, 확인 |
| T3 | bag. 시험마다 켜고 끈다 |
| T4 | sysid 노드 (S, C, T). 시험마다 다시 띄운다 |
| T5 | probe (P). 시험마다 다시 띄운다 |

**모든 터미널(T1~T5)에서 처음에 한 번:**
```bash
source ~/jh_ws/ros2_ws/install/setup.bash
source ~/jw_ws/install/setup.bash
cd ~/jw_ws/src/fr3_husky
```
- jh_ws(franka underlay)를 먼저, jw_ws를 나중에 source한다. 순서와 빌드 옵션은 [fr3_command_response_probe_experi_step.md](fr3_command_response_probe_experi_step.md) 0절과 같다.
- 한 터미널에서 확인한다. 아래 명령이 `fr3_sysid_node`와 `fr3_command_response_probe` 두 줄을 내야 한다.
  ```bash
  ros2 pkg executables fr3_husky_nn_policy | grep -E "fr3_sysid_node|fr3_command_response_probe"
  ```

**알아 둘 것**
- **시작 서비스(T2)는 노드(T4, T5)가 다 뜬 뒤에 따로 보낸다.** 노드 로그에 설정 줄이 나온 뒤에 보낸다.
- **노드는 한 번에 하나만 띄운다.** sysid 노드와 probe는 같은 PolicyControl action을 쓴다.
- **sysid 노드는 launch가 아니라 `ros2 run`으로 띄운다.**
  - `fr3_sysid.launch.py`는 컨트롤러를 따로 하나 더 띄운다. 그러면 probe와 번갈아 쓸 때마다 T1을 다시 띄워야 한다.
  - `ros2 run`에서는 `-p return_to_start_between_trials:=true`를 직접 줘야 한다. 빠지면 노드가 시작하지 않는다. launch는 이 값을 넣어 준다.
- **긴 명령은 `\`로 줄을 나눴다.** 한 덩어리째 붙여 넣는다.

---

## 7. 세션 준비 (전원 켜기 전후)

- [ ] **전원 켠 시각을 적는다.** 11번(시간·온도)의 기준이다.
- [ ] **Husky 받침 기울기를 잰다.** 휴대폰 수평계를 팔 장착면 근처 윗판에 놓고 앞뒤·좌우 각도를 적는다. 9/29 j1 토크가 위치에 따라 1.7 Nm/rad로 변하던 것이 기울기 때문인지 가린다.
- [ ] **디스크:** `df -h ~`에서 남은 공간이 **80 GB 이상**인지 본다. 모자라면 bag 경로(`~/jw_ws/bags/...`)를 외장 디스크로 바꿔서 명령을 친다.
- [ ] **Desk EE 설정이 판인지 본다.** 0단계 ④에서 0.1324 kg이 나오는지 확인한다. 컨트롤러의 중력 보상은 Desk 값을 쓴다.
- [ ] **박스를 치운다.** 판 아래와 주변에 10 cm 이상 비어 있는지 본다.
- [ ] **비상정지를 손에 둔다.**
- [ ] **bag을 기록하는 동안 로봇 PC에서 colcon 빌드나 무거운 작업을 하지 않는다.** 1 kHz 루프가 늦어지면 데이터가 끊기거나 reflex가 난다.

---

## 8. 단계별 명령

**모든 시험의 흐름**
1. T2: 그 시험의 자세로 이동. **자세가 같아도 매 시험 전에 다시 보낸다.** 끝나고 팔이 멈췄는지 본다.
   - 시험이 끝나면 손목·j2·j4가 정지마찰로 q0 근처에 남고, 다음 시험은 그 자리를 새 q0로 잡는다. 11-26 R0에서 S1 → S2 → P_R → P_L로 갈수록 q0가 쌓여 밀렸다(왼팔 j2 0.3 → 6.8 → 11.9 → 17.1 mrad, 오른팔 j4 2.0 → −4.5 → −11.2 → −14.8 mrad).
   - 다시 이동하면 시험마다 시작 자세와 직전 움직임 이력이 비슷해진다. 1·2·11번(반복 하한, 시간·온도)을 비교할 때 자세 차이가 섞이지 않는다.
   - move_to_joint도 j7에 약 15 mrad가 남는다(S1 q0). 그래서 완벽히 같은 자세가 되지는 않는다. 쌓이는 것만 막는다.
2. T3: bag 시작.
3. T4 또는 T5: 노드 실행. 설정 로그를 확인한다.
4. T2: 시작 서비스. 응답이 `success=True`인지 본다.
5. 끝 메시지를 기다린다.
   - sysid: `captured repeatable ... home from live q0` → `SysID PolicyControl goal accepted; excitation started` → `SysID sweep complete; stopping at stable q0`
   - probe: `q0 latched` → `[1/21] ...` → `all trials done; q0 is stable`
6. T4/T5에서 Ctrl+C → T3에서 Ctrl+C.
7. 기록 표(10절)에 시작·끝 시각과 특이사항을 적는다.

**중간에 멈출 때 (T2)**
```bash
# sysid 노드(S, C, T)
ros2 service call /fr3_sysid_node/stop_probe std_srvs/srv/Trigger {}
# probe(P)
ros2 service call /fr3_command_response_probe/stop_probe std_srvs/srv/Trigger {}
```
급하면 비상정지를 누른다.

### 0. 준비 (약 15분)

**① T1: 컨트롤러 (양팔, 토크 모드, MoveIt 포함)**
```bash
ros2 launch fr3_husky_controller fr3_action_controller.launch.py robot_side:=dual load_gripper:=false command_interface:=effort
```
로그에 `dual_fr3_action_controller`, `left_franka_robot_state_broadcaster`, `right_franka_robot_state_broadcaster`가 activated로 나와야 한다.

**② T2: 컨트롤러 확인**
```bash
ros2 control list_controllers | grep -E "action_controller|robot_state_broadcaster"
```
위 세 개가 `active`여야 한다.

**③ T2: 충돌 임계값** (T1을 다시 띄울 때마다 다시 보낸다)
```bash
ros2 service call /service_server/set_force_torque_collision_behavior franka_msgs/srv/SetForceTorqueCollisionBehavior \
  "{lower_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], upper_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], \
    lower_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0], upper_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]}"
```
응답이 `success=True`인지 보고 시각을 적는다.

**④ T2: Desk EE 질량 확인** (양팔 모두 0.1324가 나와야 한다. 다르면 Desk 설정부터 맞춘다)
```bash
ros2 topic echo --once /left_franka_robot_state_broadcaster/robot_state --field inertia_ee.inertia.m
ros2 topic echo --once /right_franka_robot_state_broadcaster/robot_state --field inertia_ee.inertia.m
```

**⑤ T2: R0로 이동** (첫 이동은 팔이 Husky나 서로에게 가까이 가지 않는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854 \
  --right-target-positions 0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854
```

**⑥ dry run: S 설정, j4 한 관절** (bag 없이, 로봇 안 움직임)

T4:
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=true \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/dry_S/real \
  -p "sweep_joints:=[3]"
```
T4 로그에 `SysID excitation ready: profile=rich, friction=True, inertia=True, dry_run=True`가 나오면 T2:
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```
T4 로그에 `captured repeatable ... home from live q0`와 시작 로그가 나오면 T4에서 Ctrl+C.

**⑦ dry run: C 설정이 노드에서 거부되지 않는지**

T4:
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=true \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/dry_C/real \
  -p "sweep_joints:=[0, 1, 3, 7, 8, 10]" \
  -p do_inertia_sine:=false -p friction_amplitude_rad:=0.05 \
  -p "friction_speeds_rad_s:=[0.05, 0.10, 0.05, 0.10]" -p wrist_excitation_scale:=1.0
```
`SysID excitation ready: profile=rich, friction=True, inertia=False, dry_run=True`가 나오면 바로 T4에서 Ctrl+C. 시작 서비스는 보내지 않는다.

**⑧ dry run: probe**

T5:
```bash
ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py \
  robot:=dual launch_controller:=false command_interface:=effort dry_run:=true \
  excite_arm:=right label:=dry "notes:=dry run"
```
설정 로그가 나오면 T2:
```bash
ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
```
T5 로그에 `q0 latched`와 `[1/21]`이 나오면 T5에서 Ctrl+C.

---

### 1. R0, S 1회차 (`r0_S1`, 전원 켠 직후, 약 12분)

팔은 R0에 있다(0단계 ⑤).

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/r0_S1_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T4: sysid 노드**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/r0_S1/real
```
`SysID excitation ready: profile=rich, friction=True, inertia=True, dry_run=False`를 확인한다.

**③ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**④ 끝나면:** `SysID sweep complete; stopping at stable q0` → T4 Ctrl+C → T3 Ctrl+C.

### 2. R0, S 2회차 (`r0_S2`, 1번 끝나자마자, 약 12분)

**⓪ T2: R0로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854 \
  --right-target-positions 0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/r0_S2_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T4: sysid 노드**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/r0_S2/real
```

**③ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**④ 끝나면:** T4 Ctrl+C → T3 Ctrl+C.

### 3. R0, P 오른팔 (`r0_P_R`, 약 10분)

**⓪ T2: R0로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854 \
  --right-target-positions 0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/r0_P_R_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T5: probe**
```bash
ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py \
  robot:=dual launch_controller:=false command_interface:=effort dry_run:=false \
  excite_arm:=right label:=r0_P_R "notes:=dual on Husky, plate EE, pose R0 (9/29 ready)"
```

**③ T2: 시작**
```bash
ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
```

**④ 끝나면:** `all trials done; q0 is stable` → T5 Ctrl+C → T3 Ctrl+C.

### 4. R0, P 왼팔 (`r0_P_L`, 약 10분)

**⓪ T2: R0로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854 \
  --right-target-positions 0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/r0_P_L_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T5: probe**
```bash
ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py \
  robot:=dual launch_controller:=false command_interface:=effort dry_run:=false \
  excite_arm:=left label:=r0_P_L "notes:=dual on Husky, plate EE, pose R0 (9/29 ready)"
```

**③ T2: 복귀 허용치를 넓히고 시작** (T5 설정 로그가 나온 뒤. 순서대로 보낸다)
```bash
ros2 param set /fr3_command_response_probe home_position_tolerance_rad 0.025
ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
```
- 첫 줄 응답이 `Set parameter successful`인지 본다.
- 기본 허용치 0.01 rad은 Kp 80에서 손목 정지마찰로 남는 오차보다 작다. 11-26 `r0_P_L`에서 왼팔 j5가 −0.10 계단 뒤 q0에서 14.5 mrad 떨어진 채 멈췄고, 30 s 뒤 `timed out waiting for the q0 return to settle`로 중단됐다(오른팔 j5는 9.5 mrad로 간신히 통과). 0.025는 관측 최대 16.8 mrad에 여유를 둔 값이다.

**④ 끝나면:** T5 Ctrl+C → T3 Ctrl+C.

### 5. M, C (`m_C`, 약 3분)

**① T2: M으로 이동** (팔이 멈출 때까지 기다린다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.2033 0.0576 -0.0682 -2.1578 -0.5026 1.5392 0.6362 \
  --right-target-positions 0.2033 0.0576 0.0682 -2.1578 0.5026 1.5392 0.9346
```

**② T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/m_C_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**③ T4: sysid 노드 (짧은 등속 왕복, j1·j2·j4 양팔)**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/m_C/real \
  -p "sweep_joints:=[0, 1, 3, 7, 8, 10]" \
  -p do_inertia_sine:=false -p friction_amplitude_rad:=0.05 \
  -p "friction_speeds_rad_s:=[0.05, 0.10, 0.05, 0.10]" -p wrist_excitation_scale:=1.0
```
`inertia=False`, `dry_run=False`를 확인한다.

**④ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**⑤ 끝나면:** T4 Ctrl+C → T3 Ctrl+C.

### 6. H, S (`h_S`, 약 12분)

**① T2: H로 이동** (도착 뒤 판 아래 9 cm 이상 비었는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

**② T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/h_S_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**③ T4: sysid 노드**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/h_S/real
```

**④ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**⑤ 끝나면:** T4 Ctrl+C → T3 Ctrl+C.

### 7. H, P 오른팔 (`h_P_R`, 약 10분)

**⓪ T2: H로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/h_P_R_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T5: probe**
```bash
ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py \
  robot:=dual launch_controller:=false command_interface:=effort dry_run:=false \
  excite_arm:=right label:=h_P_R "notes:=dual on Husky, plate EE, pose hover"
```

**③ T2: 복귀 허용치를 넓히고 시작** (T5 설정 로그가 나온 뒤. 순서대로 보낸다)
```bash
ros2 param set /fr3_command_response_probe home_position_tolerance_rad 0.025
ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
```
- 첫 줄 응답이 `Set parameter successful`인지 본다.
- 기본 허용치 0.01 rad은 Kp 80에서 손목 정지마찰로 남는 오차보다 작다. 11-26 `r0_P_L`에서 왼팔 j5가 −0.10 계단 뒤 q0에서 14.5 mrad 떨어진 채 멈췄고, 30 s 뒤 `timed out waiting for the q0 return to settle`로 중단됐다(오른팔 j5는 9.5 mrad로 간신히 통과). 0.025는 관측 최대 16.8 mrad에 여유를 둔 값이다.

**④ 끝나면:** T5 Ctrl+C → T3 Ctrl+C.

### 8. H, P 왼팔 (`h_P_L`, 약 10분)

**⓪ T2: H로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/h_P_L_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T5: probe**
```bash
ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py \
  robot:=dual launch_controller:=false command_interface:=effort dry_run:=false \
  excite_arm:=left label:=h_P_L "notes:=dual on Husky, plate EE, pose hover"
```

**③ T2: 복귀 허용치를 넓히고 시작** (T5 설정 로그가 나온 뒤. 순서대로 보낸다)
```bash
ros2 param set /fr3_command_response_probe home_position_tolerance_rad 0.025
ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
```
- 첫 줄 응답이 `Set parameter successful`인지 본다.
- 기본 허용치 0.01 rad은 Kp 80에서 손목 정지마찰로 남는 오차보다 작다. 11-26 `r0_P_L`에서 왼팔 j5가 −0.10 계단 뒤 q0에서 14.5 mrad 떨어진 채 멈췄고, 30 s 뒤 `timed out waiting for the q0 return to settle`로 중단됐다(오른팔 j5는 9.5 mrad로 간신히 통과). 0.025는 관측 최대 16.8 mrad에 여유를 둔 값이다.

**④ 끝나면:** T5 Ctrl+C → T3 Ctrl+C.

### 9. H, T (`h_T`, 약 2분)

팔은 H에 있다.

**⓪ T2: H로 다시 이동** (앞 시험의 T4/T5를 Ctrl+C한 뒤. 팔이 멈췄는지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

**① T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/h_T_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**② T4: sysid 노드 (짧은 등속 왕복, j2·j4 양팔)**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/h_T/real \
  -p "sweep_joints:=[1, 3, 8, 10]" \
  -p do_inertia_sine:=false -p friction_amplitude_rad:=0.05 \
  -p "friction_speeds_rad_s:=[0.05, 0.10, 0.05, 0.10]" -p wrist_excitation_scale:=1.0 \
  -p home_position_tolerance_rad:=0.025
```

**③ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**④ 끝나면:** T4 Ctrl+C → T3 Ctrl+C.

### 10. G, C (`g_C`, 약 3분)

**① T2: G로 이동** (내려가는 동안 판이 책상에 닿지 않는지 본다. 도착 뒤 판과 책상 사이가 3 cm 이상인지 본다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.0543 1.6190 -0.1589 -1.3667 -0.9633 1.6154 0.5753 \
  --right-target-positions 0.0322 1.6089 0.1799 -1.4290 1.0418 1.6718 0.9569
```

**② T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/g_C_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**③ T4: sysid 노드 (짧은 등속 왕복, j1·j2·j4 양팔)**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/g_C/real \
  -p "sweep_joints:=[0, 1, 3, 7, 8, 10]" \
  -p do_inertia_sine:=false -p friction_amplitude_rad:=0.05 \
  -p "friction_speeds_rad_s:=[0.05, 0.10, 0.05, 0.10]" -p wrist_excitation_scale:=1.0 \
  -p home_position_tolerance_rad:=0.025
```

**④ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**⑤ 끝나면:** T4 Ctrl+C → T3 Ctrl+C.

### 11. R0, S 3회차 (`r0_S3`, 켠 지 1.5시간 이상, 약 12분)

**① T2: 먼저 H로 올라온다** (G에서 R0로 바로 가지 않는다)
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions 0.1170 0.9006 -0.1363 -1.9593 -1.0052 1.5076 0.4869 \
  --right-target-positions -0.1170 0.9006 0.1363 -1.9593 1.0052 1.5076 1.0839
```

**② T2: R0로 이동**
```bash
ros2 run fr3_husky_task_manager move_to_joint \
  --left-target-positions -0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854 \
  --right-target-positions 0.5236 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854
```

**③ T3: bag**
```bash
mkdir -p ~/jw_ws/bags/sysid_261126 && ros2 bag record -o ~/jw_ws/bags/sysid_261126/r0_S3_$(date +%H%M%S) \
  /dual_fr3/joint_states /policy_joint_command /rosout \
  /left_franka_robot_state_broadcaster/robot_state /right_franka_robot_state_broadcaster/robot_state
```

**④ T4: sysid 노드**
```bash
ros2 run fr3_husky_nn_policy fr3_sysid_node --ros-args \
  -p return_to_start_between_trials:=true -p sweep_mode:=true -p dry_run:=false \
  -p log_root:=$HOME/jw_ws/src/fr3_husky/data/sysid_dual_261126/r0_S3/real \
  -p home_position_tolerance_rad:=0.025
```

**⑤ T2: 시작**
```bash
ros2 service call /fr3_sysid_node/start_probe std_srvs/srv/Trigger {}
```

**⑥ 끝나면:** T4 Ctrl+C → T3 Ctrl+C. 세션 끝. T1은 마지막에 Ctrl+C.

---

## 9. 문제 생길 때

| 증상 | 할 일 |
|---|---|
| sysid 노드가 시작할 때 `requires return_to_start_between_trials:=true` | T4 명령에서 `-p return_to_start_between_trials:=true`가 빠졌다 |
| sysid 노드가 시작할 때 `friction_amplitude_rad must exceed speed * friction_blend_s` | C, T 명령에서 `-p wrist_excitation_scale:=1.0`이 빠졌다 |
| `cannot run the full +/-... SysID trajectory inside its ... safe band` | 자세가 다르다. 그 단계의 move_to_joint를 다시 보내고, 팔이 멈춘 뒤 시작한다 |
| `joint_states has no finite effort ... tau_meas_* will be NaN` 경고 | CSV의 토크 열만 비고, bag의 `robot_state`에는 τ가 있다. 계속해도 된다. 기록 표에 적는다 |
| 시작 서비스가 `probe already active or pending`이나 q0 관련 메시지로 거부 | 팔이 아직 움직이고 있다. 몇 초 뒤 다시 보낸다 |
| 관절 사이 q0 복귀에서 `timed out after 45.0 s waiting for SysID q0 return` | 정지마찰 때문에 손목이 허용 오차 0.01 rad 안에 못 돌아온 것일 수 있다. 시작 서비스를 다시 보낸다. 반복되면 T4 명령 끝에 `-p home_position_tolerance_rad:=0.025`를 붙여 다시 띄우고 기록 표에 적는다(9~11번 명령에는 이미 들어 있다). sysid 노드는 이 값을 시작할 때만 읽으므로 `param set`으로는 안 바뀐다. 남은 관절만 하려면 `"sweep_joints:=[...]"`로 이어서 한다(번호는 0부터, 로그의 `[k/14]`는 1부터). 11-26 `h_S`에서 오른팔 j2가 10.1 mrad로 걸려 `[9, 10, 11, 12, 13]`으로 이어서 했다 |
| probe가 `timed out waiting for the q0 return to settle`로 중단 | 같은 이유다. 4·7·8번 ③의 `param set`(0.025)을 빠뜨리지 않았는지 본다. 3번처럼 빠진 시험이면 T5를 다시 띄우고 `param set` 뒤 처음부터 다시 한다(q0는 시작할 때 새로 잡힌다). 0.025로도 반복되면 T5 명령에 `"joints:=[1, 2, 3, 4, 5, 6]"`처럼 그 관절을 빼고 띄우고 적는다 |
| reflex로 멈춤 | T1을 다시 띄우고 0단계 ③ 임계값을 다시 보낸다. 그 시험은 처음부터 다시 하고, 시각을 적는다 |
| bag 기록 중 빌드나 무거운 작업을 했다 | 그 시험을 버리고 다시 한다 |
| 디스크가 찬다 | T3 Ctrl+C. 이미 끝난 bag을 외장 디스크로 옮긴 뒤 이어서 한다 |

---

## 10. 기록 표

세션: 날짜 2026-11-26, 전원 켠 시각 기록 없음(T1 첫 실행 16:26:48. "켠 뒤" 열은 이 시각 기준), Husky 기울기 앞뒤 ____° 좌우 ____°, Desk EE ______ kg, 충돌 임계값 넣은 시각 ______

- 11-26 끝난 뒤: bag 11개(47 GB)와 CSV를 5070(`/home/dyros/bags/sysid_261126/`, `.../fr3_husky_etri/data/fr3_dual_sysid_20261126/`)으로 옮기고 로봇 PC bag은 지웠다. CSV는 로봇 PC에도 남겼다.

| # | label | 시작 | 끝 | 켠 뒤 [분] | 결과 (completed / aborted) | 메모 (소리, reflex, 끊김, 재시도) |
|---|---|---|---|---|---|---|
| 1 | r0_S1 | 16:28 | 16:37 | 2 | completed (14/14) | bag `r0_S1_162840`. dry run(16:27~16:28) 뒤 바로 |
| 2 | r0_S2 | 16:38 | 16:47 | 11 | completed (14/14) | bag `r0_S2_163813` |
| 3 | r0_P_R | 16:47 | 16:57 | 21 | completed (21/21) | bag `r0_P_R_164729`. 복귀 허용치 0.01(그때는 `param set` 없음). step j5 −0.10 뒤 −9.5 mrad로 간신히 통과 |
| 4 | r0_P_L | 17:09 | 17:19 | 42 | completed (21/21, 2차) | 1차 16:58~17:00 aborted(4/21): step j5 −0.10 뒤 −14.5 mrad로 `timed out waiting for the q0 return to settle`(tol 0.01). 1차 CSV `261126_165801_dual_r0_P_L` 남김, 1차 bag은 지움. T1 다시 띄움(17:01:51, 17:06:39). 2차는 `param set` tol 0.025, bag `r0_P_L_170901` |
| 5 | m_C | 17:19 | 17:22 | 53 | completed (6/6) | bag `m_C_171926` |
| 6 | h_S | 17:23 | 17:35 | 57 | completed (2회로 나눠 함) | 오른팔 j2(sweep 8) 뒤 q0 복귀 타임아웃(−10.1 mrad, tol 0.01). H로 다시 이동 후 `sweep_joints:=[9..13]`, tol 0.025로 이어서 함. CSV는 `h_S/real/` 한 폴더(14개). bag 2개 `h_S_172327` + `h_S_resume_173207` → 11절대로 합친다 |
| 7 | h_P_R | 17:37 | 17:47 | 71 | completed (21/21) | bag `h_P_R_173719`. tol 0.025 |
| 8 | h_P_L | 17:48 | 17:58 | 81 | completed (21/21) | probe 끝난 뒤 17:59:11에 디스크가 차서 bag 기록이 `SqliteException ... disk is full`로 비정상 종료. 시험 구간(17:48:17~17:57:48)은 모두 들어 있다(robot_state 양팔 각 약 66.6만 개, 1 kHz, `quick_check` ok). `ros2 bag reindex`로 `metadata.yaml`을 다시 만들고 storage id를 `sqlite3`로 채웠다. 공간 확보로 11-17 bag·CSV·tar(5070에 있음)와 4번 첫 시도 bag `r0_P_L_165752`를 지웠다 |
| 9 | h_T | 18:04 | 18:06 | 98 | completed (4/4) | bag `h_T_180420`. tol 0.025 |
| 10 | g_C | 18:07 | 18:10 | 100 | completed (6/6) | bag `g_C_180702`. tol 0.025. 끝난 뒤 T1 종료(18:09:58) |
| 11 | r0_S3 | | | | 미룸 | 11-26에는 시간이 모자라 못 함. 다른 날 하면 같은 세션의 시간·온도 비교(1 대 11)가 아니라 날짜 간 비교가 된다 |

---

## 11. 끝난 뒤

**데이터 옮기기 (로봇 PC → 5070)**
- bag(약 55 GB)은 git에 넣지 않는다. 외장 디스크나 rsync로 옮긴다.
- sysid CSV(`data/sysid_dual_261126/`)와 probe CSV(`data/command_response/real/`)는 작다.
- dry run 폴더(`data/sysid_dual_261126/dry_*`, probe의 `..._dry`)는 지운다.
- 기록 표를 채운 이 문서도 같이 올린다.

**나눠 찍은 bag 합치기** (실험이 끝난 뒤. 기록 중에는 하지 않는다. 합친 크기만큼 디스크가 더 필요하다)
```bash
cd ~/jw_ws/bags/sysid_261126
cat > /tmp/merge_h_S.yaml <<'YAML'
output_bags:
- uri: h_S_merged
  storage_id: sqlite3
  all: true
YAML
ros2 bag convert -i h_S_172327 -i h_S_resume_173207 -o /tmp/merge_h_S.yaml
ros2 bag info h_S_merged   # 메시지 수 = 두 bag의 합인지 본다. 맞으면 원본 두 개를 지운다
```
- 두 bag 사이(H로 다시 이동하는 동안, 약 2분)는 비어 있다. 분석에서는 관절별 구간을 `/rosout`이나 CSV 시각으로 자르므로 문제없다.

**분석 (dual_fr3_lab, 5070)**

| 순서 | 무엇 | 입력 | 가설 |
|---|---|---|---|
| 1 | bag → 1 kHz 배열 (q, q̇, τ_J,d, τ_J, θ, 명령) | 양팔 robot_state | |
| 2 | 등속 구간 c·F_d·c_v, 출발 문턱 (11-26 도구) | S, C, T | 1 |
| 3 | Isaac 재생 (PD 없이 200 ms 구간 / PD 넣어 폐루프), 1 kHz와 100 Hz | 전부 | 1, 2 |
| 4 | 조건별 CMA-ES fit + bootstrap, "조건 i 값 → 조건 j" 오차 표 | S로 fit, P로 검증 | 1, 2 |
| 5 | 응답 지표 (계단 t50·t90, 사인 이득·위상, chirp), 명령 지연·joint_states 나이 분포 | P, bag | 응답·지연 |
| 6 | 같은 데이터로 MuJoCo fit (팀원과 같은 비용, τ_J,d 기준) | 전부 | 1 (시뮬레이터) |

비교할 쌍:

| 비교 | 의미 |
|---|---|
| 1 대 2 | 반복 하한 |
| 1 대 6 (+ 5·10의 c) | 자세 |
| 모든 S의 왼팔 대 오른팔 | 팔 |
| 1 대 11 | 시간·온도 |
| 1 대 9/29 데이터 | 2달 |
| 6 대 9 | 왕복 길이 (이력) |

## 참고

- [fr3_joint_friction_261125_experi_step.md](fr3_joint_friction_261125_experi_step.md): 11-25 계획(이 문서로 대신함), 1-5절 11-17 토크 재생 기준값
- [fr3_command_response_probe_experi_step.md](fr3_command_response_probe_experi_step.md): probe 사용법, source, 빌드 옵션
- `fr3_husky_nn_policy/fr3_husky_nn_policy/about_sysid.md` 8.4절: 9/29 sysid 수집 방법
- `data/sysid_260929_README.md`, `data/sysid_raw/real/`: 9/29 데이터
- dual_fr3_lab `docs/dual_fr3_joint_dynamics.md`: 관절 동역학 식, 11-17 식별값
