# FR3 명령 응답 probe 실험 순서 (오른팔 한 팔, 새 base, 2026-11-17)

## 한눈에 보기

**무엇을 하나:** 오른팔을 ready 자세에 세워 두고, 관절을 하나씩 조금 흔든다(sweep). 그동안 20 Hz 관절 목표에서 실제 관절 위치까지의 지연과 응답을 잰다. 같은 시험을 두 모드로 한 번씩 한다.
- **토크 모드**: 지금 박스 정책이 쓰는 경로(PD Kp 80)
- **위치 모드**: 새로 추가한 경로(1 kHz 저크 제한 추종기). 11-17 B3에서 reflex가 나서 추종기를 고쳤다(3-0)

결과는 sim 액추에이터 보정에 쓴다(lab `docs/todo_1116.md` §2, §6).

**이번 환경**

| 항목 | 값 |
|---|---|
| 팔 | 오른팔(right, 172.16.6.6) 한 팔 |
| 장착 | Husky가 아닌 다른 base. 판이 팔 장착면 아래로 내려갈 수 없다 |
| EE | 지금 달린 그대로(hand, 판 등). **그리퍼 노드는 띄우지 않는다**(`load_gripper:=false`) |
| 박스 | 없음 |
| 시작 자세 | ready에서 j1만 0: `0.0 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854` |
| 하는 시험 | sweep(step, sine, chirp), 정지(hold) |
| 안 하는 시험 | 박스 재생(A4, B4). 아래 [박스 재생을 안 하는 이유](#박스-재생a4-b4을-안-하는-이유) |

**전체 순서**
```
0. 터미널 준비 (T1~T4 모두 source)
1. 세션 준비 (Desk EE, 비상정지)
2. 토크 모드:  T1 effort → 임계값 → ready → A0 dry run → A1 j4 계단 → A2 전체 sweep → A3 정지 자세 3개
3. 위치 모드:  (ready는 2에서 토크 모드로) T1 position → 임계값 → B0 dry run → B1 유지 → B2 작은 계단 → B3 전체 sweep
               → B4 정지 자세 3개 (H2, H3는 토크 모드로 옮긴 뒤 위치 모드로 다시 띄운다)
4. 결과 가져오기
```
- 전체 sweep 한 번은 약 10분 걸린다(trial 21개).
- 모드마다 약 20~25분, 전체 약 1시간으로 잡는다.
- **11-17 이후 다시 할 때**: 토크 모드 A0~A3는 11-17에 끝났다(`data/command_response/real/261117_18*`). 위치 모드는 B3에서 멈췄다. 그래서 0절, 1절 → 2-1, 2-2, 2-3(토크 모드로 ready까지, A 시험은 안 함) → 3절 전체(B0부터)만 한다. 추종기 코드가 바뀌었으므로 B0~B2도 다시 한다.

## 얼마나 움직이나

모든 시험은 시작 자세(q0) 기준이다.
- 한 번에 **관절 하나만** 움직인다. 나머지 관절은 q0에 그대로 있다.
- trial이 끝날 때마다 q0로 돌아와 멈춘 뒤 다음 trial로 넘어간다.

| 시험 | 관절 이동 (q0 기준) | 관절당 시간 |
|---|---|---|
| step | ±0.02 → ±0.05 → ±0.10 rad. 목표를 한 번에 바꿔 1.5 s 유지하고, q0로 돌아와 1.5 s 쉰다 | 19 s |
| sine | 0.5·1 Hz는 0.05 rad, 2 Hz는 0.04 rad, 3 Hz는 0.027 rad. 속도 0.5 rad/s를 넘지 않게 자동으로 줄인다 | 28 s |
| chirp | 0.2 → 5 Hz를 30 s 동안. 0.03 rad에서 시작해 5 Hz에서는 0.016 rad로 줄어든다 | 32 s |
| hold | 0 (그 자세에서 정지 유지) | 3 s (B1은 10 s) |

판 끝(플랜지 + 10 cm)으로 보면 이렇다. 시작 위치는 팔 장착면에서 앞으로 31 cm, 위로 49 cm다.
- 가장 크게 움직이는 것은 j4 step 0.10 rad다. 4.9 cm 움직이고 4.6 cm 내려간다.
- j2 step은 3.4 cm다. 나머지 관절은 3.3 cm 이하이고, sine은 2.4 cm, chirp은 1.5 cm 이하다.
- 가장 낮을 때도 판 끝은 장착면보다 약 44 cm 위다. 판 주변에 10 cm 여유만 있으면 된다.
- probe는 시작 전에 모든 trial을 검사하고, 조건을 넘으면 시작하지 않는다.
  - 관절 이동: 0.12 rad 이하
  - 속도: 0.8 rad/s 이하
  - 관절 한계: 한계에서 0.02 rad 안쪽
  - 지금 기본 sweep은 0.10 rad, 0.62 rad/s로 통과한다.
- **주의할 것은 step 하나다.** 토크 모드에서는 목표가 한 번에 바뀌어 짧게 툭 움직인다. 정책도 한 스텝에 최대 0.20 rad까지 목표를 바꾸니, 그 절반 수준이다. 그래서 전체 sweep 전에 j4 step만 먼저 해 본다.

## 0. 터미널 준비

**모든 터미널(T1~T4)에서 이 순서로 source한다.**
```bash
source ~/jh_ws/ros2_ws/install/setup.bash    # franka underlay (alias jh). 먼저
source ~/jw_ws/install/setup.bash            # 나중
ros2 pkg prefix fr3_husky_controller         # /home/husky-rcu/jw_ws/install/fr3_husky_controller 이어야 한다
ros2 pkg prefix franka_hardware              # /home/husky-rcu/jw_ws/install/franka_hardware 이어야 한다 (limitRate)
```
- jh_ws에도 예전 `fr3_husky_*` 패키지가 있다. jh_ws를 나중에 source하면 jw_ws를 덮어쓴다.
- franka_hardware는 jw_ws에 복사본(`~/jw_ws/src/franka_ros2`, jh_ws와 같은 커밋 923742e)이 있다. 위치 명령 limitRate가 여기에만 들어 있다. jh_ws는 그대로다.
- 순서가 틀리면 [문제 생길 때](#문제-생길-때)의 첫 두 줄 에러가 난다.

| 터미널 | 용도 |
|---|---|
| T1 | 컨트롤러. 모드마다 한 번 띄운다 |
| T2 | 짧은 명령: 충돌 임계값, move_to_joint, start/stop |
| T3 | bag. 시험마다 켜고 끈다 |
| T4 | probe. 시험마다 다시 띄운다 |

T2, T3, T4에 아래 함수를 한 번 붙여 넣는다. `<…>`는 실제 값으로 바꾼다.
```bash
# T2
pstart() { ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}; }
pstop()  { ros2 service call /fr3_command_response_probe/stop_probe  std_srvs/srv/Trigger {}; }
ready()  { ros2 run fr3_husky_task_manager move_to_joint --arm right \
             --right-target-positions 0.0 -0.7854 0.0 -2.3562 0.0 1.5708 0.7854; }

# T3   사용: rec <label>
rec() { mkdir -p ~/jw_ws/bags && ros2 bag record -o ~/jw_ws/bags/cmdresp_$1_$(date +%m%d_%H%M%S) \
          /right_fr3/joint_states /right_franka_robot_state_broadcaster/robot_state /policy_joint_command /rosout \
          /right_fr3_position_action_controller/policy_position_debug; }

# T4
probe() { ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py robot:=right launch_controller:=false "$@"; }
NOTES="notes:=right single arm on new base, EE <hand/판 구성> <질량> kg, collision 100/80 Nm 100 N set <시각>, pose ready j1=0"
```
- `"$NOTES"`는 따옴표째 붙여야 한 인자로 넘어간다.
- `policy_position_debug`는 위치 모드에서만 1 kHz로 나온다(bag에서 10분에 약 430 MB). 토크 모드에서는 비어 있다.
- **11-17 추종기 수정을 pull한 뒤에는 다시 빌드한다.**
  ```bash
  cd ~/jw_ws/src/fr3_husky && git pull
  cd ~/jw_ws && colcon build --symlink-install --packages-select fr3_husky_controller fr3_husky_nn_policy
  ```
- 빌드 캐시(`~/jw_ws/build/<패키지>`)를 지웠거나 `--cmake-clean-cache`를 쓸 때는 11-05에 정한 옵션을 꼭 다시 준다. 빠지면 pinocchio 3.4(libfranka, /usr/local)와 4.1.0(/opt/openrobots)이 섞여 컨트롤러가 configure에서 죽는다(`free(): invalid pointer`, `model.hxx ... addJoint` assertion).
  ```bash
  L="-Wl,--no-as-needed -Wl,--disable-new-dtags -Wl,--version-script=$HOME/jw_ws/cmake/hide_pinocchio.map"
  colcon build --symlink-install --packages-select fr3_husky_controller --cmake-clean-cache --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio \
    -DCMAKE_CXX_FLAGS=-UPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION \
    "-DCMAKE_SHARED_LINKER_FLAGS=$L" "-DCMAKE_MODULE_LINKER_FLAGS=$L" -DCMAKE_INSTALL_RPATH=/opt/openrobots/lib
  ```
- franka_hardware를 고쳤을 때만 따로 빌드한다(jh_ws의 같은 패키지를 덮어쓰므로 `--allow-overriding`이 필요하다).
  ```bash
  cd ~/jw_ws && colcon build --symlink-install --packages-select franka_hardware --allow-overriding franka_hardware
  ```

## 1. 세션 준비

- **Desk의 EE 설정을 지금 달린 것과 맞춘다.** hand가 달려 있으면 hand(+판, 볼트, 패드) 전체 질량과 무게중심을 넣는다.
  - 컨트롤러의 중력·질량 모델은 URDF가 아니라 Desk 설정을 쓴다.
  - 틀리면 팔이 처지거나, reflex가 나거나, 측정값이 치우친다.
- EE 구성과 질량을 `NOTES`에 적는다. sim 보정할 때 같은 EE로 맞춘다.
- 판 아래와 주변에 10 cm 이상 공간이 있는지 본다.
- 비상정지를 손에 둔다.

## 시험 하나를 돌리는 방법 (모든 시험 공통)

```
(1) T3: rec <label>                → bag 시작
(2) T4: probe <인자들>              → 설정 로그에 robot=right, dry_run=..., joint_states=/right_fr3/joint_states 확인
(3) T2: pstart                     → 팔이 멈춰 있으면 q0를 잡고 시작
(4) T4 로그:  q0 latched: [...]  →  [1/N] <trial 이름> ...  →  all trials done; q0 is stable
(5) T4 Ctrl+C,  T3 Ctrl+C
```
- 중간에 멈출 때는 T2에서 `pstop`을 한다. 급하면 비상정지를 누른다.
- 결과는 `~/jw_ws/src/fr3_husky/data/command_response/real/<YYMMDD_HHMMSS>_right_<label>/`에 쌓인다. 안에 `metadata.json`과 trial별 CSV가 있다.
- dry run은 bag을 켜지 않아도 된다.

## 2. 토크 모드

**2-1. T1: 컨트롤러**
```bash
ros2 launch fr3_husky_controller fr3_action_controller.launch.py \
  robot_side:=right load_gripper:=false command_interface:=effort
```
- 로그에 아래 두 줄이 나와야 한다.
  - `Configured and activated right_fr3_action_controller`
  - `Configured and activated right_franka_robot_state_broadcaster`
- RViz에 손목 카메라(RealSense)와 Husky 위 카메라(Azure Kinect)가 보이는 것은 정상이다. 한 팔 URDF가 항상 넣는다.

**2-2. T2: 충돌 임계값** (T1을 새로 띄울 때마다 넣는다)
```bash
ros2 service call /service_server/set_force_torque_collision_behavior franka_msgs/srv/SetForceTorqueCollisionBehavior \
  "{lower_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], upper_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], \
    lower_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0], upper_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]}"
```
- 응답이 `success: True`인지 보고, 시각을 `NOTES`에 적는다.
- 서비스가 없다고 나오면 `ros2 service list | grep collision`으로 이름을 찾는다.

**2-3. T2: 시작 자세**
```bash
ready
```
- 끝날 때까지 기다린다. 처음 이동할 때는 j1이 0으로 도는 경로에 걸리는 게 없는지 눈으로 본다.

**2-4. 시험** (위의 "시험 하나를 돌리는 방법"대로 한다)

| 순서 | T4 명령 | 시간 | 확인 |
|---|---|---|---|
| A0 dry run | `probe command_interface:=effort label:=torque_dry "$NOTES"` | 몇 초 | `q0 latched`가 ready 값과 같고 `[1/21]`이 찍히면 끝이다. 로봇은 안 움직인다. `pstop`으로 멈추고 Ctrl+C |
| A1 첫 동작 | `probe command_interface:=effort dry_run:=false label:=torque_step_j4 tests:="[step]" joints:="[4]" "$NOTES"` | 약 20 s | j4가 계단 6개를 따라가고 q0로 돌아오는지, 튐이나 reflex가 없는지 |
| A2 전체 sweep | `probe command_interface:=effort dry_run:=false label:=torque_default "$NOTES"` | 약 10분 | trial 21개: j1~j7 step → j1~j7 sine → j1~j7 chirp |
| A3 정지 | 아래 "정지 시험" | 자세당 약 10 s | |

- A0의 dry run은 trial 시간을 그대로 흘려보내서, 끝까지 두면 10분이 걸린다. 한계 검사는 `q0 latched` 전에 trial 21개 전부에 대해 끝나므로, `[1/21]`까지만 보면 된다.
- dry run 결과 폴더(`..._right_torque_dry`)는 지워도 된다.

**정지 시험** (A3, B4 공통)

자세마다 T2에서 옮긴 뒤 probe를 다시 띄운다. 세 자세 모두 정면(y = 0)이고 판이 아래를 향한다. 판 끝은 장착면보다 40 cm 이상 위에 있다. 위치 모드(B4)에서는 자세를 옮길 수 없으므로 3-4의 "B4 자세 옮기기"를 따른다.

| 자세 | T2: 옮기기 | 판 끝 (앞 / 위) |
|---|---|---|
| H1 ready | `ready` | 31 / 49 cm |
| H2 앞으로 | `ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.0 -0.3 0.0 -2.0 0.0 1.7 0.7854` | 46 / 48 cm |
| H3 더 앞으로 | `ros2 run fr3_husky_task_manager move_to_joint --arm right --right-target-positions 0.0 0.0 0.0 -1.9 0.0 1.9 0.7854` | 56 / 40 cm |

```bash
# T4 (토크 모드). <자세>는 H1, H2, H3
probe command_interface:=effort dry_run:=false label:=torque_hold_<자세> tests:="[hold]" "$NOTES"
```
- 위치 모드로 넘어가기 전에 `ready`로 돌아온다.

## 3. 위치 모드

**B1과 B2를 통과해야 B3로 넘어간다.**

**3-0. 11-17 B3 reflex 뒤 바뀐 것**

11-17 B3(chirp j2)에서 로봇이 0.38 s 동안 명령을 받지 않았다. 그동안 추종기는 혼자 앞서 나갔고, 로봇이 다시 받을 때 명령이 10~25 mrad 튀어 reflex가 났다.

| 항목 | 11-17 | 지금 |
|---|---|---|
| 목표 사이 | 새 목표에 가서 멈췄다가 다음 목표를 기다림 | 이전 목표에서 새 목표까지 50 ms 동안 직선으로 잇고, 그 속도도 같이 줌. wp 50 → 20 |
| 기준 | 추종기 자기 상태 | 매 1 ms 로봇이 실제로 실행한 명령(q_d)에서 다음 명령을 계산. 로봇이 명령을 안 받으면 명령도 같이 멈춰서, 다시 받을 때 튀지 않는다 |
| reflex가 나면 | 알아채지 못하고 계속 정책을 따라감 | goal이 `robot reflex or control restart (...)`로 끝나고 팔은 그 자리에 멈춘다. 다시 움직이지 않는다(토크 모드도 같음) |
| 자세 이동 | move_to_joint를 위치 모드에서도 부를 수 있었음(시작 3 ms 만에 reflex) | 위치 모드에서 move_to_joint, JTC는 거부된다. 자세 이동은 토크 모드에서만 한다 |
| 기록 | 정책 목표(20 Hz)와 로봇 상태만 | 컨트롤러가 1 ms마다 쓴 명령, q_d, 기준점, 로봇 시각 등을 `policy_position_debug`로 발행. T3 `rec`에 들어 있다 |

- 설정은 `fr3_ros_controllers.yaml`의 `right_fr3_position_action_controller` → `policy_position_tracker`에 있다.
- 11-17 동작으로 되돌리려면 `wp: 50.0`, `interpolate: false`, `sync_to_robot: false`로 바꾸고 T1을 다시 띄운다.

**3-0-2. 듀얼 위치 모드를 위해 더 바뀐 것 (11-17 저녁, 로봇 PC)**

듀얼 하드웨어(`FrankaMultiHardwareInterface`)는 두 팔을 1 ms 안에 처리하려고 "상태 읽기 → 지난 tick에 계산한 명령 전송" 순서로 돈다. 그래서 q_d에는 한팔보다 한 tick 늦게, 2 tick 전 명령이 보인다. 위 "기준" 항목을 그대로 쓰면 듀얼에서는 매 tick 잘못 다시 맞추다가 발산한다(시뮬레이션).

| 항목 | 내용 |
|---|---|
| q_d 비교 | q_d를 `command_lag_ticks` tick 전 명령과 비교한다. yaml에서 한팔 블록은 1, 듀얼 블록은 2. 한팔 동작은 그대로다 |
| limitRate | jw_ws의 franka_hardware가 위치 명령을 보내기 직전, 방금 읽은 q_d, dq_d, ddq_d 기준으로 libfranka limitRate를 건다. lowpass는 끈다. 기본으로 켜져 있고 T1 `position_rate_limit:=false`로 끈다. 평상시에는 명령을 자르지 않는다(추종기가 한계의 0.8배 안에서 움직임). 로봇이 멈췄다가 다시 받는 순간만 자른다 |
| 로봇 시각 멈춤 | 로봇 시각이 100 ms 넘게 그대로면(하드웨어 자동 복구가 실패해 같은 상태만 돌려줌) goal을 `robot time has not advanced for N ms`로 끝낸다 |
| goal 끝 로그 | q_d가 c_{k-1}, c_{k-2} 중 어느 쪽과 맞았는지 횟수를 남긴다. `command_lag_ticks`가 하드웨어와 다르면 경고한다 |
| debug 토픽 flags | 32: q_d = c_{k-1}, 64: q_d = c_{k-2} (움직이는 tick에서만) |

- 듀얼에서는 둘 다 필요하다. 보냈지만 아직 q_d에 안 보이는 명령이 항상 하나 있어서, 로봇이 멈췄다 다시 받는 순간은 컨트롤러가 맞출 수 없다. 380 ms 멈춤 시뮬레이션에서 재개 순간 가속도는 limitRate 없이 1260 rad/s², 켜면 10 rad/s²(libfranka 한계에서 자름)였다.
- `command_lag_ticks`를 한팔에서 2로 잘못 넣으면 매 tick 다시 맞추지만 자기 상태로 맞추는 것이라 움직임은 같다(goal 끝에 경고). 듀얼에서 1로 넣으면 발산하므로 듀얼 블록은 반드시 2다.
- 듀얼 위치 모드 실물은 이 두 가지가 다 들어간 빌드에서만 한다. 듀얼 T1: `robot_side:=dual command_interface:=position`.

**3-1. T1을 끄고 다시 띄운다**
```bash
ros2 launch fr3_husky_controller fr3_action_controller.launch.py \
  robot_side:=right load_gripper:=false command_interface:=position
```
- 로그에 `Configured and activated right_fr3_position_action_controller`가 나와야 한다.
- 띄울 때 T1에 `joint position command limitRate: on (FRANKA_JOINT_POSITION_RATE_LIMIT=1), low-pass filter: off`가 나와야 한다. 이 줄이 없으면 jh_ws의 franka_hardware가 뜬 것이다(0절 source 순서와 `ros2 pkg prefix franka_hardware`).
- probe가 첫 goal을 보내면(B1부터) T1에 아래 내용의 두 줄이 나와야 한다.
  ```
  position tracker wp=20.0/s limit_fraction=0.80 brake_fraction=0.70 reference=linear 50 ms sync_to_robot_q_d=on command_lag_ticks=1 debug_topic=on
  robot state monitor (reflex / control restart / stalled robot time stops the policy): on
  ```
  - 실물에서 `unavailable (sim)`이 나오면 로봇 상태를 못 읽은 것이다. 멈추고 T1 로그에서 `Franka robot state ... not available`을 찾는다.
- goal이 끝날 때마다 T1에 두 줄이 나온다.
  - `q_d sync: N re-syncs, longest run M ms`: **0이 정상이다.**
  - `q_d lag check (command_lag_ticks=1): ... q_d = c_{k-1} in A, c_{k-2} in B, neither in C`: 한팔은 A가 거의 전부여야 한다(듀얼은 B).
- 위치 컨트롤러가 내려갈 때(T1 종료 등) `joint position limitRate (position interface stopped): 0 of N commands cut`이 나온다. 0이 정상이다. 종료 중이라 줄이 안 찍힐 수도 있는데, 잘린 적이 있으면 그때마다 1초에 한 번 경고가 이미 나온다.
- 예외: T1을 띄운 직후 `limitRate (cutting commands): 1 of 1 commands cut ... max change 0.000001 rad` 정도가 한 번 나오는 것은 정상이다(11-17 실물). 하드웨어가 첫 위치 명령을 측정값으로 채워서 q_d와 µrad 단위로 다른 것을 맞춘 것이다. 이후 summary가 `1 of N`이면 그 한 번뿐이다.

**3-2. T2: 충돌 임계값을 다시 넣는다** (2-2와 같은 명령)

**3-3. 시작 자세: 따로 하지 않는다**
- 2-4 끝에서 토크 모드로 이미 ready에 와 있다.
- 위치 모드에서는 `ready`(move_to_joint)가 `Reject: not available on the position command interface`로 거부된다.

**3-4. 시험**

| 순서 | T4 명령 | 시간 | 확인 |
|---|---|---|---|
| B0 dry run | `probe command_interface:=position label:=position_dry "$NOTES"` | 몇 초 | A0와 같다 |
| B1 유지 | `probe command_interface:=position dry_run:=false label:=position_hold tests:="[hold]" hold_duration_s:=10.0 "$NOTES"` | 약 10 s | 시작과 끝에서 튐이나 reflex가 없는지. T1 로그 두 줄(3-1)과 `q_d sync: 0 re-syncs`. 팔이 안 움직이므로 `q_d lag check`의 숫자는 모두 0이어도 정상이다 |
| B2 작은 계단 | `probe command_interface:=position dry_run:=false label:=position_step_small tests:="[step]" step_amplitudes_rad:="[0.005]" joints:="[4]" "$NOTES"` | 약 7 s | 0.005 rad 계단을 매끄럽게 따라가는지. `q_d sync: 0 re-syncs`. 여기서 처음 `q_d lag check`를 본다: c_{k-1}(A)이 거의 전부, c_{k-2}(B)와 neither(C)는 0에 가까워야 한다 |
| B3 전체 sweep | `probe command_interface:=position dry_run:=false label:=position_default "$NOTES"` | 약 10분 | A2와 같은 trial 21개. 11-17에 멈춘 chirp j2(화면 `[16/21]`, 파일 `trial15_...`)를 지나는지 |
| B4 정지 | H1: 지금 자세(ready)에서 `probe command_interface:=position dry_run:=false label:=position_hold_H1 tests:="[hold]" "$NOTES"`. H2, H3: 표 아래 "B4 자세 옮기기" | 자세당 약 10 s, H2·H3는 재시작 포함 몇 분 | |

- T1과 T4의 `command_interface`가 다르면 probe가 바로 멈춘다. 두 터미널의 모드를 맞춘다.
- `q_d sync`가 0이 아니면 [문제 생길 때](#문제-생길-때)를 본다.

**B4 자세 옮기기 (H2, H3)**

위치 모드에서는 자세를 옮길 수 없다. 자세마다 아래를 한다.
1. T1 Ctrl+C → 2-1 명령(`command_interface:=effort`)으로 다시 띄운다 → 2-2 임계값
2. T2: 정지 시험 표의 H2(또는 H3) `move_to_joint`
3. T1 Ctrl+C → 3-1 명령(`command_interface:=position`)으로 다시 띄운다 → 2-2 임계값
4. T3 `rec position_hold_<자세>`, T4 `probe command_interface:=position dry_run:=false label:=position_hold_<자세> tests:="[hold]" "$NOTES"`, T2 `pstart`
- 끝나면 1~2를 반복해 토크 모드로 `ready`에 돌아온다.

## 4. 시간이 남으면

- **접촉 강성(AB-c)**
  - 판을 고정된 물체에 닿기 직전에 두고, 그 자세에서 probe를 띄운다.
  - 명령: `tests:="[press]" joints:="[<관절 하나>]" press_direction:=1.0`. 반대 방향이면 `-1.0`이다.
  - 0.002, 0.005, 0.01 rad 순서로 누른다. 시작 토크보다 10 Nm 커지면 q0로 물러난다.
  - 위치 모드는 아주 단단하므로 이 기본값보다 키우지 않는다.
- 비전 지연(C2)과 패드 마찰은 박스, 카메라 세팅이 필요해서 이번 장착에서는 하지 않는다.

## 5. 결과 가져오기

- 아래 두 가지를 lab PC로 옮긴다.
  - `~/jw_ws/src/fr3_husky/data/command_response/real/` 아래 run 폴더 전부(dry run 폴더는 빼도 된다)
  - `~/jw_ws/bags/cmdresp_*`
- 분석은 lab PC에서 한다.
  ```bash
  python scripts/tools/analyze_command_response.py <run>... --plot <dir> --json <file>
  ```
  - 토크 모드와 위치 모드 run을 같이 넣으면 같은 표와 그림으로 비교한다.
- 위치 모드 bag의 1 kHz 진단 기록은 CSV로 바꿔 본다(lab).
  ```bash
  source /opt/ros/jazzy/setup.bash
  python3 scripts/tools/export_position_debug.py <bag 폴더>... --out <dir>
  ```
  - goal마다 CSV 하나와 `summary.json`이 나온다.
  - `q_d frozen`: 로봇이 명령을 실행하지 않은 시간. 0이 정상이다(11-17 B3는 380 ms).
  - `robot time steps != 1 ms`: 로봇 쪽 통신 끊김. 1 ms가 아닌 큰 값은 자동 복구 재시작이다.
  - `robot-side |a| max`: 로봇이 보는 명령 가속도. FR3 한계 10 rad/s² 아래여야 한다(추종기는 8까지 씀).

## 문제 생길 때

| 증상 | 원인과 조치 |
|---|---|
| `package 'franka_description' not found` | jh_ws underlay를 source하지 않았다. [0. 터미널 준비](#0-터미널-준비) |
| `right_franka_robot_state_broadcaster` 실패, `right_single_fr3/robot_state does not exist` | jh_ws를 jw_ws보다 나중에 source해서 예전 패키지가 떴다. 새 터미널에서 순서대로 다시 source한다 |
| `QPIK_weight.joint.velocity.manipulator expected 7 values, got 14` | 컨트롤러 yaml 수정이 빠졌다. `fr3_ros_controllers.yaml`의 `right_fr3_action_controller`, `right_fr3_position_action_controller`에서 이 값이 7개여야 한다 |
| `timed out waiting for a stationary start pose` | 팔이 멈추지 않았다. move_to_joint가 끝났는지 본다 |
| `... would leave the safe band` (한계 검사 실패) | q0에서 trial 범위가 관절 한계 − 0.02 rad를 넘는다. 자세가 ready인지 본다 |
| `fr3_policy_control action server is not ready` | T1 컨트롤러가 active가 아니다. T1 로그를 본다 |
| reflex로 멈춤 | 아래 명령으로 reflex 종류를 본다. Desk EE 설정과 충돌 임계값(2-2)을 다시 확인한다 |
| T4 `controller ended the goal during the run: success=False, 'robot reflex or control restart (...)'` | reflex 또는 franka_hardware 자동 복구가 났다. 팔은 그 자리에 멈춰 있다. 괄호 안 이유를 적고, 아래 명령으로 reflex 종류를 본다. bag은 5절 `export_position_debug.py`로 본다. 다시 하려면 토크 모드로 `ready`에 돌아간 뒤 3-1부터 한다 |
| `Reject: not available on the position command interface` | 위치 모드에서 move_to_joint나 JTC를 불렀다. 토크 모드로 다시 띄워 옮긴다(3-4 "B4 자세 옮기기") |
| T1 경고 `robot q_d has not followed the position command for 50 ms` | 로봇이 명령을 실행하지 않고 있다(11-17 B3 reflex 직전과 같은 상황). 명령은 q_d에 묶여 있어 다시 받아도 튀지 않는다. 시각을 적고, bag을 `export_position_debug.py`로 본다 |
| `q_d sync: N re-syncs`가 시험 시간(ms)과 비슷하게 큼 | q_d가 `command_lag_ticks` tick 전 명령과 맞지 않는다. 바로 아래 `q_d lag check` 줄을 본다. A(c_{k-1})나 B(c_{k-2}) 중 큰 쪽이 하드웨어의 실제 지연이다. 그 값으로 yaml `command_lag_ticks`를 고치고 T1을 다시 띄운 뒤 B1부터 한다. 둘 다 작고 C(neither)가 크면 시험을 멈추고 `sync_to_robot: false`로 띄운 뒤 bag을 lab PC로 가져온다 |
| T1 경고 `q_d followed c_{k-N} but command_lag_ticks=M` | yaml의 `command_lag_ticks`가 하드웨어와 다르다. 경고가 말하는 값으로 고치고 T1을 다시 띄운다 |
| T1 경고 `joint position limitRate (cutting commands): ...` | franka_hardware가 위치 명령을 잘랐다. 로봇이 멈췄다 다시 받는 순간이면 정상 동작이다(같은 시각에 `robot q_d has not followed` 경고가 있음). 평상시에 계속 나오면 추종기 명령이 한계에 닿은 것이다. 시각과 max change를 적고 bag을 lab PC로 가져온다 |
| T4 `... 'robot reflex or control restart (<arm>: robot time has not advanced for N ms ...)'` | franka_hardware가 자동 복구나 제어 재시작에 실패해 같은 상태만 돌려준다. T1 로그에서 `Automatic error recovery failed`나 `Restarting active control failed`를 찾는다. T1을 다시 띄운다 |
| T1에 `joint position command limitRate` 줄이 없음 | jh_ws의 franka_hardware(limitRate 없음)가 떴다. 0절 순서로 다시 source하고 `ros2 pkg prefix franka_hardware`를 확인한다. 듀얼 위치 모드는 이 상태로 하지 않는다 |

```bash
find ~/.ros/log -maxdepth 1 -name "ros2_control_node_*.log" -newermt "<시작 시각>" -exec grep -Hn "reflex" {} +
```

## 박스 재생(A4, B4)을 안 하는 이유

원래 계획에는 실물 박스 5499 성공 회차(`20261107_194728`)의 오른팔 목표를 그대로 다시 보내는 재생 시험이 있었다. 정책 추론 없이, 저장된 목표를 다시 보내는 시험이다. 지금 장착에서는 할 수 없다.

- **원래 시작 자세에서는 판이 지면에 닿는다.**
  - 재생 목표는 원래 회차 시작 자세(pregrasp 표 3715줄)에서의 변화량이다. 그래서 probe는 q0가 그 자세와 0.05 rad 안에 있어야 시작한다.
  - 그 자세에서 판은 팔 장착면보다 22 cm 아래에 있고, 재생 중에는 32 cm 아래까지 내려간다.
- **ready에서 같은 변화량을 주면 관절 한계를 넘는다.** j2는 −2.06 rad(한계 −1.84 rad)까지, j4는 −3.47 rad(한계 −3.08 rad)까지 간다.

Husky에 다시 달았을 때 하는 절차:
```bash
cd ~/jw_ws/src/fr3_husky
python3 scripts/tools/make_boxlift_replay.py data/dual_fr3_boxlift_real/20261107_194728_296023_KST \
  --arm right --out data/command_response/replay/boxlift_5499_194728_right
ros2 run fr3_husky_task_manager move_to_joint --arm right \
  --right-target-positions -0.1449 1.6350 0.1339 -1.3317 0.8948 1.5717 0.9992    # 스크립트가 출력한 시작 자세와 같은지 본다
probe command_interface:=effort dry_run:=false label:=torque_replay tests:="[replay]" \
  replay_file:=$HOME/jw_ws/src/fr3_husky/data/command_response/replay/boxlift_5499_194728_right.csv "$NOTES"
```
- 위치 모드는 `command_interface:=position label:=position_replay`로 바꾼다.
- 들기 구간에서 목표가 j2 약 1.0 rad/s, j6 약 1.2 rad/s로 움직인다. 판 아래 공간을 먼저 확인한다.
