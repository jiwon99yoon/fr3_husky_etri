# FR3 명령 응답 probe 실험 순서 (한 팔, 2026-11-17)

## 목적

같은 20 Hz 절대 관절 목표를 두 모드로 보내서, 명령에서 실제 관절 위치까지의 지연과 응답을 잰다.
- **토크 모드**: 지금 박스 정책이 쓰는 경로(PD Kp 80)
- **위치 모드**: 새로 추가한 경로

결과는 sim 액추에이터 보정에 쓴다(lab `docs/todo_1116.md` §2, §6).

## 기본 정보

- 노드는 `fr3_command_response_probe`(`fr3_husky_nn_policy`)이고, launch 파일은 `fr3_command_response_probe.launch.py`이다.
- **기본값은 dry run이다.** 로봇을 실제로 움직이려면 `dry_run:=false`를 붙여야 한다.
- 결과 폴더는 `data/command_response/real/<시각>_<label>/`이다. 안에 `metadata.json`과 trial별 CSV가 있다.
- 한 팔·한 모드에 약 15분 걸린다.

## 0. 빌드 (pull한 뒤 한 번)

```bash
cd ~/jw_ws
colcon build --symlink-install --packages-select fr3_husky_controller fr3_husky_nn_policy
source install/setup.bash
ros2 pkg executables fr3_husky_nn_policy | grep command_response    # fr3_command_response_probe가 보이면 된다
```

로봇 PC는 원래 `--symlink-install`로 빌드해 왔다. 박스 로그가 `src/.../data/`에 남는 것도 그 때문이다. 이 옵션을 빼면 결과 폴더가 install 아래로 바뀐다.

## 1. 세션 준비

- 박스는 치우고 판만 단 상태로 한다.
- **EE 질량**: 판, 볼트, 패드를 저울로 함께 재서 Desk에 넣는다.
- **패드 구성**(개수, 위치)을 적어 둔다. 아래 `notes`에 넣는다. 나중에 마찰을 추정할 때 쓴다.
- 비상정지를 손에 둔다.

## 2. 터미널 구성

컨트롤러는 모드마다 한 번만 띄우고, probe는 시험마다 다시 띄운다. 이렇게 하면 충돌 임계값을 모드마다 한 번만 넣으면 된다.

**T1: 컨트롤러**
```bash
ros2 launch fr3_husky_controller fr3_action_controller.launch.py \
  robot_side:=left load_gripper:=false command_interface:=effort
```
위치 모드에서는 T1을 끄고 `command_interface:=position`으로 다시 띄운다.

**T2: 충돌 임계값** (T1을 새로 띄울 때마다 넣는다)
```bash
ros2 service call /service_server/set_force_torque_collision_behavior franka_msgs/srv/SetForceTorqueCollisionBehavior \
  "{lower_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], upper_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 80.0, 80.0], \
    lower_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0], upper_force_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]}"
```
- 캐비닛 실험과 같은 값이다(`dual_fr3_cabinet_book_experi_step.md`).
- 응답이 `success: True`인지 확인하고, 시각을 `notes`에 적는다.
- 서비스 이름이 다르면 `ros2 service list | grep collision`으로 찾는다.

**T2: 시작 자세 (박스 pregrasp, 왼팔)**
```bash
ros2 run fr3_husky_task_manager move_to_joint --arm left \
  --left-target-positions 0.0012 1.6314 -0.1775 -1.3488 -1.0002 1.6352 0.5559
```
- 판 아래와 주변에 10 cm 이상 공간이 있는지 본다. 계단 시험은 관절을 최대 0.10 rad 움직인다.
- 공간이 부족하면 더 높은 자세를 쓰고, 그 관절값을 `notes`에 적는다.

**T3: bag** (시험마다 켜고 끈다)
```bash
ros2 bag record -o ~/bags/cmdresp_$(date +%m%d_%H%M%S) \
  /left_fr3/joint_states /left_franka_robot_state_broadcaster/robot_state /policy_joint_command /rosout
```

**T4: probe**
- 띄우는 명령은 아래 3, 4절에 있다.
- 시작과 정지:
  ```bash
  ros2 service call /fr3_command_response_probe/start_probe std_srvs/srv/Trigger {}
  ros2 service call /fr3_command_response_probe/stop_probe std_srvs/srv/Trigger {}    # 중간에 멈출 때
  ```
- 동작 방식:
  - probe는 팔이 정지하면 그 자세를 q0로 잡고 시작한다.
  - trial이 끝날 때마다 q0로 돌아와 정지한 뒤 다음 trial로 넘어간다.
  - 끝나면 노드가 종료 로그를 남긴다.

T4의 공통 앞부분이다. `<…>`는 실제 값으로 바꾼다.
```bash
probe() { ros2 launch fr3_husky_nn_policy fr3_command_response_probe.launch.py robot:=left launch_controller:=false "$@"; }
NOTES="notes:=EE <질량> kg, pad <개수/위치>, collision 100/80 Nm 100 N set <시각>, pose pregrasp"
```
- 아래 표의 명령은 이 함수를 쓴다. `"$NOTES"`는 따옴표째 붙여야 한 인자로 넘어간다.

## 3. 토크 모드 (T1 `command_interface:=effort`)

| 순서 | 명령 (T4) | 확인 |
|---|---|---|
| 1. dry run | `probe command_interface:=effort label:=torque_dry "$NOTES"` → start_probe | 로그의 q0와 trial 수, 한계 검사 통과. 로봇은 안 움직인다 |
| 2. 첫 동작 | `probe command_interface:=effort dry_run:=false label:=torque_step_j4 tests:="[step]" joints:="[4]" "$NOTES"` → start_probe | j4가 계단을 따라가고 q0로 돌아오는지 |
| 3. A1~A3 | `probe command_interface:=effort dry_run:=false label:=torque_default "$NOTES"` → start_probe | 기본 `tests:=[step, sine, chirp]`, 관절 7개. 약 15분 |
| 4. A4 재생 | 아래 "A4 재생" | |
| 5. A5 정지 | `probe command_interface:=effort dry_run:=false label:=torque_hold_<자세> tests:="[hold]" "$NOTES"` | 자세 3~4개. 자세마다 move_to_joint로 옮긴 뒤 다시 띄운다 |

**A4 재생** (실물 박스 5499 성공 회차의 왼팔 목표)
```bash
cd ~/jw_ws/src/fr3_husky
python3 scripts/tools/make_boxlift_replay.py data/dual_fr3_boxlift_real/20261107_194728_296023_KST \
  --arm left --out data/command_response/replay/boxlift_5499_194728_left
```
1. 출력된 시작 자세로 move_to_joint를 한다.
2. 그다음 아래 명령을 실행한다.
   ```bash
   probe command_interface:=effort dry_run:=false label:=torque_replay tests:="[replay]" \
     replay_file:=$HOME/jw_ws/src/fr3_husky/data/command_response/replay/boxlift_5499_194728_left.csv "$NOTES"
   ```
- **주의:**
  - 들기 구간에서 j2가 약 1 rad/s로 움직인다.
  - 원래 회차에서는 판이 탁자 약 1 cm 위까지 내려갔다. 탁자를 앞으로 밀어 두거나 판 아래 공간을 먼저 확인한다.
  - 판이 탁자에 닿으면 reflex로 멈춘다.
- `replay_file`은 절대 경로로 준다. 스크립트가 출력한 `.csv` 경로와 같은지 확인한다.

## 4. 위치 모드 (T1을 `command_interface:=position`으로 다시 띄우고 임계값을 다시 넣는다)

위치 모드는 실물에서 처음 쓴다. **B0을 통과해야 다음으로 넘어간다.**

| 순서 | 명령 (T4) | 확인 |
|---|---|---|
| B0-1 유지 | `probe command_interface:=position dry_run:=false label:=position_hold tests:="[hold]" hold_duration_s:=10.0 "$NOTES"` | 시작·끝에서 튐이나 reflex가 없는지 |
| B0-2 작은 계단 | `probe command_interface:=position dry_run:=false label:=position_step_small tests:="[step]" step_amplitudes_rad:="[0.005]" joints:="[4]" "$NOTES"` | 0.005 rad 계단을 매끄럽게 따라가는지 |
| B1~B3 | `probe command_interface:=position dry_run:=false label:=position_default "$NOTES"` | 토크 모드 3번과 같은 profile |
| B4 재생 | 토크 모드 A4와 같다(`command_interface:=position`, `label:=position_replay`) | |
| B5 정지 | 토크 모드 A5와 같다 | |

- 컨트롤러 feedback의 모드가 `command_interface`와 다르면 probe가 바로 멈춘다. T1과 T4의 모드를 맞춘다.

## 5. 시간이 남으면

- **AB-c 접촉 강성**
  - 판을 고정된 물체에 닿기 직전에 둔다.
  - `tests:="[press]" joints:="[<관절 하나>]" press_direction:=1.0`으로 한다. 반대 방향이면 `-1.0`.
  - 시작 토크보다 10 Nm 커지면 q0로 물러난다.
  - 위치 모드는 아주 단단하므로 `press_levels_rad` 기본값(0.002, 0.005, 0.01 rad)부터 시작한다.
- **C2 비전 지연**: 마커 박스를 판에 고정하고 0.5~1 Hz 사인으로 움직인다. bag에 `/box_tf`도 넣는다.
- **패드 마찰**: 패드 1개와 2개로 경사 시험을 한다. 박스를 패드 위에 올리고 미끄러지는 각도를 잰다.

## 6. 결과 가져오기

- `data/command_response/real/` 아래 run 폴더 전부와 `~/bags/cmdresp_*`를 lab PC로 옮긴다.
- 분석은 lab PC에서 한다.
  ```bash
  python scripts/tools/analyze_command_response.py <run>... --plot <dir> --json <file>
  ```
  - 토크 모드와 위치 모드 run을 같이 넣으면 같은 표와 그림으로 비교한다.

## 문제 생길 때

- **`replay start pose differs from q0`**: A4 시작 자세가 맞지 않는다. move_to_joint를 다시 한다.
- **`timed out waiting for a stationary start pose`**: 팔이 멈추지 않았다. move_to_joint가 끝났는지 확인한다.
- **한계 검사 실패**: q0에서 trial 범위가 관절 한계 − 0.02 rad를 넘는다. 자세를 바꾼다.
- **reflex로 멈춘 경우**: 아래를 실행해 reflex 종류를 본다.
  ```bash
  find ~/.ros/log -maxdepth 1 -name "ros2_control_node_*.log" -newermt "<시작 시각>" -exec grep -Hn "reflex" {} +
  ```
