> 원본: `dual_fr3_lab/docs/dual_fr3_box_real_setup.md` (2026-11-03 복사). `scripts/tools/...` 경로는 dual_fr3_lab 기준이다. 실험 실행 순서는 [dual_fr3_box_experi_step.md](dual_fr3_box_experi_step.md).

# box410 실로봇 배치 기준 (2026-11-03)

box lift 정책이 학습한 박스 위치를 실로봇에서 재현하기 위한 좌표 기준이다. 대상 task는 `Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Box410-v0`와 `-Light-Pregrasp-v0`(둘 다 실물 2차 박스)다.

그림은 `scripts/tools/draw_box_workspace.py`로 sim 화면 위에 좌표를 투영해 그렸다. 로봇은 ready 자세이고, 박스는 학습 범위 한가운데(x 0.485, y 0, yaw 0)에 놓았다.

```bash
python scripts/tools/draw_box_workspace.py --viz none \
    --task Isaac-Lift-Box-DualFR3-Plate-Sim2Real-100Hz-Light-Box410-Play-v0 --out_dir docs/figures --prefix box410_workspace
```

| 위 (+x 위쪽, 로봇 왼쪽 = 화면 왼쪽) | 옆 (로봇 오른쪽에서 +y 방향으로) | 앞 (-x 방향으로, 로봇 왼쪽 = 화면 오른쪽) |
|---|---|---|
| ![top](figures/box410_workspace_top.png) | ![side](figures/box410_workspace_side.png) | ![front](figures/box410_workspace_front.png) |

- 화살표: x 빨강, y 초록, z 파랑, 길이 0.25 m. 흰 점 O가 원점, O'는 O를 바닥에 내린 점이다(위에서 보면 O와 겹친다).
- 주황 점선: 박스 **중심** 학습 범위. 노랑: x·y·yaw 범위 전체에서 박스 **밑면**이 차지할 수 있는 영역. 하늘색: 범위 한가운데, yaw 0일 때 밑면.

## 좌표계

- 원점 O는 로봇 root frame `base`다. 팔 장착 상판 윗면에 있다.
  - 두 팔의 장착점(link0 원점)이 (0, ±0.215, 0)이다. 즉 **O는 두 팔 장착점을 이은 선의 중점**이다.
- 축: x 로봇 정면, y 로봇 왼쪽, z 위.
- 정책의 박스 pose 관측과 goal은 모두 이 frame 기준이다. 실로봇 노드(`fr3_husky_nn_policy.boxlift_core`)도 같은 `base` frame을 쓴다.
- 바닥은 z = −0.405 m다(sim USD에서 팔 장착면이 바닥 위 0.405 m). **실물에서 한 번 재서 확인해야 한다.**
- Husky 몸체 가장 앞부분은 x = +0.109 m다.

## 박스 학습 범위

| 항목 | 값 [m] (base frame) |
|---|---|
| 박스 크기 | x 0.410 (앞뒤) × y 0.310 (판이 잡는 방향) × z 0.280 |
| 박스 중심 x | 0.450 ~ 0.520 |
| 박스 중심 y | −0.100 ~ +0.100 |
| 박스 yaw | −5.7° ~ +5.7° (±0.1 rad) |
| 박스 밑면 z | −0.405 (바닥), 중심 z −0.265 |
| 로봇 쪽 면(밑면 뒷변) x, yaw 0 | 0.245 ~ 0.315 (Husky 앞에서 13.6 ~ 20.6 cm) |
| 먼 쪽 면 x, yaw 0 | 0.655 ~ 0.725 |
| 밑면이 차지할 수 있는 영역 (yaw 포함) | x 0.231 ~ 0.739, y −0.275 ~ +0.275 |
| goal (박스 중심) | x·y는 시작 위치 그대로, z +0.15 ~ +0.35 (41.5 ~ 61.5 cm 들기) |

- Pregrasp는 같은 범위의 표에서 시작 자세를 뽑고, 리셋 때 박스에 ±1 cm, ±2.9°(0.05 rad)를 더 흔든다.
- 판은 140 × 140 mm이고, Pregrasp는 판 중심이 박스 중심보다 5 cm 높은 곳(바닥에서 19 cm)을 잡는다.

## 실물에 놓는 방법

1. O' 표시: 두 팔 장착점의 중점에서 바닥으로 수선을 내려 바닥에 표시한다.
2. 로봇 중심선 표시: O'에서 로봇 정면(x) 방향으로 선을 긋는다.
3. 박스 배치: 박스의 410 mm 변을 이 선과 나란하게 놓는다.
   - 로봇 쪽 면이 O'에서 0.245 ~ 0.315 m 앞에 오게 한다. 이때 박스 중심 x는 0.45 ~ 0.52다.
   - 박스 중심이 중심선에서 좌우 ±0.10 m 안에 오게 한다.
4. 처음 시험은 범위 한가운데에 둔다: 로봇 쪽 면이 O'에서 0.28 m 앞(중심 x 0.485), 중심선 위(y 0), yaw 0.
5. 비전/모캡 박스 pose가 이 값과 맞는지 확인하고 시작한다.
   - 박스 frame 원점은 박스 중심이다. x는 410 mm 변, y는 310 mm 변 방향이다.
   - 노드의 `start_position_margin_m` 검사는 이 범위를 기준으로 한다.

- x 0.50 이상을 쓰지 않아도 된다. 학습 범위 0.45 ~ 0.52의 앞쪽 대부분이 0.50 아래다.
- Pregrasp 시작 자세 표도 x 0.45 ~ 0.50에서 99%가 유효하다. 0.50 ~ 0.52는 92%다.

## Pregrasp model_2999 실물 시험 (2026-11-03)

- 정책: `fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999.npz` (`export_boxlift_actor.py`, `manifest["start"]` = `pregrasp_table`).
- 시작 자세: `fr3_husky_etri/model/dual_fr3_boxlift_pregrasp_box410_model2999_start_poses.yaml`.
  - `scripts/tools/pregrasp_start_poses.py`로 만들었다. 학습 표에서 요청한 박스 위치에 가장 가까운 줄을 골라, 그 줄의 박스 pose와 양팔 관절각, `move_to_joint` 명령을 적는다.

| 요청 박스 (x, y, yaw) | 표 줄 | 실제 박스 중심 (x, y, yaw) | 로봇 쪽 면 x | 관절 여유 (Margin 한계까지) |
|---|---|---|---|---|
| 0.46, 0.00, 0° | 1919 | 0.460, −0.001, 0.0° | 0.255 | 0.106 rad |
| 0.48, 0.00, 0° | 3788 | 0.477, +0.000, 0.0° | 0.272 | 0.083 rad |
| 0.50, 0.00, 0° | 3925 | 0.498, −0.003, −0.3° | 0.293 | 0.052 rad |
| 0.48, +0.05, 0° | 720 | 0.479, +0.052, 0.0° | 0.274 | 0.066 rad |
| 0.48, −0.05, 0° | 2228 | 0.481, −0.051, +0.3° | 0.276 | 0.061 rad |

- sim 확인: 이 다섯 줄에서 노이즈 없이 시작해 500 env를 돌렸다(질량·마찰 DR, 외란 켬).
  - 성공 100%, 성공까지 1.40 s, 끝 기울기 4.7°, 박스 yaw 0.4°, 최대 판 힘 16 N, 판 외 접촉 0, FR3 한계 여유 최소 0.115 rad.
- 순서
  1. 박스 없이 `move_to_joint`로 양팔을 그 줄의 관절각으로 보낸다. 이동은 RL과 무관하다.
     - `MoveToJoint`는 박스를 모르므로, 박스를 먼저 두면 판이 박스를 지나갈 수 있다.
  2. 박스를 표의 위치에 놓는다. 판이 박스 면에서 4 cm 바깥이므로, 로봇 앞쪽(+x)에서 두 판 사이로 밀어 넣는다.
     - 학습 노이즈가 ±1 cm, ±3°이므로 그 안에 둔다.
  3. RL 노드를 시작한다. 박스 pose(`/box_pose`)는 들기 동안에도 계속 들어와야 한다. 정책 관측에 박스 위치·자세가 매 step 들어간다.
- 노드 설정
  - 시작 검사(2026-11-03 노드 수정): 매니페스트 `start.mode`가 `pregrasp_table`이면, 노드가 npz 안의 표에서 팔(관절마다 ±0.03 rad)과 박스(x·y ±1 cm, yaw ±0.05 rad)가 같은 줄에 맞는지 본다. `ready_tolerance_rad`는 쓰지 않으므로 바꿀 필요가 없다.
  - 에피소드는 4 s로 학습했다.
  - goal 높이는 학습 범위 0.15~0.35의 최솟값 0.15(기본값)로 시작한다.
- 이 checkpoint는 기존 표(판이 면 안에서 18° 돌아감)로 학습했다. 시작 자세도 기존 표에서 뽑아야 한다. 새 표(`--roll_align`)의 관절각은 이 정책에 쓰지 않는다.

![old vs new start](figures/pregrasp_start_old_vs_rollalign.png)
