# 9/29 SysID 분석 결과 안내

2026-09-29에 실로봇과 MuJoCo에서 같은 excitation으로 얻은 데이터(`data/sysid_raw/`, 커밋 `de20751`)를 분석한 결과다. 이 문서는 결과 폴더가 무엇인지, 다시 만드는 방법을 정리한다(2026-11-03 작성).

## 결과 폴더

| 폴더 | 내용 | 만든 스크립트 |
|---|---|---|
| `sysid_validation_260929/` | 정지 상태 중력 토크 검증: 실측 τ vs MuJoCo g(q) | `scripts/tools/report_sysid_static_gravity.py` |
| `sysid_passive_fit_260929/` | 역동역학 잔차로 관절별 armature A, 건마찰 τdry, 점성 B fit | `scripts/tools/fit_sysid_passive_parameters.py` |
| `sysid_forward_validation_260929/` | fit 후보를 MuJoCo forward rollout(Kp 80, Kd 4, 중력 끔)으로 실측 위치와 비교 | `scripts/tools/validate_sysid_forward_rollout.py` |
| `sysid_timing_260929/` | 실측 vs MuJoCo 시간 지연(교차상관)과 DTW | **repo에 없음.** 만든 스크립트를 찾지 못했다 |
| `sysid_response_plots/` | 관절 위치·속도·토크 응답 그림 | `scripts/tools/plot_sysid_rich_responses.py` |

## 요약 (자세한 값은 각 `report.md`)

- passive fit: armature A는 모든 관절에서 하한 0에 걸렸다. τdry는 0.2~0.6 Nm 정도다(link 쪽 측정 토크 `tau_J` 기준).
- forward rollout: fit 후보가 현재 기본값(armature 0.1, friction 0.2, viscous 0.003)보다 위치 RMSE가 작다. 0.504° → 0.392°, held-out 0.463° → 0.349°.
- fit 값은 후보일 뿐이다. MJCF와 Isaac 학습 설정에는 적용하지 않았다(`sysid_passive_fit_260929/report.md`의 "Interpretation boundary").

## MuJoCo 모델 다시 만들기

위 결과의 report와 manifest는 모두 `/tmp/dual_fr3_plate_260929.xml`을 가리킨다. 이 파일은 재부팅하면 사라진다. 다시 만들려면:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fr3_ws/install/setup.bash     # franka_description (mjcf/)
source ~/etri_ws/install/setup.bash    # fr3_husky_description
scripts/tools/regen_sysid_mjcf_260929.sh            # -> /tmp/dual_fr3_plate_260929.xml
```

- `fr3_husky_description/mjcf/dual_fr3.xml.xacro`를 `hand:=false with_realsense:=false with_azure:=false mobile:=false`로 xacro한다.
- husky_description은 필요 없다. `mobile:=false`라 Husky 매크로가 아무것도 만들지 않으므로, 스크립트가 Husky include 두 줄을 빈 매크로 stub으로 바꾼다. husky_description에 `mjcf/`가 없는 PC가 있다.
- 2026-11-03 검증: 이렇게 만든 파일로 `report_sysid_static_gravity.py`와 `fit_sysid_passive_parameters.py`를 다시 돌리면 `static_gravity_summary.csv`, `passive_parameter_fit.csv`가 기존과 byte 단위로 같았다.
- 스크립트는 끝에 sha256을 검증값과 비교한다(xacro가 넣는 소스 경로 주석 한 줄은 빼고 계산).
  - 다르게 나오는 경우 1: franka_description이 다른 경로에 설치됐다. 파일 안에 mesh 절대경로가 들어가므로 모델이 같아도 hash가 다르다.
  - 다르게 나오는 경우 2: `fr3_husky_description/mjcf/`(특히 `plate_ee.xml.xacro`)가 `02d7201` 이후 바뀌었다. install이 symlink라 소스 수정이 바로 반영된다.

## 결과 다시 만들기

모델을 만든 뒤 repo 루트에서:

```bash
python scripts/tools/report_sysid_static_gravity.py --mjcf /tmp/dual_fr3_plate_260929.xml \
  --csv 'data/sysid_raw/real/*.csv' --output-dir data/sysid_validation_260929
python scripts/tools/fit_sysid_passive_parameters.py --mjcf /tmp/dual_fr3_plate_260929.xml \
  --csv 'data/sysid_raw/real/*.csv' --output-dir data/sysid_passive_fit_260929
python scripts/tools/validate_sysid_forward_rollout.py --mjcf /tmp/dual_fr3_plate_260929.xml \
  --fit-csv data/sysid_passive_fit_260929/passive_parameter_fit.csv --output-dir data/sysid_forward_validation_260929
python scripts/tools/plot_sysid_rich_responses.py --raw-root data/sysid_raw
```

- 이 PC에서는 `/home/dyros/anaconda3/envs/cRobotics/bin/python`(mujoco 포함)으로 돌렸다.
