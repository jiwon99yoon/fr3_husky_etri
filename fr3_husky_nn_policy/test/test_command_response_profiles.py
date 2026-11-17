"""Offline checks of the command-response probe profiles (no ROS needed)."""

import json
import math

import numpy as np
import pytest

from fr3_husky_nn_policy.command_response_profiles import (
    ProfileConfig,
    build_trials,
    chirp_segments,
    load_replay,
    sine_amplitude,
    sine_segments,
    step_segments,
    Trial,
)

NAMES = [f"left_fr3_joint{i}" for i in range(1, 8)]


def _trial(segments, joint=1):
    return Trial("t", "x", joint, 7, segments)


def test_steps_order_and_levels():
    trial = _trial(step_segments([0.05, 0.02], hold_s=1.0, rest_s=0.5, pre_s=1.0))
    labels = [s.label for s in trial.segments]
    assert labels == ["pre", "step+0.020", "rest", "step-0.020", "rest",
                      "step+0.050", "rest", "step-0.050", "rest"]
    assert trial.duration == pytest.approx(1.0 + 4 * 1.5)
    delta, phase = trial.evaluate(1.2)
    assert phase == "step+0.020" and delta[1] == pytest.approx(0.02) and np.count_nonzero(delta) == 1
    assert trial.evaluate(1.0 + 1.0 + 0.1)[1] == "rest"
    assert trial.evaluate(trial.duration + 0.01)[1] == "return"


def test_sine_amplitude_respects_speed_cap_and_is_continuous():
    assert sine_amplitude(3.0, 0.05, 0.5) == pytest.approx(0.5 / (2 * math.pi * 3.0))
    segments = sine_segments([0.5, 3.0], 0.05, 0.5, steady_cycles=4, ramp_cycles=1,
                             rest_s=1.0, pre_s=0.5)
    trial = _trial(segments)
    t = np.arange(0.0, trial.duration, 1e-3)
    x = np.array([trial.evaluate(s)[0][1] for s in t])
    speed = np.max(np.abs(np.diff(x))) / 1e-3
    assert speed <= trial.speed_bound() + 1e-6
    assert np.max(np.abs(x)) <= 0.05 + 1e-9
    # continuity: no jumps larger than the speed bound allows
    assert np.max(np.abs(np.diff(x))) < 1e-3
    steady = [s.label for s in segments if s.label.startswith("sine_f") and not s.label.endswith(("_in", "_out"))]
    assert steady == ["sine_f0.50", "sine_f3.00"]


def test_chirp_speed_and_frequency_range():
    segments = chirp_segments(0.2, 5.0, 30.0, 0.03, 0.5, 1.0, 1.0, 1.0)
    trial = _trial(segments)
    t = np.arange(0.0, trial.duration, 5e-4)
    x = np.array([trial.evaluate(s)[0][1] for s in t])
    assert np.max(np.abs(np.diff(x))) / 5e-4 <= trial.speed_bound() + 1e-3
    assert np.max(np.abs(x)) <= 0.03 + 1e-9
    # zero crossings in the last second ~ 2 * f1
    chirp = (t > 1.0 + 29.0) & (t < 1.0 + 30.0)
    crossings = np.count_nonzero(np.diff(np.sign(x[chirp])) != 0)
    assert 8 <= crossings <= 11


def test_build_trials_order_offsets_and_bounds():
    cfg = ProfileConfig()
    trials = build_trials(["step", "hold"], [1, 7], arm_offset=0, n_joints=7, cfg=cfg, joint_names=NAMES)
    assert [t.name for t in trials] == ["step:left_fr3_joint1", "step:left_fr3_joint7", "hold"]
    assert trials[0].max_abs_delta()[0] == pytest.approx(0.10)
    assert trials[2].joint == -1 and np.all(trials[2].max_abs_delta() == 0.0)
    dual = [f"{s}_fr3_joint{i}" for s in ("left", "right") for i in range(1, 8)]
    right = build_trials(["sine"], [2], arm_offset=7, n_joints=14, cfg=cfg, joint_names=dual)
    assert right[0].joint == 8 and right[0].name == "sine:right_fr3_joint2"
    with pytest.raises(ValueError):
        build_trials(["bogus"], [1], 0, 7, cfg, NAMES)
    with pytest.raises(ValueError):
        build_trials(["replay"], [1], 0, 7, cfg, NAMES)


def test_press_levels_and_direction():
    cfg = ProfileConfig(press_levels=(0.01, 0.002), press_direction=-1.0)
    trial = build_trials(["press"], [1], 0, 7, cfg, NAMES)[0]
    levels = [s.label for s in trial.segments if s.label.startswith("press")]
    assert levels == ["press-0.0020", "press-0.0100"]


def test_replay_roundtrip(tmp_path):
    path = tmp_path / "replay.csv"
    rows = ["t_s," + ",".join(f"d_{n}" for n in NAMES)]
    for k in range(4):
        rows.append(f"{0.05 * k:.3f}," + ",".join(f"{0.01 * k * (i + 1):.4f}" for i in range(7)))
    path.write_text("\n".join(rows) + "\n")
    path.with_suffix(".json").write_text(json.dumps({"q_start": {n: 0.1 * i for i, n in enumerate(NAMES)},
                                                     "source": "unit"}))
    replay = load_replay(path)
    assert replay.dt == pytest.approx(0.05) and replay.deltas.shape == (4, 7)
    trial = build_trials(["replay"], [1], 0, 7, ProfileConfig(), NAMES, replay, NAMES[:3])[0]
    delta, phase = trial.evaluate(0.11)
    assert phase == "replay"
    assert delta[:3] == pytest.approx([0.02, 0.04, 0.06]) and np.all(delta[3:] == 0.0)
    assert trial.evaluate(0.21)[1] == "return"
