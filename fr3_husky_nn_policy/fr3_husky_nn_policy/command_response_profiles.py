"""Excitation profiles for the FR3 command-response probe (pure numpy, no ROS).

Every profile is a q0-relative joint-position reference.  The probe samples it at
the policy rate (20 Hz by default) and sends the sample as an *absolute* target
that the controller holds until the next command (zero-order hold), which is the
box-lift policy path on both command interfaces:

* effort:   held target -> Isaac PD (Kp 80, Kd 4) + 1000 Nm/s rate limit
* position: held target -> 1 kHz jerk-limited tracker -> Franka joint impedance

A trial excites one joint (step / sine / chirp / press) or holds / replays the
whole arm (hold / replay).  Each trial is a list of segments; the probe returns
the arm to q0 and waits until it settles before the next trial.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

PER_JOINT_TESTS = ("step", "sine", "chirp", "press")
WHOLE_ARM_TESTS = ("hold", "replay")
ALL_TESTS = PER_JOINT_TESTS + WHOLE_ARM_TESTS


@dataclass
class Segment:
    """One labelled piece of a single-joint profile (local time 0..duration)."""

    label: str
    duration: float
    delta: Callable[[float], float]
    # Upper bound of |d delta/dt| for continuous segments (None for held levels).
    speed_bound: float = 0.0


def _const(value: float) -> Callable[[float], float]:
    return lambda _t: value


@dataclass
class Trial:
    """A q0-relative reference over ``n_joints`` command entries."""

    name: str
    kind: str
    joint: int  # active command index, or -1 for a whole-arm trial
    n_joints: int
    segments: list[Segment] = field(default_factory=list)
    # Whole-arm replay: (K, n_joints) deltas sampled every replay_dt seconds.
    replay: Optional[np.ndarray] = None
    replay_dt: float = 0.05
    replay_joints: tuple[int, ...] = ()

    @property
    def duration(self) -> float:
        if self.replay is not None:
            return self.replay.shape[0] * self.replay_dt
        return float(sum(s.duration for s in self.segments))

    def evaluate(self, t: float) -> tuple[np.ndarray, str]:
        """Return (q0-relative delta vector, phase label) at trial time t."""
        delta = np.zeros(self.n_joints, dtype=np.float64)
        if t < 0.0:
            return delta, "pre"
        if self.replay is not None:
            k = int(math.floor(t / self.replay_dt + 1e-9))
            if k >= self.replay.shape[0]:
                return delta, "return"
            return self.replay[k].copy(), "replay"
        start = 0.0
        for segment in self.segments:
            if t < start + segment.duration:
                delta[self.joint] = float(segment.delta(t - start))
                return delta, segment.label
            start += segment.duration
        return delta, "return"

    def max_abs_delta(self) -> np.ndarray:
        """Per-entry bound of |delta| (analytic for segments, exact for replay)."""
        bound = np.zeros(self.n_joints, dtype=np.float64)
        if self.replay is not None:
            return np.max(np.abs(self.replay), axis=0) if self.replay.size else bound
        if self.joint >= 0:
            peak = 0.0
            for segment in self.segments:
                samples = np.linspace(0.0, segment.duration, 2001)
                peak = max(peak, float(np.max(np.abs([segment.delta(s) for s in samples]))))
            bound[self.joint] = peak
        return bound

    def speed_bound(self) -> float:
        """Largest continuous reference speed (held levels and replay excluded)."""
        return max((s.speed_bound for s in self.segments), default=0.0)

    def segment_table(self) -> list[dict]:
        """Segment start/end times for metadata and offline analysis."""
        rows, start = [], 0.0
        for segment in self.segments:
            rows.append({"label": segment.label, "t0": start, "t1": start + segment.duration})
            start += segment.duration
        return rows


# ------------------------------------------------------------------ builders
def step_segments(
    amplitudes: Sequence[float], hold_s: float, rest_s: float, pre_s: float, scale: float = 1.0
) -> list[Segment]:
    """+a, rest, -a, rest for each amplitude (smallest first)."""
    segments = [Segment("pre", pre_s, _const(0.0))]
    for amplitude in sorted(abs(a) for a in amplitudes):
        for sign in (1.0, -1.0):
            value = sign * amplitude * scale
            segments.append(Segment(f"step{value:+.3f}", hold_s, _const(value)))
            segments.append(Segment("rest", rest_s, _const(0.0)))
    return segments


def _taper(t: float, duration: float, ramp: float) -> float:
    """Raised-cosine fade-in / fade-out envelope."""
    if ramp <= 0.0:
        return 1.0
    if t < ramp:
        return 0.5 * (1.0 - math.cos(math.pi * t / ramp))
    if t > duration - ramp:
        return 0.5 * (1.0 - math.cos(math.pi * max(duration - t, 0.0) / ramp))
    return 1.0


def sine_amplitude(frequency_hz: float, amplitude: float, max_speed: float) -> float:
    return min(amplitude, max_speed / (2.0 * math.pi * frequency_hz))


def sine_segments(
    frequencies_hz: Sequence[float],
    amplitude: float,
    max_speed: float,
    steady_cycles: int,
    ramp_cycles: int,
    rest_s: float,
    pre_s: float,
    scale: float = 1.0,
) -> list[Segment]:
    """Tapered sines.  Label suffix: _in / _out during the taper, none when steady."""
    segments = [Segment("pre", pre_s, _const(0.0))]
    for f in frequencies_hz:
        a = sine_amplitude(f, amplitude, max_speed) * scale
        period = 1.0 / f
        ramp = ramp_cycles * period
        duration = (steady_cycles + 2 * ramp_cycles) * period
        label = f"sine_f{f:.2f}"

        def fn(t: float, a=a, f=f, ramp=ramp, duration=duration) -> float:
            return a * _taper(t, duration, ramp) * math.sin(2.0 * math.pi * f * t)

        # Split into in/steady/out sub-segments so the label tells the analysis
        # which window is steady.  The three share one continuous function.
        segments.append(Segment(label + "_in", ramp, fn, speed_bound=_sine_speed(a, f, ramp)))
        steady = Segment(label, steady_cycles * period, lambda t, fn=fn, ramp=ramp: fn(t + ramp),
                         speed_bound=2.0 * math.pi * f * a)
        segments.append(steady)
        segments.append(Segment(label + "_out", ramp,
                                lambda t, fn=fn, off=ramp + steady_cycles * period: fn(t + off),
                                speed_bound=_sine_speed(a, f, ramp)))
        segments.append(Segment("rest", rest_s, _const(0.0)))
    return segments


def _sine_speed(a: float, f: float, ramp: float) -> float:
    """Bound of |d/dt (a * taper * sin)| = a*(2 pi f + pi/(2 ramp))."""
    extra = math.pi / (2.0 * ramp) if ramp > 0.0 else 0.0
    return a * (2.0 * math.pi * f + extra)


def chirp_segments(
    f0_hz: float,
    f1_hz: float,
    duration_s: float,
    amplitude: float,
    max_speed: float,
    taper_s: float,
    rest_s: float,
    pre_s: float,
    scale: float = 1.0,
) -> list[Segment]:
    """Logarithmic chirp, amplitude capped so that a(t) * 2 pi f(t) <= max_speed."""
    if not (0.0 < f0_hz < f1_hz):
        raise ValueError("chirp needs 0 < f0 < f1")
    k = f1_hz / f0_hz
    log_k = math.log(k)

    def freq(t: float) -> float:
        return f0_hz * k ** (t / duration_s)

    def phase(t: float) -> float:
        return 2.0 * math.pi * f0_hz * duration_s / log_k * (k ** (t / duration_s) - 1.0)

    def fn(t: float) -> float:
        a = sine_amplitude(freq(t), amplitude, max_speed) * scale
        return a * _taper(t, duration_s, taper_s) * math.sin(phase(t))

    # The cap keeps a*2*pi*f <= max_speed; the taper adds at most pi/(2 taper)*a0.
    bound = max_speed * scale + (
        math.pi / (2.0 * taper_s) * sine_amplitude(f0_hz, amplitude, max_speed) * scale
        if taper_s > 0.0 else 0.0
    )
    return [
        Segment("pre", pre_s, _const(0.0)),
        Segment(f"chirp_{f0_hz:g}-{f1_hz:g}Hz", duration_s, fn, speed_bound=bound),
        Segment("rest", rest_s, _const(0.0)),
    ]


def press_segments(
    levels: Sequence[float], direction: float, hold_s: float, rest_s: float, pre_s: float
) -> list[Segment]:
    """Increasing contact levels, each from q0, along ``direction`` (+1 / -1)."""
    if direction not in (1.0, -1.0):
        raise ValueError("press direction must be +1 or -1")
    segments = [Segment("pre", pre_s, _const(0.0))]
    for level in sorted(abs(v) for v in levels):
        value = direction * level
        segments.append(Segment(f"press{value:+.4f}", hold_s, _const(value)))
        segments.append(Segment("rest", rest_s, _const(0.0)))
    return segments


def hold_segments(duration_s: float) -> list[Segment]:
    return [Segment("hold", duration_s, _const(0.0))]


# -------------------------------------------------------------------- replay
@dataclass
class ReplayData:
    dt: float
    joint_names: list[str]
    deltas: np.ndarray  # (K, len(joint_names)), relative to q_start
    q_start: np.ndarray
    source: str


def load_replay(path: str | Path) -> ReplayData:
    """Read a replay CSV (columns t_s, d_<joint>...) and its JSON sidecar."""
    path = Path(path)
    with open(path, newline="") as handle:
        rows = list(csv.reader(handle))
    header, body = rows[0], rows[1:]
    if not header or header[0] != "t_s" or not all(h.startswith("d_") for h in header[1:]):
        raise ValueError(f"{path}: expected columns t_s, d_<joint>...")
    data = np.asarray([[float(v) for v in row] for row in body], dtype=np.float64)
    if data.shape[0] < 2:
        raise ValueError(f"{path}: replay needs at least two rows")
    dt = float(np.median(np.diff(data[:, 0])))
    if not np.allclose(np.diff(data[:, 0]), dt, atol=1e-6):
        raise ValueError(f"{path}: replay rows must be uniformly spaced")
    meta = json.loads(path.with_suffix(".json").read_text())
    names = [h[2:] for h in header[1:]]
    q_start = np.asarray([meta["q_start"][n] for n in names], dtype=np.float64)
    return ReplayData(dt, names, data[:, 1:], q_start, str(meta.get("source", "")))


def replay_trial(
    replay: ReplayData, command_joint_names: Sequence[str], replay_joint_names: Sequence[str]
) -> Trial:
    """Map replay columns onto the command vector; other entries stay at q0."""
    index = {name: i for i, name in enumerate(command_joint_names)}
    columns = {name: i for i, name in enumerate(replay.joint_names)}
    missing = [n for n in replay_joint_names if n not in columns or n not in index]
    if missing:
        raise ValueError(f"replay file or command vector lacks joints {missing}")
    deltas = np.zeros((replay.deltas.shape[0], len(command_joint_names)), dtype=np.float64)
    for name in replay_joint_names:
        deltas[:, index[name]] = replay.deltas[:, columns[name]]
    trial = Trial(
        name="replay", kind="replay", joint=-1, n_joints=len(command_joint_names),
        replay=deltas, replay_dt=replay.dt,
        replay_joints=tuple(index[n] for n in replay_joint_names),
    )
    return trial


# ------------------------------------------------------------------- planner
@dataclass
class ProfileConfig:
    pre_s: float = 1.0
    wrist_scale: float = 1.0
    step_amplitudes: tuple[float, ...] = (0.02, 0.05, 0.10)
    step_hold_s: float = 1.5
    step_rest_s: float = 1.5
    sine_frequencies: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)
    sine_amplitude: float = 0.05
    sine_max_speed: float = 0.5
    sine_steady_cycles: int = 4
    sine_ramp_cycles: int = 1
    sine_rest_s: float = 1.0
    chirp_f0: float = 0.2
    chirp_f1: float = 5.0
    chirp_duration_s: float = 30.0
    chirp_amplitude: float = 0.03
    chirp_max_speed: float = 0.5
    chirp_taper_s: float = 1.0
    chirp_rest_s: float = 1.0
    press_levels: tuple[float, ...] = (0.002, 0.005, 0.01)
    press_direction: float = 1.0
    press_hold_s: float = 2.0
    press_rest_s: float = 1.5
    hold_duration_s: float = 3.0


def is_wrist(joint_number: int) -> bool:
    """FR3 joints 5-7 (1-based)."""
    return joint_number >= 5


def build_trials(
    tests: Sequence[str],
    joint_numbers: Sequence[int],
    arm_offset: int,
    n_joints: int,
    cfg: ProfileConfig,
    joint_names: Sequence[str],
    replay: Optional[ReplayData] = None,
    replay_joint_names: Sequence[str] = (),
) -> list[Trial]:
    """Trials in the order of ``tests``; per-joint tests loop over ``joint_numbers``."""
    trials: list[Trial] = []
    for test in tests:
        if test not in ALL_TESTS:
            raise ValueError(f"unknown test '{test}' (choose from {ALL_TESTS})")
        if test == "hold":
            trials.append(Trial("hold", "hold", -1, n_joints, hold_segments(cfg.hold_duration_s)))
            continue
        if test == "replay":
            if replay is None:
                raise ValueError("test 'replay' needs replay_file")
            trials.append(replay_trial(replay, joint_names, replay_joint_names))
            continue
        for number in joint_numbers:
            if not 1 <= number <= 7:
                raise ValueError("joint numbers are 1..7")
            j = arm_offset + number - 1
            scale = cfg.wrist_scale if is_wrist(number) else 1.0
            if test == "step":
                segments = step_segments(cfg.step_amplitudes, cfg.step_hold_s, cfg.step_rest_s,
                                         cfg.pre_s, scale)
            elif test == "sine":
                segments = sine_segments(cfg.sine_frequencies, cfg.sine_amplitude, cfg.sine_max_speed,
                                         cfg.sine_steady_cycles, cfg.sine_ramp_cycles,
                                         cfg.sine_rest_s, cfg.pre_s, scale)
            elif test == "chirp":
                segments = chirp_segments(cfg.chirp_f0, cfg.chirp_f1, cfg.chirp_duration_s,
                                          cfg.chirp_amplitude, cfg.chirp_max_speed,
                                          cfg.chirp_taper_s, cfg.chirp_rest_s, cfg.pre_s, scale)
            else:  # press
                segments = press_segments(cfg.press_levels, cfg.press_direction, cfg.press_hold_s,
                                          cfg.press_rest_s, cfg.pre_s)
            trials.append(Trial(f"{test}:{joint_names[j]}", test, j, n_joints, segments))
    return trials
