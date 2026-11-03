#!/usr/bin/env python3
"""Create a reproducible static gravity-validation report for dual-FR3 SysID.

The report evaluates low-velocity samples from real CSV logs with a gravity-on,
contact-disabled MuJoCo inverse-dynamics model.  Its purpose is to validate the
payload, gravity direction, and joint mapping before fitting friction or
armature.  It deliberately zeros MuJoCo passive terms so those effects remain
in the residual rather than being silently credited to the model.

Example:
  python scripts/tools/report_sysid_static_gravity.py \
    --mjcf /tmp/dual_fr3_plate_260929.xml \
    --csv 'data/sysid_raw/real/*.csv' \
    --output-dir data/sysid_validation_260929
"""

from __future__ import annotations

import argparse
import csv
import glob
import html
import json
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fit_sysid_dynamics_mujoco import JOINT_NAMES, MujocoEngine, load_log


def _load_logs(patterns: list[str]):
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(Path(item) for item in sorted(glob.glob(pattern)))
    if not paths:
        raise SystemExit("no CSV files matched")
    logs = [log for log in (load_log(path) for path in paths) if log is not None]
    if not logs:
        raise SystemExit("no usable CSV logs")
    return paths, logs


def _collect_stationary(logs, engine: MujocoEngine, qd_threshold: float):
    tau_by_joint = [[] for _ in JOINT_NAMES]
    gravity_by_joint = [[] for _ in JOINT_NAMES]
    residual_by_joint = [[] for _ in JOINT_NAMES]
    total_rows = 0

    for log in logs:
        # A sample is stationary only when every arm joint is below threshold.
        stationary = np.all(np.abs(log.qd) < qd_threshold, axis=1)
        indices = np.flatnonzero(stationary)
        if not indices.size:
            continue
        gravity = engine.gravity_torque(log.q[indices])
        total_rows += int(indices.size)
        for joint in range(len(JOINT_NAMES)):
            measured = log.tau[indices, joint]
            finite = np.isfinite(measured)
            if not finite.any():
                continue
            tau_by_joint[joint].append(measured[finite])
            gravity_by_joint[joint].append(gravity[finite, joint])
            residual_by_joint[joint].append(measured[finite] - gravity[finite, joint])

    arrays = []
    for joint in range(len(JOINT_NAMES)):
        if not residual_by_joint[joint]:
            raise SystemExit(f"no stationary torque samples for {JOINT_NAMES[joint]}")
        arrays.append(
            (
                np.concatenate(tau_by_joint[joint]),
                np.concatenate(gravity_by_joint[joint]),
                np.concatenate(residual_by_joint[joint]),
            )
        )
    return total_rows, arrays


def _summaries(arrays):
    rows = []
    for joint, (tau, gravity, residual) in enumerate(arrays):
        rows.append(
            {
                "joint": JOINT_NAMES[joint],
                "samples": int(residual.size),
                "tau_meas_mean_nm": float(np.mean(tau)),
                "gravity_mean_nm": float(np.mean(gravity)),
                "residual_mean_nm": float(np.mean(residual)),
                "residual_abs_mean_nm": float(np.mean(np.abs(residual))),
                "residual_std_nm": float(np.std(residual)),
                "residual_median_nm": float(np.median(residual)),
                "residual_p05_nm": float(np.percentile(residual, 5.0)),
                "residual_p95_nm": float(np.percentile(residual, 95.0)),
                "residual_max_abs_nm": float(np.max(np.abs(residual))),
            }
        )
    return rows


def _write_summary_csv(path: Path, rows):
    fields = list(rows[0])
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _style_axes(ax):
    ax.grid(axis="y", color="#d9d9d9", linewidth=0.7, alpha=0.8)
    ax.spines[["top", "right"]].set_visible(False)


def _plot_mean_comparison(path: Path, rows):
    names = [f"{item['joint'].split('_')[0][0].upper()}J{idx % 7 + 1}" for idx, item in enumerate(rows)]
    tau = np.array([item["tau_meas_mean_nm"] for item in rows])
    gravity = np.array([item["gravity_mean_nm"] for item in rows])
    residual = np.array([item["residual_mean_nm"] for item in rows])
    std = np.array([item["residual_std_nm"] for item in rows])
    x = np.arange(len(rows))

    fig, axes = plt.subplots(2, 1, figsize=(15, 8), sharex=True,
                             gridspec_kw={"height_ratios": [2.0, 1.0]})
    width = 0.36
    axes[0].bar(x - width / 2, tau, width, label=r"real $\tau_{meas}$", color="#1f77b4")
    axes[0].bar(x + width / 2, gravity, width, label=r"MuJoCo $g(q)$", color="#ff7f0e")
    axes[0].set_ylabel("Mean torque (Nm)")
    axes[0].set_title("Static gravity check: measured torque versus MuJoCo gravity torque")
    axes[0].legend(ncols=2, loc="upper left")
    _style_axes(axes[0])

    axes[1].bar(x, residual, color="#2ca02c")
    axes[1].errorbar(x, residual, yerr=std, fmt="none", ecolor="#1b5e20", capsize=3,
                     label="± 1 residual std")
    axes[1].axhline(0.0, color="#333333", linewidth=0.9)
    axes[1].set_ylabel(r"Mean residual (Nm)")
    axes[1].set_xticks(x, names)
    axes[1].legend(loc="upper left")
    _style_axes(axes[1])
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_residual_distribution(path: Path, arrays):
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    labels = [f"J{i}" for i in range(1, 8)]
    for arm, ax in enumerate(axes):
        residuals = [arrays[arm * 7 + joint][2] for joint in range(7)]
        box = ax.boxplot(residuals, labels=labels, showfliers=False, patch_artist=True,
                         medianprops={"color": "#222222", "linewidth": 1.3})
        for patch in box["boxes"]:
            patch.set_facecolor("#9ecae1" if arm == 0 else "#a1d99b")
        ax.axhline(0.0, color="#333333", linewidth=0.9)
        ax.set_ylabel("Residual (Nm)")
        ax.set_title("Left arm" if arm == 0 else "Right arm")
        _style_axes(ax)
    fig.suptitle("Stationary residual distribution (outliers hidden)", y=0.995)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_scatter(path: Path, arrays, rows):
    fig, axes = plt.subplots(2, 7, figsize=(18, 5.6))
    for joint, ax in enumerate(axes.ravel()):
        measured, gravity, _ = arrays[joint]
        if measured.size > 3000:
            indices = np.linspace(0, measured.size - 1, 3000, dtype=int)
            measured, gravity = measured[indices], gravity[indices]
        bound = max(0.5, float(np.max(np.abs(np.concatenate([measured, gravity])))))
        ax.scatter(gravity, measured, s=3, alpha=0.18, color="#1f77b4", rasterized=True)
        ax.plot([-bound, bound], [-bound, bound], color="#d62728", linewidth=1.0,
                label="ideal")
        ax.set_xlim(-bound, bound)
        ax.set_ylim(-bound, bound)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"{JOINT_NAMES[joint].replace('_fr3_joint', ' J')}\n"
                     f"mean r={rows[joint]['residual_mean_nm']:+.3f} Nm",
                     fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(color="#e5e5e5", linewidth=0.5)
        if joint % 7 == 0:
            ax.set_ylabel(r"real $\tau_{meas}$ (Nm)", fontsize=8)
        if joint >= 7:
            ax.set_xlabel(r"MuJoCo $g(q)$ (Nm)", fontsize=8)
    fig.suptitle("Stationary samples: real measured torque versus MuJoCo gravity torque", y=1.01)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _format_table(rows):
    header = (
        "| Joint | Samples | Mean measured torque (Nm) | Mean MuJoCo gravity (Nm) | "
        "Mean residual (Nm) | Residual std (Nm) | 5–95% residual (Nm) |\n"
        "|---|---:|---:|---:|---:|---:|---:|\n"
    )
    body = []
    for row in rows:
        body.append(
            f"| {row['joint']} | {row['samples']:,} | {row['tau_meas_mean_nm']:.3f} | "
            f"{row['gravity_mean_nm']:.3f} | {row['residual_mean_nm']:+.3f} | "
            f"{row['residual_std_nm']:.3f} | [{row['residual_p05_nm']:.3f}, "
            f"{row['residual_p95_nm']:.3f}] |"
        )
    return header + "\n".join(body)


def _write_markdown(path: Path, rows, metadata):
    worst = max(rows, key=lambda item: abs(item["residual_mean_nm"]))
    path.write_text(
        "# Dual-FR3 static gravity validation\n\n"
        "## Result\n\n"
        f"- Stationary samples: **{metadata['stationary_rows']:,}**\n"
        f"- Velocity criterion: every arm joint satisfies `abs(qd) < "
        f"{metadata['stationary_qd_rad_s']:.3f} rad/s`\n"
        f"- Largest absolute mean residual: **{worst['joint']} = "
        f"{abs(worst['residual_mean_nm']):.3f} Nm**\n"
        f"- MJCF: `{metadata['mjcf']}`\n\n"
        "## Method\n\n"
        "For each stationary real-log row, MuJoCo runs `mj_inverse` with the "
        "same 14 measured joint positions and zero joint velocity/acceleration. "
        "Gravity is set to -9.81 m/s². Contact is disabled, and `damping`, "
        "`frictionloss`, and `armature` are set to zero, so the comparison is:\n\n"
        "$$\\tau_{meas} \\stackrel{?}{\\approx} g_{MJ}(q), "
        "\\quad r=\\tau_{meas}-g_{MJ}(q).$$\n\n"
        "This validates the payload/gravity/joint-mapping part of the model. "
        "It does **not** identify friction, damping, armature, controller delay, "
        "or contact dynamics. Those remain in the residual and are handled in the "
        "next free-motion SysID stage.\n\n"
        "## Plots\n\n"
        "![Mean comparison](static_gravity_mean_comparison.png)\n\n"
        "![Residual distribution](static_gravity_residual_distribution.png)\n\n"
        "![Measured versus gravity scatter](static_gravity_scatter.png)\n\n"
        "## Joint summary\n\n"
        + _format_table(rows)
        + "\n\n## Reproduction\n\n"
        "```bash\n"
        "python scripts/tools/report_sysid_static_gravity.py \\\n"
        f"  --mjcf {metadata['mjcf']} \\\n"
        f"  --csv '{metadata['csv_pattern']}' \\\n"
        f"  --output-dir {metadata['output_dir']}\n"
        "```\n",
        encoding="utf-8",
    )


def _write_html(path: Path, rows, metadata):
    worst = max(rows, key=lambda item: abs(item["residual_mean_nm"]))
    body_rows = "\n".join(
        "<tr>"
        f"<td>{html.escape(row['joint'])}</td>"
        f"<td>{row['samples']:,}</td>"
        f"<td>{row['tau_meas_mean_nm']:.3f}</td>"
        f"<td>{row['gravity_mean_nm']:.3f}</td>"
        f"<td>{row['residual_mean_nm']:+.3f}</td>"
        f"<td>{row['residual_std_nm']:.3f}</td>"
        f"<td>[{row['residual_p05_nm']:.3f}, {row['residual_p95_nm']:.3f}]</td>"
        "</tr>"
        for row in rows
    )
    path.write_text(
        f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Dual-FR3 static gravity validation</title>
<style>
  body {{ font-family: Arial, sans-serif; color: #1f2933; margin: 0; background: #f7f9fb; }}
  main {{ max-width: 1220px; margin: 0 auto; padding: 32px 24px 48px; }}
  h1, h2 {{ color: #102a43; }}
  .card {{ background: white; border: 1px solid #d9e2ec; border-radius: 8px; padding: 20px; margin: 18px 0; }}
  .summary {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; }}
  .metric {{ background: #eef5fb; padding: 14px; border-radius: 6px; }}
  .metric strong {{ display: block; font-size: 1.4rem; color: #0b4f71; margin-top: 4px; }}
  img {{ width: 100%; height: auto; border: 1px solid #d9e2ec; margin-top: 10px; }}
  table {{ border-collapse: collapse; width: 100%; font-size: 0.9rem; }}
  th, td {{ border-bottom: 1px solid #d9e2ec; padding: 8px; text-align: right; white-space: nowrap; }}
  th:first-child, td:first-child {{ text-align: left; }}
  th {{ background: #102a43; color: white; }}
  .scroll {{ overflow-x: auto; }}
  code {{ background: #edf2f7; padding: 2px 4px; border-radius: 3px; }}
</style>
</head>
<body><main>
<h1>Dual-FR3 static gravity validation</h1>
<p>Real SysID logs versus gravity-on MuJoCo inverse dynamics. Generated {html.escape(metadata['generated_utc'])}.</p>
<section class="card summary">
  <div class="metric">Stationary samples<strong>{metadata['stationary_rows']:,}</strong></div>
  <div class="metric">Velocity threshold<strong>{metadata['stationary_qd_rad_s']:.3f} rad/s</strong></div>
  <div class="metric">Largest |mean residual|<strong>{html.escape(worst['joint'])}: {abs(worst['residual_mean_nm']):.3f} Nm</strong></div>
</section>
<section class="card"><h2>Interpretation</h2>
<p>For stationary samples, the test evaluates <code>tau_meas ≈ g_MJ(q)</code>. MuJoCo contact is disabled and passive terms are zeroed. This checks payload, gravity direction, and joint mapping. It does not validate friction, damping, armature, delay, or contact dynamics.</p>
</section>
<section class="card"><h2>Mean torque comparison</h2><img src="static_gravity_mean_comparison.png" alt="Mean measured and gravity torque comparison"></section>
<section class="card"><h2>Residual distribution</h2><img src="static_gravity_residual_distribution.png" alt="Residual distribution by joint"></section>
<section class="card"><h2>Measured torque versus gravity torque</h2><img src="static_gravity_scatter.png" alt="Stationary measured torque versus gravity scatter plots"></section>
<section class="card"><h2>Joint summary</h2><div class="scroll"><table><thead><tr>
<th>Joint</th><th>Samples</th><th>Mean measured torque (Nm)</th><th>Mean MuJoCo gravity (Nm)</th><th>Mean residual (Nm)</th><th>Residual std (Nm)</th><th>5–95% residual (Nm)</th>
</tr></thead><tbody>{body_rows}</tbody></table></div></section>
<section class="card"><h2>Reproduction inputs</h2>
<p>MJCF: <code>{html.escape(metadata['mjcf'])}</code></p>
<p>CSV pattern: <code>{html.escape(metadata['csv_pattern'])}</code></p>
<p><a href="static_gravity_summary.csv">Download joint summary CSV</a> · <a href="report.md">Open Markdown report</a> · <a href="validation_manifest.json">Open manifest</a></p>
</section>
</main></body></html>""",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mjcf", required=True, help="expanded plate-matched MJCF XML")
    parser.add_argument("--csv", required=True, nargs="+", help="real SysID CSV glob(s)")
    parser.add_argument("--output-dir", required=True, help="directory for report and plots")
    parser.add_argument("--stationary-qd", type=float, default=0.01,
                        help="all-joint stationary threshold in rad/s")
    parser.add_argument("--gravity", type=float, default=-9.81)
    args = parser.parse_args()

    paths, logs = _load_logs(args.csv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    engine = MujocoEngine(args.mjcf, zero_passive=True, gravity=args.gravity)
    stationary_rows, arrays = _collect_stationary(logs, engine, args.stationary_qd)
    rows = _summaries(arrays)
    metadata = {
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        "mjcf": str(Path(args.mjcf).resolve()),
        "csv_pattern": ", ".join(args.csv),
        "input_csvs": [str(path.resolve()) for path in paths],
        "output_dir": str(output_dir.resolve()),
        "stationary_qd_rad_s": args.stationary_qd,
        "gravity_m_s2": args.gravity,
        "stationary_rows": stationary_rows,
        "passive_terms": "zeroed: dof_damping, dof_frictionloss, dof_armature",
        "contact": "disabled for mj_inverse",
    }

    _write_summary_csv(output_dir / "static_gravity_summary.csv", rows)
    _plot_mean_comparison(output_dir / "static_gravity_mean_comparison.png", rows)
    _plot_residual_distribution(output_dir / "static_gravity_residual_distribution.png", arrays)
    _plot_scatter(output_dir / "static_gravity_scatter.png", arrays, rows)
    _write_markdown(output_dir / "report.md", rows, metadata)
    _write_html(output_dir / "report.html", rows, metadata)
    (output_dir / "validation_manifest.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    worst = max(rows, key=lambda item: abs(item["residual_mean_nm"]))
    print(f"wrote static gravity validation report: {output_dir}")
    print(f"stationary rows: {stationary_rows:,}")
    print(f"worst |mean residual|: {worst['joint']} = {abs(worst['residual_mean_nm']):.3f} Nm")


if __name__ == "__main__":
    main()
