#!/usr/bin/env python3
"""Compare range_gru under matched wheels vs. a +50% left-wheel-diameter
mismatch, focused on per-motor DDSM115 current.

Both captures run the *same* policy (range_gru) through the same automatic
test-bench sequence:

    Results/paper/range_gru_segmented.csv                      baseline
    Results/paper/range_gru_segmented_left_wheel_bigger.csv    left wheel +50%

The policy was trained with matched wheel radii and has no way to sense the
size mismatch directly -- it only ever sees wheel angular velocity/position
and the current it commanded last cycle (obs_9/obs_10). A larger left wheel
changes the odometry scale and the force-per-amp on that side, so any
compensation the policy performs shows up as an asymmetry between the two
DDSM115 current channels. That asymmetry, not just overall stability, is
the point of this script.

Output (PDF, vector), in Results/paper/wheel_asymmetry_plots/:
    stage_<name>.pdf        per test-bench stage: left current, right
                             current, and the left-right current gap
                             (baseline vs. left-wheel-bigger), plus pitch
                             for stability context
    summary_current.pdf     per-stage RMS left/right current and mean
                             current bias (left - right), both conditions
    wheel_asymmetry_metrics.csv   the numbers behind every panel

Usage:
    python Tools/plot_wheel_asymmetry.py
"""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

import plot_policy_comparison as base

PAPER_DIR = base.PAPER_DIR
OUT_DIR = PAPER_DIR / "wheel_asymmetry_plots"

RUNS = {
    "baseline": (PAPER_DIR / "range_gru_segmented.csv", "Baseline (matched wheels)"),
    "bigger": (PAPER_DIR / "range_gru_segmented_left_wheel_bigger.csv", "Left wheel +50% diameter"),
}

# categorical slot 1 (blue) for baseline, slot 7 (violet) for the perturbed
# condition -- deliberately not orange, which already means "GRU" elsewhere
# in this repo's plots, to avoid cross-plot color collisions.
COLOR_BASELINE = "#2a78d6"
COLOR_BIGGER = "#4a3aa7"
RUN_COLORS = {"baseline": COLOR_BASELINE, "bigger": COLOR_BIGGER}

COLOR_LEFT = "#2a78d6"
COLOR_RIGHT = "#eb6834"
COLOR_AXIS = base.COLOR_AXIS

STAGES = base.STAGES
STAGE_LABELS = base.STAGE_LABELS


def rms(values: list[float]) -> float:
    return base.rms(values)


def mean_sd(values: list[float]) -> tuple[float, float]:
    if len(values) < 2:
        return (values[0] if values else float("nan")), 0.0
    return statistics.mean(values), statistics.stdev(values)


# ------------------------------------------------------------ per-stage --

def compute_stage_currents(runs: dict[str, base.Run], stage_name: str):
    spans = {}
    for label, run in runs.items():
        span = run.find_stage(stage_name)
        if span is None:
            return None
        spans[label] = span

    metrics: dict[str, dict[str, float]] = {}
    for label, run in runs.items():
        i0, i1 = spans[label]
        cur_l = run.f("wheel_left_current_measured_A", i0, i1)
        cur_r = run.f("wheel_right_current_measured_A", i0, i1)
        bias = [l - r for l, r in zip(cur_l, cur_r)]
        pitch_deg = [math.degrees(v) for v in run.f("pitch_rad", i0, i1)]
        metrics[label] = {
            "duration_s": run.t[i1] - run.t[i0],
            "rms_current_left_A": rms(cur_l),
            "rms_current_right_A": rms(cur_r),
            "mean_current_bias_A": sum(bias) / len(bias),
            "rms_current_bias_A": rms(bias),
            "rms_pitch_deg": rms(pitch_deg),
            "max_abs_pitch_deg": max(abs(v) for v in pitch_deg),
        }
    return spans, metrics


def make_stage_figure(runs: dict[str, base.Run], stage_name: str, title: str, out_path: Path):
    import matplotlib.pyplot as plt

    result = compute_stage_currents(runs, stage_name)
    if result is None:
        print(f"  ! {stage_name}: missing from one or more runs, skipping")
        return None
    spans, metrics = result

    fig, (ax_l, ax_r, ax_bias, ax_pitch) = plt.subplots(
        4, 1, figsize=(6.5, 8.4), constrained_layout=True
    )

    for label, run in runs.items():
        i0, i1 = spans[label]
        t_rel = [run.t[k] - run.t[i0] for k in range(i0, i1 + 1)]
        color = RUN_COLORS[label]
        name = RUNS[label][1]

        cur_l = run.f("wheel_left_current_measured_A", i0, i1)
        cur_r = run.f("wheel_right_current_measured_A", i0, i1)
        bias = [l - r for l, r in zip(cur_l, cur_r)]
        pitch_deg = [math.degrees(v) for v in run.f("pitch_rad", i0, i1)]

        ax_l.plot(t_rel, cur_l, color=color, linewidth=1.0, label=name)
        ax_r.plot(t_rel, cur_r, color=color, linewidth=1.0, label=name)
        ax_bias.plot(t_rel, bias, color=color, linewidth=1.0, label=name)
        ax_pitch.plot(t_rel, pitch_deg, color=color, linewidth=1.0, label=name)

    for ax in (ax_l, ax_r, ax_bias, ax_pitch):
        ax.axhline(0, color=COLOR_AXIS, linewidth=0.7)
        ax.set_xlabel("time since stage start (s)")

    ax_l.set_ylabel("left DDSM115 current (A)")
    ax_l.legend(loc="upper right", fontsize=7.5)
    ax_r.set_ylabel("right DDSM115 current (A)")
    ax_bias.set_ylabel("current bias, left - right (A)")
    ax_pitch.set_ylabel("pitch (deg)")

    fig.suptitle(f"{title}  ({stage_name})", fontsize=11, fontweight="bold")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")
    return metrics


# --------------------------------------------------------------- summary --

def make_summary_figure(all_metrics, out_path: Path) -> None:
    import matplotlib.pyplot as plt

    stage_keys = [s[0] for s in STAGES if s[0] in all_metrics]
    stage_titles = [STAGE_LABELS.get(k, k) for k in stage_keys]
    labels = list(RUN_COLORS.keys())

    fig, (ax_l, ax_r, ax_bias) = plt.subplots(3, 1, figsize=(7.5, 8.5), constrained_layout=True)
    width = 0.35
    x = range(len(stage_keys))

    for panel, key, ylabel in (
        (ax_l, "rms_current_left_A", "RMS left current (A)"),
        (ax_r, "rms_current_right_A", "RMS right current (A)"),
        (ax_bias, "mean_current_bias_A", "mean current bias, L-R (A)"),
    ):
        for j, label in enumerate(labels):
            vals = [all_metrics[s][label][key] for s in stage_keys]
            offset = (j - 0.5) * width
            panel.bar([i + offset for i in x], vals, width, color=RUN_COLORS[label], label=RUNS[label][1])
        panel.set_ylabel(ylabel)
        panel.set_xticks(list(x))
        panel.set_xticklabels(stage_titles, rotation=35, ha="right", fontsize=8)
        panel.axhline(0, color=COLOR_AXIS, linewidth=0.7)

    ax_l.legend(loc="upper right", fontsize=8)
    ax_l.set_title("Per-wheel DDSM115 current: matched wheels vs. left wheel +50% diameter (range_gru)")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


def write_metrics_csv(all_metrics, out_path: Path) -> None:
    fieldnames = [
        "stage", "condition", "duration_s",
        "rms_current_left_A", "rms_current_right_A",
        "mean_current_bias_A", "rms_current_bias_A",
        "rms_pitch_deg", "max_abs_pitch_deg",
    ]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for stage_key, _title, _kind in STAGES:
            if stage_key not in all_metrics:
                continue
            for label, m in all_metrics[stage_key].items():
                row = {"stage": stage_key, "condition": label}
                row.update(m)
                writer.writerow(row)
    print(f"wrote {out_path}")


def main() -> int:
    base.setup_style()

    runs = {
        "baseline": base.load_run("baseline", *RUNS["baseline"]),
        "bigger": base.load_run("bigger", *RUNS["bigger"]),
    }

    all_metrics = {}
    for stage_key, title, _kind in STAGES:
        print(f"stage: {stage_key}")
        metrics = make_stage_figure(runs, stage_key, title, OUT_DIR / f"stage_{stage_key}.pdf")
        if metrics is not None:
            all_metrics[stage_key] = metrics

    make_summary_figure(all_metrics, OUT_DIR / "summary_current.pdf")
    write_metrics_csv(all_metrics, OUT_DIR / "wheel_asymmetry_metrics.csv")

    # headline numbers
    stage_keys = list(all_metrics.keys())
    base_bias = mean_sd([all_metrics[s]["baseline"]["mean_current_bias_A"] for s in stage_keys])
    big_bias = mean_sd([all_metrics[s]["bigger"]["mean_current_bias_A"] for s in stage_keys])
    base_l = mean_sd([all_metrics[s]["baseline"]["rms_current_left_A"] for s in stage_keys])
    big_l = mean_sd([all_metrics[s]["bigger"]["rms_current_left_A"] for s in stage_keys])
    base_r = mean_sd([all_metrics[s]["baseline"]["rms_current_right_A"] for s in stage_keys])
    big_r = mean_sd([all_metrics[s]["bigger"]["rms_current_right_A"] for s in stage_keys])
    print(
        f"\nheadline, mean +/- s.d. across {len(stage_keys)} stages:\n"
        f"  current bias L-R:  baseline {base_bias[0]:+.4f} +/- {base_bias[1]:.4f} A"
        f"   vs   left-wheel-bigger {big_bias[0]:+.4f} +/- {big_bias[1]:.4f} A\n"
        f"  RMS left current:  baseline {base_l[0]:.4f} +/- {base_l[1]:.4f} A"
        f"   vs   left-wheel-bigger {big_l[0]:.4f} +/- {big_l[1]:.4f} A\n"
        f"  RMS right current: baseline {base_r[0]:.4f} +/- {base_r[1]:.4f} A"
        f"   vs   left-wheel-bigger {big_r[0]:.4f} +/- {big_r[1]:.4f} A"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
