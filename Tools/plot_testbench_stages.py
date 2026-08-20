#!/usr/bin/env python3
"""Plot per-experiment (testbench-stage) behaviour from a paper telemetry CSV.

Unlike the generic diagnostic plot produced during capture (make_plot in
paper_uart_capture.py), this breaks the run down by `testbench_stage_name`
so each experiment in the automatic comparison test bench (settle, left_leg,
forward, spin_positive, sine_legs_forward_reverse, ...) can be read off
individually -- useful for the paper's results section.

Usage:
    python Tools/plot_testbench_stages.py Results/paper/gru_testbench.csv \
        --output Results/paper/gru_testbench_stages.png --title "Range GRU"
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

# Skip the leading/trailing idle padding and the short neutral/hold transition
# stages so the bar chart focuses on the labelled experiments.
SUMMARY_STAGES = [
    "settle",
    "left_leg",
    "right_leg",
    "forward",
    "reverse",
    "spin_positive",
    "spin_negative",
    "sine_legs_forward_reverse",
    "crossed_asymmetric_hold",
    "one_leg_sine_forward_reverse",
]

STAGE_LABELS = {
    "idle": "idle",
    "settle": "settle",
    "left_leg": "left leg",
    "neutral_after_left": "neutral",
    "right_leg": "right leg",
    "neutral_after_right": "neutral",
    "forward": "forward",
    "hold_after_forward": "hold",
    "reverse": "reverse",
    "hold_after_reverse": "hold",
    "spin_positive": "spin +",
    "hold_after_positive_spin": "hold",
    "spin_negative": "spin -",
    "hold_after_negative_spin": "hold",
    "sine_legs_forward_reverse": "sine legs\nfwd/rev",
    "final_hold": "hold",
    "crossed_asymmetric_hold": "crossed\nhold",
    "neutral_after_crossed": "neutral",
    "one_leg_sine_forward_reverse": "one-leg sine\nfwd/rev",
}


def load_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def segment_by_stage(rows: list[dict[str, str]]) -> list[tuple[str, int, int]]:
    """Return contiguous (stage_name, start_index, end_index_inclusive) runs."""
    segments: list[tuple[str, int, int]] = []
    start = 0
    prev = rows[0]["testbench_stage_name"]
    for i, row in enumerate(rows):
        stage = row["testbench_stage_name"]
        if stage != prev:
            segments.append((prev, start, i - 1))
            start = i
            prev = stage
    segments.append((prev, start, len(rows) - 1))
    return segments


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def make_figure(csv_path: Path, output_path: Path, title: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("requires matplotlib: python -m pip install matplotlib") from exc

    rows = load_rows(csv_path)
    t = [float(r["time_s"]) - float(rows[0]["time_s"]) for r in rows]
    theta = [float(r["theta_deg"]) for r in rows]
    cur_l = [float(r["wheel_left_current_measured_A"]) for r in rows]
    cur_r = [float(r["wheel_right_current_measured_A"]) for r in rows]
    pos = [float(r["position_m"]) for r in rows]

    segments = segment_by_stage(rows)

    fig, (ax_theta, ax_cur, ax_pos, ax_bar) = plt.subplots(
        4, 1, figsize=(14, 14), constrained_layout=True,
        gridspec_kw={"height_ratios": [1.1, 1, 1, 1.3]},
    )

    colors = plt.cm.tab20.colors
    for idx, (stage, i0, i1) in enumerate(segments):
        t_span = (t[i0], t[i1] + 1e-6)
        color = colors[idx % len(colors)]
        for ax in (ax_theta, ax_cur, ax_pos):
            ax.axvspan(*t_span, color=color, alpha=0.15, linewidth=0)
        label = STAGE_LABELS.get(stage, stage).replace("\n", " ")
        mid = (t_span[0] + t_span[1]) / 2
        ax_theta.text(
            mid, 1.03, label, transform=ax_theta.get_xaxis_transform(),
            rotation=90, ha="center", va="bottom", fontsize=7,
        )

    ax_theta.plot(t, theta, color="tab:blue", linewidth=0.9)
    ax_theta.axhline(0, color="black", linewidth=0.6)
    ax_theta.set_ylabel("platform tilt (deg)")
    fig.suptitle(f"{title}: automatic test bench, per-experiment breakdown", y=1.05)

    ax_cur.plot(t, cur_l, color="tab:green", linewidth=0.7, label="left measured")
    ax_cur.plot(t, cur_r, color="tab:red", linewidth=0.7, label="right measured")
    ax_cur.set_ylabel("wheel current (A)")
    ax_cur.legend(loc="upper right", fontsize=8)

    ax_pos.plot(t, pos, color="tab:purple", linewidth=0.9)
    ax_pos.set_ylabel("chassis position (m)")
    ax_pos.set_xlabel("time (s)")

    stage_labels, theta_max_vals, theta_rms_vals, cur_rms_vals = [], [], [], []
    for stage, i0, i1 in segments:
        if stage not in SUMMARY_STAGES:
            continue
        seg_theta = theta[i0 : i1 + 1]
        seg_cur = [
            0.5 * (cur_l[k] + cur_r[k]) for k in range(i0, i1 + 1)
        ]
        stage_labels.append(STAGE_LABELS.get(stage, stage))
        theta_max_vals.append(max(abs(v) for v in seg_theta))
        theta_rms_vals.append(rms(seg_theta))
        cur_rms_vals.append(rms(seg_cur))

    x = range(len(stage_labels))
    width = 0.28
    ax_bar.bar([i - width for i in x], theta_max_vals, width, label="max |tilt| (deg)", color="tab:blue")
    ax_bar.bar(x, theta_rms_vals, width, label="RMS tilt (deg)", color="tab:cyan")
    ax_bar2 = ax_bar.twinx()
    ax_bar2.bar([i + width for i in x], cur_rms_vals, width, label="RMS current (A)", color="tab:orange")
    ax_bar.set_xticks(list(x))
    ax_bar.set_xticklabels(stage_labels, rotation=40, ha="right", fontsize=8)
    ax_bar.set_ylabel("tilt (deg)")
    ax_bar2.set_ylabel("current (A)")
    ax_bar.set_title("Per-experiment stability summary")

    lines1, labels1 = ax_bar.get_legend_handles_labels()
    lines2, labels2 = ax_bar2.get_legend_handles_labels()
    ax_bar.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    print(f"wrote {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    make_figure(args.csv, args.output, args.title or args.csv.stem)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
