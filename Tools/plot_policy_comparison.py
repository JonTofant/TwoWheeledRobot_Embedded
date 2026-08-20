#!/usr/bin/env python3
"""Compare the range_mlp and range_gru policies for the paper.

The point_mlp policy fell immediately (its captured CSV never leaves the
`idle` test-bench stage), so it is excluded here -- there is nothing to
compare it against. This script works from the two full test-bench runs
that *did* complete every stage:

    Results/paper/range_mlp_segmented.csv
    Results/paper/range_gru_segmented.csv

Each CSV is one continuous capture that walks through the automatic
comparison test bench (see `testbench_steps[]` in Core/Src/main.c); the
`testbench_stage_name` column annotates which experiment is running at
each row. This script splits each run on that column and, for every
experiment shared by both runs, plots the two policies on top of each
other so it's easy to see which one is doing better where.

Output (all PDF, vector, paper-ready):
    comparison_plots/overview_<label>.pdf        one full-run timeline per policy
    comparison_plots/stage_<stage>.pdf            per-experiment MLP-vs-GRU overlay
    comparison_plots/summary_stability.pdf        tilt/current bar summary, all stages
    comparison_plots/summary_tracking.pdf         velocity/yaw-rate error bar summary
    comparison_plots/stage_metrics.csv            the numbers behind every bar

Usage:
    python Tools/plot_policy_comparison.py
    python Tools/plot_policy_comparison.py --outdir Results/paper/comparison_plots
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PAPER_DIR = REPO_ROOT / "Results" / "paper"

# label -> (csv path, display name)
DEFAULT_RUNS = {
    "range_mlp": (PAPER_DIR / "range_mlp_segmented.csv", "Range MLP"),
    "range_gru": (PAPER_DIR / "range_gru_segmented.csv", "Range GRU"),
}

# validated categorical palette (dataviz skill, references/palette.md) --
# slot 1 (blue) / slot 2 (orange), fixed roles: MLP always blue, GRU always orange.
COLOR_MLP = "#2a78d6"
COLOR_GRU = "#eb6834"
COLOR_CMD = "#0b0b0b"      # commanded/reference line
COLOR_GRID = "#e1e0d9"
COLOR_AXIS = "#c3c2b7"
COLOR_MUTED = "#898781"
COLOR_TEXT = "#0b0b0b"

RUN_COLORS = {"range_mlp": COLOR_MLP, "range_gru": COLOR_GRU}

# (stage_name, display title, kind) -- kind picks the 4th comparison panel.
#   "balance"  -- pure disturbance/balance stage (velocity=yaw=0): show chassis drift
#   "velocity" -- forward/reverse drive present: show commanded vs measured velocity
#   "yaw"      -- spin present: show commanded vs measured yaw rate
STAGES = [
    ("left_leg", "Left leg extend & hold", "balance"),
    ("right_leg", "Right leg extend & hold", "balance"),
    ("forward", "Forward drive", "velocity"),
    ("reverse", "Reverse drive", "velocity"),
    ("spin_positive", "Spin, +yaw", "yaw"),
    ("spin_negative", "Spin, -yaw", "yaw"),
    ("sine_legs_forward", "Sine legs + forward drive", "velocity"),
    ("sine_legs_reverse", "Sine legs + reverse drive", "velocity"),
    ("crossed_asymmetric_hold", "Crossed asymmetric hold", "balance"),
    ("one_leg_sine_forward", "One-leg bump + forward drive", "velocity"),
    ("one_leg_sine_reverse", "One-leg bump + reverse drive", "velocity"),
]

STAGE_LABELS = {
    "idle": "idle",
    "settle": "settle",
    "left_leg": "left leg",
    "neutral_after_left": "neutral",
    "right_leg": "right leg",
    "neutral_after_right": "neutral",
    "checkpoint_before_forward": "checkpoint",
    "forward": "forward",
    "hold_after_forward": "hold",
    "checkpoint_before_reverse": "checkpoint",
    "reverse": "reverse",
    "hold_after_reverse": "hold",
    "spin_positive": "spin +",
    "hold_after_positive_spin": "hold",
    "spin_negative": "spin -",
    "hold_after_negative_spin": "hold",
    "checkpoint_before_sine_forward": "checkpoint",
    "sine_legs_forward": "sine legs fwd",
    "checkpoint_before_sine_reverse": "checkpoint",
    "sine_legs_reverse": "sine legs rev",
    "final_hold": "hold",
    "crossed_asymmetric_hold": "crossed hold",
    "neutral_after_crossed": "neutral",
    "checkpoint_before_one_leg_forward": "checkpoint",
    "one_leg_sine_forward": "one-leg fwd",
    "checkpoint_before_one_leg_reverse": "checkpoint",
    "one_leg_sine_reverse": "one-leg rev",
}


@dataclass
class Run:
    label: str
    name: str
    rows: list[dict[str, str]]
    t: list[float]
    segments: list[tuple[str, int, int]]

    def f(self, col: str, i0: int, i1: int) -> list[float]:
        return [float(self.rows[k][col]) for k in range(i0, i1 + 1)]

    def find_stage(self, stage_name: str) -> tuple[int, int] | None:
        for stage, i0, i1 in self.segments:
            if stage == stage_name:
                return i0, i1
        return None


def load_run(label: str, csv_path: Path, name: str) -> Run:
    with csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    t0 = float(rows[0]["time_s"])
    t = [float(r["time_s"]) - t0 for r in rows]
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
    return Run(label=label, name=name, rows=rows, t=t, segments=segments)


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def setup_style() -> None:
    import matplotlib

    matplotlib.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.size": 9,
            "axes.edgecolor": COLOR_AXIS,
            "axes.labelcolor": COLOR_TEXT,
            "axes.titlesize": 10,
            "axes.titleweight": "bold",
            "text.color": COLOR_TEXT,
            "xtick.color": COLOR_MUTED,
            "ytick.color": COLOR_MUTED,
            "grid.color": COLOR_GRID,
            "grid.linewidth": 0.6,
            "axes.grid": True,
            "axes.axisbelow": True,
            "legend.frameon": False,
            "pdf.fonttype": 42,  # embed as real text, not paths
            "ps.fonttype": 42,
        }
    )


# ---------------------------------------------------------------- overview --

def make_overview(run: Run, out_path: Path) -> None:
    import matplotlib.pyplot as plt

    t, rows = run.t, run.rows
    pitch_deg = [math.degrees(float(r["pitch_rad"])) for r in rows]
    cur_l = [float(r["wheel_left_current_measured_A"]) for r in rows]
    cur_r = [float(r["wheel_right_current_measured_A"]) for r in rows]
    pos = [float(r["position_m"]) for r in rows]

    fig, (ax_p, ax_c, ax_x) = plt.subplots(
        3, 1, figsize=(7.0, 6.4), constrained_layout=True, sharex=True
    )

    for idx, (stage, i0, i1) in enumerate(run.segments):
        if stage in ("idle",):
            continue
        shade = "#f4f3f0" if idx % 2 == 0 else "#fcfcfb"
        for ax in (ax_p, ax_c, ax_x):
            ax.axvspan(t[i0], t[i1] + 1e-6, color=shade, zorder=0, linewidth=0)
        label = STAGE_LABELS.get(stage, stage)
        if "checkpoint" in stage or label == "neutral" or label == "hold":
            continue
        mid = (t[i0] + t[i1]) / 2
        ax_p.text(
            mid, 1.02, label, transform=ax_p.get_xaxis_transform(),
            rotation=90, ha="center", va="bottom", fontsize=6, color=COLOR_MUTED,
        )

    ax_p.plot(t, pitch_deg, color=RUN_COLORS[run.label], linewidth=0.8)
    ax_p.axhline(0, color=COLOR_AXIS, linewidth=0.7)
    ax_p.set_ylabel("pitch (deg)")
    fig.suptitle(f"{run.name}: full test-bench run", y=1.1, fontsize=11, fontweight="bold")

    ax_c.plot(t, cur_l, color=COLOR_MLP, linewidth=0.6, label="left measured", alpha=0.8)
    ax_c.plot(t, cur_r, color=COLOR_GRU, linewidth=0.6, label="right measured", alpha=0.8)
    ax_c.set_ylabel("wheel current (A)")
    ax_c.legend(loc="upper right", fontsize=7, ncols=2)

    ax_x.plot(t, pos, color="#4a3aa7", linewidth=0.8)
    ax_x.axhline(0, color=COLOR_AXIS, linewidth=0.7)
    ax_x.set_ylabel("chassis position (m)")
    ax_x.set_xlabel("time (s)")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")


# ------------------------------------------------------------ stage plots --

def compute_stage_metrics(
    runs: dict[str, Run], stage_name: str, kind: str
) -> tuple[dict[str, tuple[int, int]], dict[str, dict[str, float]]] | None:
    """Per-run (span, metrics) for one test-bench stage, no plotting."""
    spans = {}
    for label, run in runs.items():
        span = run.find_stage(stage_name)
        if span is None:
            return None
        spans[label] = span

    metrics: dict[str, dict[str, float]] = {}
    for label, run in runs.items():
        i0, i1 = spans[label]

        pitch_deg = [math.degrees(v) for v in run.f("pitch_rad", i0, i1)]
        roll_deg = [math.degrees(v) for v in run.f("roll_rad", i0, i1)]
        cur_l = run.f("wheel_left_current_measured_A", i0, i1)
        cur_r = run.f("wheel_right_current_measured_A", i0, i1)
        cur_avg = [0.5 * (abs(a) + abs(b)) for a, b in zip(cur_l, cur_r)]

        m = {
            "duration_s": run.t[i1] - run.t[i0],
            "max_abs_pitch_deg": max(abs(v) for v in pitch_deg),
            "rms_pitch_deg": rms(pitch_deg),
            "max_abs_roll_deg": max(abs(v) for v in roll_deg),
            "rms_roll_deg": rms(roll_deg),
            "rms_current_A": rms(cur_avg),
        }

        if kind == "balance":
            pos = run.f("position_m", i0, i1)
            drift = [p - pos[0] for p in pos]
            m["max_abs_drift_m"] = max(abs(v) for v in drift)
        elif kind == "velocity":
            v_cmd = run.f("obs_7", i0, i1)
            v_meas = run.f("velocity_mps", i0, i1)
            err = [meas - cmd for meas, cmd in zip(v_meas, v_cmd)]
            m["rms_velocity_error_mps"] = rms(err)
        elif kind == "yaw":
            w_cmd = [v * 2.0 for v in run.f("obs_8", i0, i1)]
            w_meas = run.f("yaw_rate_world_radps", i0, i1)
            err = [meas - cmd for meas, cmd in zip(w_meas, w_cmd)]
            m["rms_yawrate_error_radps"] = rms(err)

        metrics[label] = m

    return spans, metrics


def make_stage_figure(
    runs: dict[str, Run], stage_name: str, title: str, kind: str, out_path: Path
) -> dict[str, dict[str, float]] | None:
    import matplotlib.pyplot as plt

    result = compute_stage_metrics(runs, stage_name, kind)
    if result is None:
        print(f"  ! {stage_name}: missing from one or more runs, skipping figure")
        return None
    spans, metrics = result

    fig, axes = plt.subplots(4, 1, figsize=(6.5, 8.0), constrained_layout=True, sharex=False)
    ax_pitch, ax_roll, ax_cur, ax_track = axes

    for label, run in runs.items():
        i0, i1 = spans[label]
        t_rel = [run.t[k] - run.t[i0] for k in range(i0, i1 + 1)]
        color = RUN_COLORS[label]

        pitch_deg = [math.degrees(v) for v in run.f("pitch_rad", i0, i1)]
        roll_deg = [math.degrees(v) for v in run.f("roll_rad", i0, i1)]
        cur_l = run.f("wheel_left_current_measured_A", i0, i1)
        cur_r = run.f("wheel_right_current_measured_A", i0, i1)
        cur_avg = [0.5 * (abs(a) + abs(b)) for a, b in zip(cur_l, cur_r)]

        ax_pitch.plot(t_rel, pitch_deg, color=color, linewidth=1.1, label=run.name)
        ax_roll.plot(t_rel, roll_deg, color=color, linewidth=1.0, label=run.name)
        ax_cur.plot(t_rel, cur_avg, color=color, linewidth=1.0, label=run.name)

        if kind == "balance":
            pos = run.f("position_m", i0, i1)
            drift = [p - pos[0] for p in pos]
            ax_track.plot(t_rel, drift, color=color, linewidth=1.0, label=run.name)
        elif kind == "velocity":
            v_cmd = run.f("obs_7", i0, i1)
            v_meas = run.f("velocity_mps", i0, i1)
            ax_track.plot(t_rel, v_meas, color=color, linewidth=1.0, label=f"{run.name} measured")
            if label == next(iter(runs)):
                ax_track.plot(t_rel, v_cmd, color=COLOR_CMD, linewidth=1.0, linestyle="--", label="commanded")
        elif kind == "yaw":
            w_cmd = [v * 2.0 for v in run.f("obs_8", i0, i1)]
            w_meas = run.f("yaw_rate_world_radps", i0, i1)
            ax_track.plot(t_rel, w_meas, color=color, linewidth=1.0, label=f"{run.name} measured")
            if label == next(iter(runs)):
                ax_track.plot(t_rel, w_cmd, color=COLOR_CMD, linewidth=1.0, linestyle="--", label="commanded")

    for ax in axes:
        ax.axhline(0, color=COLOR_AXIS, linewidth=0.7)
        ax.set_xlabel("time since stage start (s)")

    ax_pitch.set_ylabel("pitch (deg)")
    ax_pitch.legend(loc="upper right", fontsize=8)
    ax_roll.set_ylabel("roll (deg)")
    ax_cur.set_ylabel("mean |wheel current| (A)")

    if kind == "balance":
        ax_track.set_ylabel("chassis drift (m)")
    elif kind == "velocity":
        ax_track.set_ylabel("velocity (m/s)")
        ax_track.legend(loc="upper right", fontsize=8)
    elif kind == "yaw":
        ax_track.set_ylabel("yaw rate (rad/s)")
        ax_track.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"{title}  ({stage_name})", fontsize=11, fontweight="bold")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")
    return metrics


# --------------------------------------------------------------- summary --

def make_stability_summary(all_metrics: dict[str, dict[str, dict[str, float]]], out_path: Path) -> None:
    import matplotlib.pyplot as plt

    stage_keys = [s[0] for s in STAGES if s[0] in all_metrics]
    stage_titles = [STAGE_LABELS.get(k, k) for k in stage_keys]
    labels = list(RUN_COLORS.keys())

    fig, (ax_max, ax_rms, ax_cur) = plt.subplots(3, 1, figsize=(7.5, 9.0), constrained_layout=True)
    width = 0.35
    x = range(len(stage_keys))

    for panel, key, ylabel in (
        (ax_max, "max_abs_pitch_deg", "max |pitch| (deg)"),
        (ax_rms, "rms_pitch_deg", "RMS pitch (deg)"),
        (ax_cur, "rms_current_A", "RMS wheel current (A)"),
    ):
        for j, label in enumerate(labels):
            vals = [all_metrics[s][label][key] for s in stage_keys]
            offset = (j - 0.5) * width
            panel.bar(
                [i + offset for i in x], vals, width,
                color=RUN_COLORS[label],
                label={"range_mlp": "Range MLP", "range_gru": "Range GRU"}[label],
            )
        panel.set_ylabel(ylabel)
        panel.set_xticks(list(x))
        panel.set_xticklabels(stage_titles, rotation=35, ha="right", fontsize=8)
        panel.axhline(0, color=COLOR_AXIS, linewidth=0.7)

    ax_max.legend(loc="upper right", fontsize=8)
    ax_max.set_title("Stability comparison across every test-bench experiment")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")


def make_tracking_summary(all_metrics: dict[str, dict[str, dict[str, float]]], out_path: Path) -> None:
    import matplotlib.pyplot as plt

    vel_stages = [s[0] for s in STAGES if s[2] == "velocity" and s[0] in all_metrics]
    yaw_stages = [s[0] for s in STAGES if s[2] == "yaw" and s[0] in all_metrics]
    labels = list(RUN_COLORS.keys())

    fig, (ax_v, ax_w) = plt.subplots(2, 1, figsize=(6.5, 6.0), constrained_layout=True)
    width = 0.35

    x = range(len(vel_stages))
    for j, label in enumerate(labels):
        vals = [all_metrics[s][label]["rms_velocity_error_mps"] for s in vel_stages]
        offset = (j - 0.5) * width
        ax_v.bar([i + offset for i in x], vals, width, color=RUN_COLORS[label],
                 label={"range_mlp": "Range MLP", "range_gru": "Range GRU"}[label])
    ax_v.set_ylabel("RMS velocity error (m/s)")
    ax_v.set_xticks(list(x))
    ax_v.set_xticklabels([STAGE_LABELS.get(s, s) for s in vel_stages], rotation=35, ha="right", fontsize=8)
    ax_v.legend(loc="upper right", fontsize=8)
    ax_v.set_title("Command-tracking error comparison")

    x = range(len(yaw_stages))
    for j, label in enumerate(labels):
        vals = [all_metrics[s][label]["rms_yawrate_error_radps"] for s in yaw_stages]
        offset = (j - 0.5) * width
        ax_w.bar([i + offset for i in x], vals, width, color=RUN_COLORS[label],
                 label={"range_mlp": "Range MLP", "range_gru": "Range GRU"}[label])
    ax_w.set_ylabel("RMS yaw-rate error (rad/s)")
    ax_w.set_xticks(list(x))
    ax_w.set_xticklabels([STAGE_LABELS.get(s, s) for s in yaw_stages], rotation=0, ha="center", fontsize=8)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")


def write_metrics_csv(all_metrics: dict[str, dict[str, dict[str, float]]], out_path: Path) -> None:
    fieldnames = [
        "stage", "policy", "duration_s",
        "max_abs_pitch_deg", "rms_pitch_deg",
        "max_abs_roll_deg", "rms_roll_deg",
        "rms_current_A", "max_abs_drift_m",
        "rms_velocity_error_mps", "rms_yawrate_error_radps",
    ]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for stage_key, title, _kind in STAGES:
            if stage_key not in all_metrics:
                continue
            for label, m in all_metrics[stage_key].items():
                row = {"stage": stage_key, "policy": label}
                row.update(m)
                writer.writerow(row)
    print(f"wrote {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--outdir", type=Path, default=PAPER_DIR / "comparison_plots")
    parser.add_argument(
        "--mlp-csv", type=Path, default=DEFAULT_RUNS["range_mlp"][0],
        help="range_mlp_segmented.csv equivalent",
    )
    parser.add_argument(
        "--gru-csv", type=Path, default=DEFAULT_RUNS["range_gru"][0],
        help="range_gru_segmented.csv equivalent",
    )
    args = parser.parse_args()

    setup_style()

    runs = {
        "range_mlp": load_run("range_mlp", args.mlp_csv, DEFAULT_RUNS["range_mlp"][1]),
        "range_gru": load_run("range_gru", args.gru_csv, DEFAULT_RUNS["range_gru"][1]),
    }

    for label, run in runs.items():
        make_overview(run, args.outdir / f"overview_{label}.pdf")

    all_metrics: dict[str, dict[str, dict[str, float]]] = {}
    for stage_key, title, kind in STAGES:
        print(f"stage: {stage_key}")
        metrics = make_stage_figure(runs, stage_key, title, kind, args.outdir / f"stage_{stage_key}.pdf")
        if metrics is not None:
            all_metrics[stage_key] = metrics

    make_stability_summary(all_metrics, args.outdir / "summary_stability.pdf")
    make_tracking_summary(all_metrics, args.outdir / "summary_tracking.pdf")
    write_metrics_csv(all_metrics, args.outdir / "stage_metrics.csv")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
