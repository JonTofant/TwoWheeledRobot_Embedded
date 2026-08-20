#!/usr/bin/env python3
"""Build the paper-ready hardware-validation figures and the DWT timing table.

The manuscript (main.pdf) currently has no hardware section: Sec. 2.8
"Experimental Protocol" is an empty header, Sec. 4.3 "Sim-to-Real Gap" is an
empty stub, and the Data Availability statement says hardware trials "remain
to be added". This script produces that content from the same two full
test-bench captures used by plot_policy_comparison.py
(range_mlp_segmented.csv = Variant B, range_gru_segmented.csv = Variant C),
styled to match the simulation figures already in the paper:

  - Fig. 4 (aggregate bar comparison, mean +/- s.d., "lower is better")
  - Fig. 5 (scenario-wise RMS velocity-tracking-error bars)

Outputs, in Results/paper/paper_figures/:
    fig_hw_aggregate.pdf            hardware analog of Fig. 4 (B vs C)
    fig_hw_scenario_velocity.pdf    hardware analog of Fig. 5 (B vs C, all
                                     6 driving scenarios)
    fig_hw_scenario_yaw.pdf         same idea for the 2 spin scenarios
    fig_hw_transient_forward.pdf    representative transient: forward drive
    fig_hw_transient_spin.pdf       representative transient: spin+
    fig_hw_protocol_overview.pdf    full-run timeline for Sec. 2.8
    timing_table.csv                DWT mean/WCET/jitter per pipeline stage
    timing_table.tex                same, as a paste-ready MDPI-style table

Variant A (point MLP) is not plotted: it fell before leaving the `idle`
test-bench stage, so there is no time series to show. That result is a
one-line fact for the text (see the script's printed summary), not a figure.

Usage:
    python Tools/build_paper_hardware_results.py
"""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

import plot_policy_comparison as base

REPO_ROOT = base.REPO_ROOT
PAPER_DIR = base.PAPER_DIR
OUT_DIR = PAPER_DIR / "paper_figures"

COLOR_MLP = base.COLOR_MLP
COLOR_GRU = base.COLOR_GRU
COLOR_CMD = base.COLOR_CMD
COLOR_AXIS = base.COLOR_AXIS
RUN_COLORS = base.RUN_COLORS
STAGE_LABELS = base.STAGE_LABELS
STAGES = base.STAGES

# Timing quantities exported by paper_metrics.c (Core/Src/paper_metrics.c),
# in pipeline order, with the paper-facing label for each.
TIMING_QUANTITIES = [
    ("imu_read", "IMU read"),
    ("encoder_read", "Wheel encoder read"),
    ("observation", "Observation assembly"),
    ("inference", "Policy inference"),
    ("action", "Action processing"),
    ("motor_write", "Motor command write"),
    ("total", "Total task"),
    ("control_period", "Control period"),
    ("release_latency", "Task release latency"),
    ("rs485_cycle", "Two-wheel RS485 cycle"),
    ("imu_age", "IMU sample age"),
    ("wheel_age", "Wheel sample age"),
    ("leg_age", "Leg sample age"),
]

TIMING_SOURCES = {
    "MLP (A/B)": PAPER_DIR / "range_mlp_segmented.csv",
    "GRU (C)": PAPER_DIR / "range_gru_segmented.csv",
}


def mean_sd(values: list[float]) -> tuple[float, float]:
    if len(values) < 2:
        return (values[0] if values else float("nan")), 0.0
    return statistics.mean(values), statistics.stdev(values)


# ----------------------------------------------------------- aggregate fig --

def collect_all_metrics(runs: dict[str, base.Run]) -> dict[str, dict[str, dict[str, float]]]:
    all_metrics: dict[str, dict[str, dict[str, float]]] = {}
    for stage_key, _title, kind in STAGES:
        result = base.compute_stage_metrics(runs, stage_key, kind)
        if result is not None:
            _spans, metrics = result
            all_metrics[stage_key] = metrics
    return all_metrics


def make_aggregate_figure(all_metrics: dict[str, dict[str, dict[str, float]]], out_path: Path) -> None:
    import matplotlib.pyplot as plt

    labels = list(RUN_COLORS.keys())
    display_name = {"range_mlp": "B: range DR, MLP", "range_gru": "C: range DR, GRU"}

    all_stage_keys = [s[0] for s in STAGES if s[0] in all_metrics]
    vel_stage_keys = [s[0] for s in STAGES if s[2] == "velocity" and s[0] in all_metrics]
    yaw_stage_keys = [s[0] for s in STAGES if s[2] == "yaw" and s[0] in all_metrics]
    balance_stage_keys = [s[0] for s in STAGES if s[2] == "balance" and s[0] in all_metrics]

    panels = [
        ("RMS pitch", "deg", all_stage_keys, "rms_pitch_deg"),
        ("RMS velocity error", "m/s", vel_stage_keys, "rms_velocity_error_mps"),
        ("RMS yaw-rate error", "rad/s", yaw_stage_keys, "rms_yawrate_error_radps"),
        ("Chassis drift", "m", balance_stage_keys, "max_abs_drift_m"),
    ]

    fig, axes = plt.subplots(1, 4, figsize=(9.5, 3.2), constrained_layout=True)

    for ax, (title, unit, stage_keys, metric_key) in zip(axes, panels):
        x = range(len(labels))
        means, sds = [], []
        for label in labels:
            vals = [all_metrics[s][label][metric_key] for s in stage_keys]
            m, sd = mean_sd(vals)
            means.append(m)
            sds.append(sd)
        colors = [RUN_COLORS[label] for label in labels]
        ax.bar(list(x), means, yerr=sds, capsize=3, color=colors, width=0.6,
               error_kw={"linewidth": 1.0, "ecolor": "#52514e"})
        for i, v in enumerate(means):
            ax.text(i, v, f"{v:.3g}", ha="center", va="bottom", fontsize=7)
        ax.set_xticks(list(x))
        ax.set_xticklabels(["B", "C"], fontsize=9)
        ax.set_ylabel(unit)
        ax.set_title(f"{title}\n(n={len(stage_keys)} stages)", fontsize=8.5)
        ax.axhline(0, color=COLOR_AXIS, linewidth=0.7)

    handles = [plt.Rectangle((0, 0), 1, 1, color=RUN_COLORS[l]) for l in labels]
    fig.legend(handles, [display_name[l] for l in labels], loc="upper center",
               bbox_to_anchor=(0.5, 1.14), ncols=2, fontsize=9, frameon=False)
    fig.suptitle(
        "Hardware aggregate comparison, mean +/- s.d. across test-bench stages\n"
        "(single hardware run per policy; lower is better for every panel)",
        y=1.22, fontsize=9,
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# ------------------------------------------------------- scenario bar figs --

def make_scenario_bar(
    all_metrics: dict[str, dict[str, dict[str, float]]],
    stage_keys: list[str],
    metric_key: str,
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    import matplotlib.pyplot as plt

    labels = list(RUN_COLORS.keys())
    display_name = {"range_mlp": "B: range+MLP", "range_gru": "C: range+GRU"}

    fig, ax = plt.subplots(figsize=(6.0, 3.4), constrained_layout=True)
    width = 0.35
    x = range(len(stage_keys))
    for j, label in enumerate(labels):
        vals = [all_metrics[s][label][metric_key] for s in stage_keys]
        offset = (j - 0.5) * width
        ax.bar([i + offset for i in x], vals, width, color=RUN_COLORS[label],
               label=display_name[label])
    ax.set_ylabel(ylabel)
    ax.set_xticks(list(x))
    ax.set_xticklabels([STAGE_LABELS.get(s, s) for s in stage_keys], rotation=30, ha="right", fontsize=8)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(title, fontsize=10)
    ax.axhline(0, color=COLOR_AXIS, linewidth=0.7)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# ------------------------------------------------------------- transients --

def make_transient_figure(
    runs: dict[str, base.Run], stage_key: str, kind: str, title: str, out_path: Path
) -> None:
    import matplotlib.pyplot as plt

    result = base.compute_stage_metrics(runs, stage_key, kind)
    if result is None:
        print(f"  ! {stage_key}: missing, skipping transient figure")
        return
    spans, _metrics = result

    fig, (ax_pitch, ax_track) = plt.subplots(
        2, 1, figsize=(5.5, 4.6), constrained_layout=True, sharex=False
    )

    for label, run in runs.items():
        i0, i1 = spans[label]
        t_rel = [run.t[k] - run.t[i0] for k in range(i0, i1 + 1)]
        color = RUN_COLORS[label]
        pitch_deg = [math.degrees(v) for v in run.f("pitch_rad", i0, i1)]
        ax_pitch.plot(t_rel, pitch_deg, color=color, linewidth=1.1, label=run.name)

        if kind == "velocity":
            v_cmd = run.f("obs_7", i0, i1)
            v_meas = run.f("velocity_mps", i0, i1)
            ax_track.plot(t_rel, v_meas, color=color, linewidth=1.1, label=f"{run.name} measured")
            if label == next(iter(runs)):
                ax_track.plot(t_rel, v_cmd, color=COLOR_CMD, linewidth=1.0, linestyle="--", label="commanded")
            ax_track.set_ylabel("velocity (m/s)")
        elif kind == "yaw":
            w_cmd = [v * 2.0 for v in run.f("obs_8", i0, i1)]
            w_meas = run.f("yaw_rate_world_radps", i0, i1)
            ax_track.plot(t_rel, w_meas, color=color, linewidth=1.1, label=f"{run.name} measured")
            if label == next(iter(runs)):
                ax_track.plot(t_rel, w_cmd, color=COLOR_CMD, linewidth=1.0, linestyle="--", label="commanded")
            ax_track.set_ylabel("yaw rate (rad/s)")

    for ax in (ax_pitch, ax_track):
        ax.axhline(0, color=COLOR_AXIS, linewidth=0.7)
        ax.set_xlabel("time since stage start (s)")
    ax_pitch.set_ylabel("pitch (deg)")
    ax_pitch.legend(loc="upper right", fontsize=8)
    ax_track.legend(loc="upper right", fontsize=8)
    fig.suptitle(title, fontsize=10, fontweight="bold")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# --------------------------------------------------------------- timing table --

def build_timing_table(out_csv: Path, out_tex: Path) -> None:
    rows: dict[str, dict[str, dict[str, float]]] = {}  # quantity -> variant -> {mean, wcet, jitter}
    for variant, path in TIMING_SOURCES.items():
        with path.open(newline="") as handle:
            last = None
            for last in csv.DictReader(handle):
                pass
        if last is None:
            raise SystemExit(f"{path} has no rows")
        for qty, _label in TIMING_QUANTITIES:
            rows.setdefault(qty, {})[variant] = {
                "mean_us": float(last[f"{qty}_mean_us"]),
                "wcet_us": float(last[f"{qty}_wcet_us"]),
                "jitter_us": float(last[f"{qty}_jitter_us"]),
            }

    variants = list(TIMING_SOURCES.keys())

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["quantity", "label"]
            + [f"{v}_{stat}" for v in variants for stat in ("mean_us", "wcet_us", "jitter_us")]
        )
        for qty, label in TIMING_QUANTITIES:
            row = [qty, label]
            for v in variants:
                d = rows[qty][v]
                row += [f"{d['mean_us']:.2f}", f"{d['wcet_us']:.2f}", f"{d['jitter_us']:.2f}"]
            writer.writerow(row)
    print(f"wrote {out_csv}")

    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\caption{Measured task-timing statistics (mean, worst-case execution time, and jitter as the "
                 r"sample standard deviation), by pipeline stage, over the complete automatic test-bench "
                 r"capture window for each network architecture. MLP timing is shared by Variants A and B (identical "
                 r"exported graph); GRU is Variant C. IMU read and wheel encoder read are serviced asynchronously "
                 r"(interrupt/DMA) outside the timed 15\,ms task and are not summands of Total task; Total task is "
                 r"observation assembly plus inference plus action processing plus motor command write.}")
    lines.append(r"\newcolumntype{C}{>{\centering\arraybackslash}X}")
    lines.append(r"\begin{tabularx}{\textwidth}{lCCCCCC}")
    lines.append(r"\toprule")
    lines.append(
        r"\multirow{2}{*}{Pipeline stage} & \multicolumn{3}{c}{MLP (A/B), \si{\micro\second}} & "
        r"\multicolumn{3}{c}{GRU (C), \si{\micro\second}} \\"
    )
    lines.append(r"\cmidrule(lr){2-4} \cmidrule(lr){5-7}")
    lines.append(r" & Mean & WCET & Jitter & Mean & WCET & Jitter \\")
    lines.append(r"\midrule")
    for qty, label in TIMING_QUANTITIES:
        d_mlp = rows[qty]["MLP (A/B)"]
        d_gru = rows[qty]["GRU (C)"]
        lines.append(
            f"{label} & {d_mlp['mean_us']:.1f} & {d_mlp['wcet_us']:.1f} & {d_mlp['jitter_us']:.1f} & "
            f"{d_gru['mean_us']:.1f} & {d_gru['wcet_us']:.1f} & {d_gru['jitter_us']:.1f} \\\\"
        )
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabularx}")
    lines.append(r"\label{tab:timing}")
    lines.append(r"\end{table}")

    out_tex.parent.mkdir(parents=True, exist_ok=True)
    out_tex.write_text("\n".join(lines) + "\n")
    print(f"wrote {out_tex}")

    inference_mlp = rows["inference"]["MLP (A/B)"]["mean_us"]
    inference_gru = rows["inference"]["GRU (C)"]["mean_us"]
    total_mlp = rows["total"]["MLP (A/B)"]["wcet_us"]
    total_gru = rows["total"]["GRU (C)"]["wcet_us"]
    print(
        f"\nheadline: inference mean {inference_mlp:.1f} us (MLP) vs {inference_gru:.1f} us (GRU); "
        f"total-task WCET {total_mlp:.1f} us (MLP) vs {total_gru:.1f} us (GRU), "
        f"both well inside the 15000 us control period."
    )


def main() -> int:
    base.setup_style()

    runs = {
        "range_mlp": base.load_run("range_mlp", base.DEFAULT_RUNS["range_mlp"][0], base.DEFAULT_RUNS["range_mlp"][1]),
        "range_gru": base.load_run("range_gru", base.DEFAULT_RUNS["range_gru"][0], base.DEFAULT_RUNS["range_gru"][1]),
    }

    all_metrics = collect_all_metrics(runs)

    make_aggregate_figure(all_metrics, OUT_DIR / "fig_hw_aggregate.pdf")

    vel_stage_keys = [s[0] for s in STAGES if s[2] == "velocity" and s[0] in all_metrics]
    yaw_stage_keys = [s[0] for s in STAGES if s[2] == "yaw" and s[0] in all_metrics]
    make_scenario_bar(
        all_metrics, vel_stage_keys, "rms_velocity_error_mps", "RMS velocity error (m/s)",
        "Scenario-wise RMS longitudinal-velocity tracking error, hardware (B vs C)",
        OUT_DIR / "fig_hw_scenario_velocity.pdf",
    )
    make_scenario_bar(
        all_metrics, yaw_stage_keys, "rms_yawrate_error_radps", "RMS yaw-rate error (rad/s)",
        "Scenario-wise RMS yaw-rate tracking error, hardware (B vs C)",
        OUT_DIR / "fig_hw_scenario_yaw.pdf",
    )

    make_transient_figure(
        runs, "forward", "velocity", "Representative transient: forward drive (B vs C)",
        OUT_DIR / "fig_hw_transient_forward.pdf",
    )
    make_transient_figure(
        runs, "spin_positive", "yaw", "Representative transient: spin, +yaw (B vs C)",
        OUT_DIR / "fig_hw_transient_spin.pdf",
    )

    base.make_overview(runs["range_gru"], OUT_DIR / "fig_hw_protocol_overview.pdf")

    build_timing_table(OUT_DIR / "timing_table.csv", OUT_DIR / "timing_table.tex")

    print(
        "\nVariant A (point MLP) note for the text: its capture never leaves the `idle` "
        "test-bench stage -- the robot fell before the automatic sequence could start, so "
        "there is no A time series to plot alongside B/C."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
