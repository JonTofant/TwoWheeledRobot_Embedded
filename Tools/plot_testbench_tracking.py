#!/usr/bin/env python3
"""Plot command-tracking error from a paper telemetry CSV recorded during the
automatic comparison test bench (see TestBench_Update() / testbench_steps[]
in Core/Src/main.c and Docs/PAPER_TELEMETRY.md).

The wheel policy's linear-velocity and yaw-rate commands are logged directly
in the observation vector (obs_7 = v_cmd_mps, obs_8*2 = w_cmd_radps -- see
ANN_Run() in main.c), so those are decoded straight from the CSV. Leg-angle
commands are NOT logged (legs sit outside the 13-dim observation), so this
script re-derives them by replaying TestBench_Update()'s ramp/sine logic in
Python from the logged stage sequence and per-row timestamps -- it assumes
the test bench was run with the default DBG_testbench_leg_degrees = 45 deg
(the amplitude itself isn't in the CSV; pass --leg-degrees to override).

Produces three figures:
  <stem>_velocity_tracking.png  -- commanded vs measured forward velocity,
                                    velocity error, and position error
  <stem>_yawrate_tracking.png   -- commanded vs measured yaw rate, yaw-rate
                                    error, and heading error
  <stem>_leg_tracking.png       -- commanded vs measured angle and error for
                                    all four leg actuators

Usage:
    python Tools/plot_testbench_tracking.py Results/paper/gru_testbench.csv \
        --outdir Results/paper --title "Range GRU"
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

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

# (left_leg_scale, right_leg_scale) used by TestBench_Update()'s generic
# raised-cosine ramp branch. Stages not listed hold both legs at 0. The
# sine and crossed-pose stages are handled as special cases below, matching
# the firmware's own if/else ordering.
LEG_SCALE = {
    "left_leg": (1.0, 0.0),
    "right_leg": (0.0, 1.0),
}

TESTBENCH_LEG_RAMP_S = 1.0
TESTBENCH_SINE_PERIOD_S = 2.0


def load_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def segment_by_stage(rows: list[dict[str, str]]) -> list[tuple[str, int, int]]:
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


def raised_cosine(blend: float) -> float:
    blend = max(0.0, min(1.0, blend))
    return 0.5 * (1.0 - math.cos(math.pi * blend))


def reconstruct_leg_commands(
    rows: list[dict[str, str]], t: list[float], leg_degrees: float
) -> list[tuple[float, float, float, float]]:
    """Replays TestBench_Update()'s leg ramp/sine/crossed-pose logic."""
    commands: list[tuple[float, float, float, float]] = []
    ext_left = ext_right = 0.0
    start_left = start_right = 0.0
    stage_time = 0.0
    prev_stage: str | None = None
    for i, row in enumerate(rows):
        stage = row["testbench_stage_name"]
        if prev_stage is None:
            stage_time = 0.0
        elif stage != prev_stage:
            start_left, start_right = ext_left, ext_right
            stage_time = 0.0
        else:
            stage_time += t[i] - t[i - 1]
        prev_stage = stage

        if stage == "idle":
            ext_left = ext_right = 0.0
        elif stage == "one_leg_sine_forward_reverse":
            phase = 2.0 * math.pi * stage_time / TESTBENCH_SINE_PERIOD_S
            ext_left = leg_degrees * 0.5 * (1.0 - math.cos(phase))
            ext_right = 0.0
        elif stage == "sine_legs_forward_reverse":
            phase = 2.0 * math.pi * stage_time / TESTBENCH_SINE_PERIOD_S
            e = leg_degrees * 0.5 * (1.0 - math.cos(phase))
            ext_left = ext_right = e
        else:
            left_scale, right_scale = LEG_SCALE.get(stage, (0.0, 0.0))
            blend = raised_cosine(stage_time / TESTBENCH_LEG_RAMP_S)
            left_target = left_scale * leg_degrees
            right_target = right_scale * leg_degrees
            ext_left = start_left + (left_target - start_left) * blend
            ext_right = start_right + (right_target - start_right) * blend

        left_rad = math.radians(ext_left)
        right_rad = math.radians(ext_right)
        cmd = (left_rad, -left_rad, right_rad, -right_rad)

        if stage == "crossed_asymmetric_hold":
            blend = raised_cosine(stage_time / TESTBENCH_LEG_RAMP_S)
            crossed_rad = math.radians(leg_degrees) * blend
            cmd = (crossed_rad, 0.0, crossed_rad, 0.0)
        elif stage == "neutral_after_crossed":
            blend = raised_cosine(stage_time / TESTBENCH_LEG_RAMP_S)
            crossed_rad = math.radians(leg_degrees) * (1.0 - blend)
            cmd = (crossed_rad, 0.0, crossed_rad, 0.0)

        commands.append(cmd)
    return commands


def shade_stages(ax, segments, t, colors, label: bool = False) -> None:
    for idx, (stage, i0, i1) in enumerate(segments):
        t_span = (t[i0], t[i1] + 1e-6)
        color = colors[idx % len(colors)]
        ax.axvspan(*t_span, color=color, alpha=0.15, linewidth=0)
        if label:
            lbl = STAGE_LABELS.get(stage, stage).replace("\n", " ")
            mid = (t_span[0] + t_span[1]) / 2
            ax.text(
                mid, 1.03, lbl, transform=ax.get_xaxis_transform(),
                rotation=90, ha="center", va="bottom", fontsize=7,
            )


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def make_velocity_figure(rows, t, segments, colors, title, out_path):
    import matplotlib.pyplot as plt

    v_cmd = [float(r["obs_7"]) for r in rows]
    v_meas = [float(r["velocity_mps"]) for r in rows]
    v_err = [m - c for m, c in zip(v_meas, v_cmd)]
    pos_err = [float(r["obs_0"]) * 0.5 for r in rows]

    fig, (ax_v, ax_verr, ax_pos) = plt.subplots(
        3, 1, figsize=(14, 10), constrained_layout=True, sharex=True,
    )
    shade_stages(ax_v, segments, t, colors, label=True)
    shade_stages(ax_verr, segments, t, colors)
    shade_stages(ax_pos, segments, t, colors)

    ax_v.plot(t, v_cmd, color="black", linewidth=1.0, linestyle="--", label="commanded")
    ax_v.plot(t, v_meas, color="tab:blue", linewidth=0.9, label="measured")
    ax_v.set_ylabel("linear velocity (m/s)")
    ax_v.legend(loc="upper right", fontsize=8)

    ax_verr.plot(t, v_err, color="tab:red", linewidth=0.8)
    ax_verr.axhline(0, color="black", linewidth=0.6)
    ax_verr.set_ylabel("velocity error (m/s)")
    ax_verr.text(
        0.01, 0.92, f"RMS error = {rms(v_err):.3f} m/s",
        transform=ax_verr.transAxes, fontsize=8, va="top",
    )

    ax_pos.plot(t, pos_err, color="tab:purple", linewidth=0.9)
    ax_pos.axhline(0, color="black", linewidth=0.6)
    ax_pos.axhline(0.5, color="gray", linewidth=0.6, linestyle=":")
    ax_pos.axhline(-0.5, color="gray", linewidth=0.6, linestyle=":")
    ax_pos.set_ylabel("position error (m)")
    ax_pos.set_xlabel("time (s)")

    fig.suptitle(f"{title}: linear velocity tracking", y=1.05)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"wrote {out_path}")


def make_yawrate_figure(rows, t, segments, colors, title, out_path):
    import matplotlib.pyplot as plt

    w_cmd = [float(r["obs_8"]) * 2.0 for r in rows]
    w_meas = [float(r["yaw_rate_world_radps"]) for r in rows]
    w_err = [m - c for m, c in zip(w_meas, w_cmd)]
    heading_err_deg = [
        math.degrees(math.atan2(float(r["obs_4"]), float(r["obs_5"]))) for r in rows
    ]

    fig, (ax_w, ax_werr, ax_head) = plt.subplots(
        3, 1, figsize=(14, 10), constrained_layout=True, sharex=True,
    )
    shade_stages(ax_w, segments, t, colors, label=True)
    shade_stages(ax_werr, segments, t, colors)
    shade_stages(ax_head, segments, t, colors)

    ax_w.plot(t, w_cmd, color="black", linewidth=1.0, linestyle="--", label="commanded")
    ax_w.plot(t, w_meas, color="tab:blue", linewidth=0.9, label="measured (world Z)")
    ax_w.set_ylabel("yaw rate (rad/s)")
    ax_w.legend(loc="upper right", fontsize=8)

    ax_werr.plot(t, w_err, color="tab:red", linewidth=0.8)
    ax_werr.axhline(0, color="black", linewidth=0.6)
    ax_werr.set_ylabel("yaw-rate error (rad/s)")
    ax_werr.text(
        0.01, 0.92, f"RMS error = {rms(w_err):.3f} rad/s",
        transform=ax_werr.transAxes, fontsize=8, va="top",
    )

    ax_head.plot(t, heading_err_deg, color="tab:purple", linewidth=0.9)
    ax_head.axhline(0, color="black", linewidth=0.6)
    ax_head.set_ylabel("heading error (deg)")
    ax_head.set_xlabel("time (s)")

    fig.suptitle(f"{title}: yaw-rate tracking", y=1.05)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"wrote {out_path}")


def make_leg_figure(rows, t, segments, colors, title, out_path, leg_degrees):
    import matplotlib.pyplot as plt

    leg_cmd = reconstruct_leg_commands(rows, t, leg_degrees)
    corner_names = ["leg 0 (left, +)", "leg 1 (left, -)", "leg 2 (right, +)", "leg 3 (right, -)"]

    fig, axes = plt.subplots(
        4, 2, figsize=(16, 14), constrained_layout=True, sharex=True,
    )

    for corner in range(4):
        measured = [math.degrees(float(r[f"leg_{corner}_angle_rad"])) for r in rows]
        commanded = [math.degrees(c[corner]) for c in leg_cmd]
        error = [m - c for m, c in zip(measured, commanded)]

        ax_cmd = axes[corner][0]
        shade_stages(ax_cmd, segments, t, colors, label=(corner == 0))
        ax_cmd.plot(t, commanded, color="black", linewidth=1.0, linestyle="--", label="commanded")
        ax_cmd.plot(t, measured, color="tab:blue", linewidth=0.8, label="measured")
        ax_cmd.set_ylabel(f"{corner_names[corner]}\nangle (deg)")
        if corner == 0:
            ax_cmd.legend(loc="upper right", fontsize=8)

        ax_err = axes[corner][1]
        shade_stages(ax_err, segments, t, colors, label=(corner == 0))
        ax_err.plot(t, error, color="tab:red", linewidth=0.8)
        ax_err.axhline(0, color="black", linewidth=0.6)
        ax_err.set_ylabel("error (deg)")
        ax_err.text(
            0.01, 0.88, f"RMS = {rms(error):.2f} deg",
            transform=ax_err.transAxes, fontsize=8, va="top",
        )

    axes[-1][0].set_xlabel("time (s)")
    axes[-1][1].set_xlabel("time (s)")
    fig.suptitle(
        f"{title}: leg angle tracking (assumes {leg_degrees:g} deg commanded amplitude)",
        y=1.03,
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"wrote {out_path}")


def make_figures(csv_path: Path, outdir: Path, title: str, leg_degrees: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("requires matplotlib: python -m pip install matplotlib") from exc

    rows = load_rows(csv_path)
    t0 = float(rows[0]["time_s"])
    t = [float(r["time_s"]) - t0 for r in rows]
    segments = segment_by_stage(rows)
    colors = plt.cm.tab20.colors

    stem = csv_path.stem
    make_velocity_figure(rows, t, segments, colors, title, outdir / f"{stem}_velocity_tracking.png")
    make_yawrate_figure(rows, t, segments, colors, title, outdir / f"{stem}_yawrate_tracking.png")
    make_leg_figure(rows, t, segments, colors, title, outdir / f"{stem}_leg_tracking.png", leg_degrees)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--title", default="")
    parser.add_argument(
        "--leg-degrees", type=float, default=45.0,
        help="DBG_testbench_leg_degrees used for this run (not logged in the CSV; default 45)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    make_figures(args.csv, args.outdir, args.title or args.csv.stem, args.leg_degrees)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
