#!/usr/bin/env python3
"""Build the hardware-validation result tables for the paper.

Companion to build_paper_hardware_results.py (which builds the figures and
the DWT timing table). This produces the tabular counterpart to Table 10 /
Fig. 4 / Fig. 5 in the manuscript, from the same range_mlp_segmented.csv
(Variant B) and range_gru_segmented.csv (Variant C) captures, plus the
point_mlp_segmented.csv (Variant A) capture for the DNF row.

Outputs, in Results/paper/paper_figures/:
    table_hw_aggregate.csv / .tex        hardware analog of Table 10 (A/B/C)
    table_hw_stability_by_stage.csv/.tex per-stage RMS/max pitch + RMS current
    table_hw_tracking_by_scenario.csv/.tex  per-scenario RMS tracking error

Usage:
    python Tools/build_paper_hardware_tables.py
"""

from __future__ import annotations

import csv
import statistics
from pathlib import Path

import plot_policy_comparison as base
from build_paper_hardware_results import collect_all_metrics

PAPER_DIR = base.PAPER_DIR
OUT_DIR = PAPER_DIR / "paper_figures"
STAGES = base.STAGES

DISPLAY = {"range_mlp": "B (range DR, MLP)", "range_gru": "C (range DR, GRU)"}


def mean_sd(values: list[float]) -> tuple[float, float]:
    if len(values) < 2:
        return (values[0] if values else float("nan")), 0.0
    return statistics.mean(values), statistics.stdev(values)


def fmt(m: float, sd: float, digits: int) -> str:
    return f"{m:.{digits}f} $\\pm$ {sd:.{digits}f}"


# --------------------------------------------------------------- Table 1 --

def build_aggregate_table(all_metrics, out_csv: Path, out_tex: Path) -> None:
    vel_stages = [s[0] for s in STAGES if s[2] == "velocity" and s[0] in all_metrics]
    yaw_stages = [s[0] for s in STAGES if s[2] == "yaw" and s[0] in all_metrics]
    bal_stages = [s[0] for s in STAGES if s[2] == "balance" and s[0] in all_metrics]
    all_stages = [s[0] for s in STAGES if s[0] in all_metrics]

    def agg(label: str, stage_keys: list[str], metric: str) -> tuple[float, float]:
        return mean_sd([all_metrics[s][label][metric] for s in stage_keys])

    rows = []
    for label in ("range_mlp", "range_gru"):
        rms_pitch = agg(label, all_stages, "rms_pitch_deg")
        rms_cur = agg(label, all_stages, "rms_current_A")
        rms_ev = agg(label, vel_stages, "rms_velocity_error_mps")
        rms_ew = agg(label, yaw_stages, "rms_yawrate_error_radps")
        drift = agg(label, bal_stages, "max_abs_drift_m")
        rows.append((DISPLAY[label], "completed, 0 falls", rms_ev, rms_ew, drift, rms_pitch, rms_cur))

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "variant", "outcome",
            "rms_ev_mean_mps", "rms_ev_sd_mps",
            "rms_ew_mean_radps", "rms_ew_sd_radps",
            "drift_mean_m", "drift_sd_m",
            "rms_pitch_mean_deg", "rms_pitch_sd_deg",
            "rms_current_mean_A", "rms_current_sd_A",
        ])
        writer.writerow([
            "A (point, MLP)", "DNF: fell before test bench could start (idle only)",
            "", "", "", "", "", "", "", "", "", "",
        ])
        for name, outcome, ev, ew, dr, pi, cu in rows:
            writer.writerow([name, outcome, *ev, *ew, *dr, *pi, *cu])
    print(f"wrote {out_csv}")

    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\caption{Hardware aggregate metrics, embedded STM32F446RE deployment. Each row summarizes one "
                 r"single continuous run of the automatic test-bench sequence described in Sec.~2.8 (single run "
                 r"per variant; no multi-seed statistics as in Table~10). RMS values are mean $\pm$ s.d. across "
                 r"the relevant test-bench stages: RMS pitch and RMS current over all 11 stages, RMS $e_v$ over "
                 r"the 6 driving stages, RMS $e_{\dot\psi}$ over the 2 spin stages, and drift over the 3 "
                 r"pure-balance stages (left/right leg extension, crossed-asymmetric hold). Lower is preferable "
                 r"for every column.}")
    lines.append(r"\begin{tabularx}{\textwidth}{lXccccc}")
    lines.append(r"\toprule")
    lines.append(
        r"Variant & Outcome & RMS $e_v$ (m/s) & RMS $e_{\dot\psi}$ (rad/s) & Drift (m) & "
        r"RMS pitch ($^\circ$) & RMS current (A) \\"
    )
    lines.append(r"\midrule")
    lines.append(
        r"A (point, MLP) & DNF -- fell before the test bench could start (\texttt{isFallen()} blocked entry "
        r"past \texttt{idle}) & --- & --- & --- & --- & --- \\"
    )
    for name, outcome, ev, ew, dr, pi, cu in rows:
        lines.append(
            f"{name} & {outcome} & {fmt(*ev, 3)} & {fmt(*ew, 3)} & {fmt(*dr, 3)} & {fmt(*pi, 2)} & {fmt(*cu, 3)} \\\\"
        )
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabularx}")
    lines.append(r"\label{tab:hw-aggregate}")
    lines.append(r"\end{table}")
    out_tex.parent.mkdir(parents=True, exist_ok=True)
    out_tex.write_text("\n".join(lines) + "\n")
    print(f"wrote {out_tex}")


# --------------------------------------------------------------- Table 2 --

def build_stability_table(all_metrics, out_csv: Path, out_tex: Path) -> None:
    stage_keys = [s[0] for s in STAGES if s[0] in all_metrics]

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "stage", "duration_s",
            "B_rms_pitch_deg", "C_rms_pitch_deg",
            "B_max_pitch_deg", "C_max_pitch_deg",
            "B_rms_current_A", "C_rms_current_A",
        ])
        for s in stage_keys:
            b, c = all_metrics[s]["range_mlp"], all_metrics[s]["range_gru"]
            writer.writerow([
                base.STAGE_LABELS.get(s, s), f"{b['duration_s']:.2f}",
                f"{b['rms_pitch_deg']:.2f}", f"{c['rms_pitch_deg']:.2f}",
                f"{b['max_abs_pitch_deg']:.2f}", f"{c['max_abs_pitch_deg']:.2f}",
                f"{b['rms_current_A']:.3f}", f"{c['rms_current_A']:.3f}",
            ])
    print(f"wrote {out_csv}")

    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\caption{Hardware stability by test-bench stage, Variant B (range DR, MLP) vs. Variant C "
                 r"(range DR, GRU). RMS and max pitch are computed over the pitch signal within each stage; RMS "
                 r"current is the RMS of the mean absolute measured left/right wheel current. Lower is preferable "
                 r"in every column.}")
    lines.append(r"\begin{tabularx}{\textwidth}{Xccccc}")
    lines.append(r"\toprule")
    lines.append(
        r"\multirow{2}{*}{Stage} & \multirow{2}{*}{Dur. (s)} & \multicolumn{2}{c}{RMS / max pitch ($^\circ$)} & "
        r"\multicolumn{2}{c}{RMS current (A)} \\"
    )
    lines.append(r"\cmidrule(lr){3-4} \cmidrule(lr){5-6}")
    lines.append(r" & & B & C & B & C \\")
    lines.append(r"\midrule")
    for s in stage_keys:
        b, c = all_metrics[s]["range_mlp"], all_metrics[s]["range_gru"]
        lines.append(
            f"{base.STAGE_LABELS.get(s, s)} & {b['duration_s']:.1f} & "
            f"{b['rms_pitch_deg']:.1f} / {b['max_abs_pitch_deg']:.1f} & "
            f"{c['rms_pitch_deg']:.1f} / {c['max_abs_pitch_deg']:.1f} & "
            f"{b['rms_current_A']:.3f} & {c['rms_current_A']:.3f} \\\\"
        )
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabularx}")
    lines.append(r"\label{tab:hw-stability-by-stage}")
    lines.append(r"\end{table}")
    out_tex.parent.mkdir(parents=True, exist_ok=True)
    out_tex.write_text("\n".join(lines) + "\n")
    print(f"wrote {out_tex}")


# --------------------------------------------------------------- Table 3 --

def build_tracking_table(all_metrics, out_csv: Path, out_tex: Path) -> None:
    rows = []
    for stage_key, _title, kind in STAGES:
        if kind not in ("velocity", "yaw") or stage_key not in all_metrics:
            continue
        b, c = all_metrics[stage_key]["range_mlp"], all_metrics[stage_key]["range_gru"]
        if kind == "velocity":
            unit, key = "m/s", "rms_velocity_error_mps"
        else:
            unit, key = "rad/s", "rms_yawrate_error_radps"
        rows.append((base.STAGE_LABELS.get(stage_key, stage_key), kind, unit, b[key], c[key]))

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scenario", "kind", "unit", "B_rms_error", "C_rms_error"])
        for name, kind, unit, b_val, c_val in rows:
            writer.writerow([name, kind, unit, f"{b_val:.4f}", f"{c_val:.4f}"])
    print(f"wrote {out_csv}")

    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\caption{Hardware command-tracking error by test-bench scenario, Variant B (range DR, MLP) "
                 r"vs. Variant C (range DR, GRU) -- the hardware counterpart of Fig.~5. Velocity-command scenarios "
                 r"report RMS longitudinal-velocity error against the slew-limited commanded velocity (obs. index "
                 r"7); spin scenarios report RMS yaw-rate error against the slew-limited commanded yaw rate (obs. "
                 r"index 8). Lower is preferable in both columns.}")
    lines.append(r"\begin{tabularx}{\textwidth}{Xccc}")
    lines.append(r"\toprule")
    lines.append(r"Scenario & Unit & B RMS error & C RMS error \\")
    lines.append(r"\midrule")
    for name, kind, unit, b_val, c_val in rows:
        digits = 3 if unit == "m/s" else 4
        lines.append(f"{name} & {unit} & {b_val:.{digits}f} & {c_val:.{digits}f} \\\\")
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabularx}")
    lines.append(r"\label{tab:hw-tracking-by-scenario}")
    lines.append(r"\end{table}")
    out_tex.parent.mkdir(parents=True, exist_ok=True)
    out_tex.write_text("\n".join(lines) + "\n")
    print(f"wrote {out_tex}")


def main() -> int:
    runs = {
        "range_mlp": base.load_run("range_mlp", *base.DEFAULT_RUNS["range_mlp"]),
        "range_gru": base.load_run("range_gru", *base.DEFAULT_RUNS["range_gru"]),
    }
    all_metrics = collect_all_metrics(runs)

    build_aggregate_table(all_metrics, OUT_DIR / "table_hw_aggregate.csv", OUT_DIR / "table_hw_aggregate.tex")
    build_stability_table(
        all_metrics, OUT_DIR / "table_hw_stability_by_stage.csv", OUT_DIR / "table_hw_stability_by_stage.tex"
    )
    build_tracking_table(
        all_metrics, OUT_DIR / "table_hw_tracking_by_scenario.csv", OUT_DIR / "table_hw_tracking_by_scenario.tex"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
