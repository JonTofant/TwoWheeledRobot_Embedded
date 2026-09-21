# Results/paper

Telemetry captures and generated plots/tables for the MDPI *Actuators* paper's
hardware section. If you're picking this back up after a break: the raw
captures are the `*.csv` files below, `Tools/*.py` regenerates everything in
`comparison_plots/`, `wheel_asymmetry_plots/`, and `paper_figures/` from them,
and `TwoWheeledRobotEmbeddedCore/Docs/PAPER_TELEMETRY.md` has the full
operational detail (UART capture setup, CSV column reference, STM Studio Live
Expression names) if you need to record a new run.

## The 11-stage automatic test bench

Every `*_segmented.csv` / `*_testbench.csv` file is one continuous run of the
same firmware-driven test sequence, defined in `testbench_steps[]` near the
top of `Core/Src/main.c`. It exists so every policy gets pushed through an
identical, repeatable set of maneuvers instead of relying on a human joystick
operator to reproduce the same test by hand.

Trigger it by writing `DBG_testbench_start = 1` in a debugger's Live
Expressions view (or the equivalent STM Studio variable) while capturing
UART telemetry. The firmware then runs through the sequence below once and
returns `DBG_testbench_start` to `0` automatically when done, or immediately
if you write it back to `0` yourself.

11 labelled experiment stages, in order, each with the default commanded
amplitude (`DBG_testbench_linear_mps = 0.5` m/s, `DBG_testbench_yaw_rate_radps
= 1.0` rad/s, `DBG_testbench_leg_degrees = 35°` relative to the 10°-extended
nominal leg pose — all three are live-tunable before a run):

| # | Stage | Duration | What it commands |
|---|-------|----------|-------------------|
| 1 | `left_leg` | 5 s | Extend the left leg to +35° relative to nominal, hold; wheels stay at zero velocity/yaw. Pure balance-under-disturbance test. |
| 2 | `right_leg` | 5 s | Same, right leg only. |
| 3 | `forward` | 3 s | Drive straight forward at the commanded linear velocity, legs neutral. |
| 4 | `reverse` | 3 s | Drive straight backward. |
| 5 | `spin_positive` | 5 s | Spin in place at +yaw-rate, legs neutral. |
| 6 | `spin_negative` | 5 s | Spin in place at −yaw-rate. |
| 7 | `sine_legs_forward` | 4 s | Drive forward while both legs move through two full 2 s sine cycles together (symmetric leg motion as a disturbance while driving). |
| 8 | `sine_legs_reverse` | 4 s | Same sine-leg motion while driving backward. |
| 9 | `crossed_asymmetric_hold` | 5 s | Ramp the left-back and right-front actuators to +35° over 1 s while the other two joints stay nominal (a diagonally "crossed" pose), then hold for 4 s. Zero commanded velocity/yaw — balance-only. |
| 10 | `one_leg_sine_forward` | 4 s | Drive forward while only the left leg sine-cycles (right leg stays nominal) — an asymmetric disturbance. |
| 11 | `one_leg_sine_reverse` | 4 s | Same one-leg sine motion while driving backward. |

Between most of these, the firmware inserts short **neutral/hold** stages
(return legs to nominal, or hold the current velocity/heading for a few
seconds) that aren't counted among the 11 — they're transition padding, not
separate experiments. Six stages are preceded by a **manual checkpoint**
(`DBG_testbench_waiting = 1`): the robot holds position and waits for a human
to physically reposition it (the test arena isn't infinite), then the
operator writes `DBG_testbench_continue = 1` to resume. Checkpoint time is
operator-paced and excluded from the 11-stage timing analysis.

**Fall handling:** if the robot falls during an active stage, that stage is
marked failed (`testbench_recovering = 1`), commands zero out, legs return to
nominal, and the run pauses — it does not abort. Once the robot is stood back
up and `DBG_testbench_continue = 1` is written, the firmware advances to the
next stage (skipping the now-redundant return-to-neutral step) and resets any
recurrent policy state. This is exactly how Variant A (point MLP) shows up in
the data: it fell before the sequence could even leave `idle`, so its capture
has no stage-wise data at all — see `point_mlp_segmented.csv`.

## What each CSV is

| File | Policy | Notes |
|---|---|---|
| `range_mlp_segmented.csv` | Variant B (range-randomized MLP) | Full 11-stage run, no falls. |
| `range_gru_segmented.csv` | Variant C (range-randomized GRU) | Full 11-stage run, no falls. Baseline for the wheel-mismatch comparison below. |
| `range_gru_segmented_left_wheel_bigger.csv` | Variant C | Same sequence, left wheel swapped for one with 50% larger diameter — an out-of-distribution actuator-geometry probe. |
| `point_mlp_segmented.csv` | Variant A (point-estimate MLP) | Never leaves `idle` — fell before the test bench could start. |
| `range_mlp_testbench.csv`, `gru_testbench.csv`, `policy_test_gru.csv`, `gru_test.csv` | earlier/partial captures | Superseded by the `*_segmented.csv` runs above; kept for reference. |
| `yaw_test.csv`, `yaw_world_test.csv`, `yaw_action_swap_test.csv` | — | Earlier bench-top yaw-sign/convention debugging captures, not test-bench runs. |

## Regenerating the plots and tables

```bash
python Tools/plot_policy_comparison.py          # comparison_plots/: range_mlp vs range_gru
python Tools/plot_wheel_asymmetry.py             # wheel_asymmetry_plots/: matched vs +50% left wheel
python Tools/build_paper_hardware_results.py     # paper_figures/: manuscript-ready figures + timing table
python Tools/build_paper_hardware_tables.py      # paper_figures/: manuscript-ready result tables
```

Each script reads directly from the CSVs in this directory and overwrites its
own output subfolder; none of them modify the raw captures.
