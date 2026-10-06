# Matched hardware comparison — 7 October 2026

## Before scored runs

Complete the manual generation/build/target checks in [LOADING.md](LOADING.md). Save the known-working firmware and use a supervised standing check, then short low-speed forward/reverse and yaw commands for each new model. Record failures too. Stop a physically unstable candidate and retain that result; do not compensate with arm-specific gains or remove its failed trials.

Use the same robot, motor gains, current limit, sensor calibration, control period, firmware source and compiler configuration for both arms. Set identical surface, initial pose, payload placement, tire condition and leg offsets. Record battery voltage before/after each trial; recharge or pair trials at comparable charge. Check the actual flashed ELF and model used before assigning a label.

## Six full protocol runs

Run three paired repetitions with order **C–D, D–C, C–D**. Use the existing paper testbench unchanged for both arms. Set/check debugger Live Expressions:

```
DBG_testbench_linear_mps = 0.5
DBG_testbench_yaw_rate_radps = 1.0
DBG_testbench_leg_degrees = 35.0
```

Start telemetry capture first. Rearm in the usual supervised procedure, then set `DBG_testbench_start = 1` and keep it 1 through the protocol. The existing protocol includes settling, leg excursions, forward/reverse and hold, yaw turns and hold, symmetric leg motion while driving, asymmetric stance and single-leg motion. Reposition at its manual checkpoints, then set `DBG_testbench_continue = 1`; stop with `DBG_testbench_start = 0`. Log `DBG_testbench_waiting`, recovery and fall count. Testbench start and recovery reset references and C's hidden state. Do not carry C state across rearm/trials or reset it each tick.

Record all falls, aborted runs, recovery attempts and manual assistance with timestamp/stage. Manual repositioning between prescribed checkpoints is distinct from assistance during a scored stage. Keep raw data from both. If instability prevents completion, report completed seconds/stages and the failure; tracking figures alone must not hide that missing coverage.

## Capture commands

From the embedded repository root, install host capture dependencies if needed (`python -m pip install pyserial matplotlib numpy`). Set the real ST-Link VCP port: replace `COM3` below with the Windows port or `/dev/ttyACM0` on Linux. Use a unique filename for every trial; the wrapper refuses overwrites. Replace the ELF placeholder with the file actually flashed. If firmware source differs from the inspected base, supply `--firmware-commit` with that actual revision and archive the source/build diff.

```bash
python "AI_Policies/Paper 2026/cd_pilot_s43_2026-10-06/verify_package.py"
python "AI_Policies/Paper 2026/cd_pilot_s43_2026-10-06/capture_hardware_run.py" COM3 --arm C --repeat 1 --output Results/paper/cd_pilot_2026-10-07/C_repeat01.csv --firmware-elf "path/to/C_s43_i1150.elf"
python "AI_Policies/Paper 2026/cd_pilot_s43_2026-10-06/capture_hardware_run.py" COM3 --arm D --repeat 1 --output Results/paper/cd_pilot_2026-10-07/D_repeat01.csv --firmware-elf "path/to/D_s43_i1245.elf"
```

Repeat with `--repeat 2` / `repeat02` filenames in D–C order, then `--repeat 3` / `repeat03` in C–D order. Each capture runs until Ctrl+C; stop after the complete protocol and final hold. The wrapper uses the existing 921600-baud recorder and sends only its existing metric reset `R`. It does not command motors or change debugger settings. Its default declared settings are 0.5/1.0/35; if the matched experiment intentionally changes them, set both actual debugger values and wrapper arguments consistently.

The wrapper preserves original telemetry fields and adds arm, exact model SHA256, selected iteration, repeat and firmware identity to each CSV row and `.metadata.json` sidecar. **Model identity is operator-declared from the verified package and flashed build, not an on-device hash broadcast.** UART reports legacy ID only. A conflicting ID causes a nonzero result but preserves the capture. D still reports `range_mlp` in the legacy name field; use `comparison_arm=D` plus its hash.

Plot each run with the bundled existing tools:

```bash
python "AI_Policies/Paper 2026/cd_pilot_s43_2026-10-06/capture_tools/plot_testbench_tracking.py" Results/paper/cd_pilot_2026-10-07/C_repeat01.csv --outdir Results/paper/cd_pilot_2026-10-07/plots/C_repeat01 --title "C GRU s43 i1150 repeat 1" --leg-degrees 35
python "AI_Policies/Paper 2026/cd_pilot_s43_2026-10-06/capture_tools/plot_testbench_stages.py" Results/paper/cd_pilot_2026-10-07/C_repeat01.csv --output Results/paper/cd_pilot_2026-10-07/plots/C_repeat01_stages.png --title "C GRU s43 i1150 repeat 1"
```

Apply the same commands to D and remaining repetitions. Archive CSV, sidecar, ELF, generation/build reports, plots and video/event notes together.

## Additional matched disturbance checks

After full protocol runs, record separate C/D files for stationary pushes, pushes during the same low-speed command, payload hold and payload driving. Use the same payload mass/position and command/time schedule, varied paired C/D order and at least three repetitions. Record disturbance times, direction, contact location, payload mass/placement and manual intervention in adjacent event notes/video. Start/rearm each trial from the same state. Unmeasured hand pushes support a qualitative recovery comparison; do not describe them as equal measured forces. Retain the original and recovered reference trials separately with their own model/build hashes; the C/D wrapper is specifically for the new C/D package and must not label references as C.

## Paper reporting

Report falls, completion/survival time, recovery time, assistance and valid sample coverage first. Then compare velocity/yaw tracking error, zero-command position/yaw drift, current RMS/peaks and saturation, attitude, payload and push recovery using the same stage masks and exclusion rules for both arms. Mark waiting/repositioning explicitly. Preserve aborted trials and CRC/drop evidence; do not average only successful runs. Compare paired runs and show repeat variability. Do not combine hardware and simulator fall rates as equivalent: their detection rules and disturbances differ. This single training seed can establish a hardware development comparison; seed replication is separate work for a broader paper claim.
