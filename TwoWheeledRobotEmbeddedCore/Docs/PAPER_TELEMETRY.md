# Paper experiment telemetry

The firmware streams one binary sample per 15 ms control release through USART2,
connected to the Nucleo ST-Link Virtual COM Port. This stream contains the raw
values needed for plots and the running mean, WCET, and jitter for every timing
row in Table 9 of the paper. While this experiment build is active, USART2 is not
available for the legacy ESP32/joystick packet receiver.

## Connection and capture

- Nucleo USB connection: ST-Link Virtual COM Port (COM3 on the current PC)
- STM32 USART2 TX/RX: PA2/PA3, already routed to ST-Link VCP on the Nucleo
- Serial format: 921,600 baud, 8 data bits, no parity, 1 stop bit

Capture a run with:

```powershell
python Tools/paper_uart_capture.py COM3 --duration 0 --reset `
  --output Results/paper/run_01.csv `
  --plot Results/paper/run_01.png
```

Install the PC-only dependencies if needed:

```powershell
python -m pip install pyserial matplotlib
```

Sending ASCII `R` or `r` to USART1 resets both the Table 9 timing accumulators and
the `MET_*` experiment accumulators. The capture tool sends this byte when
`--reset` is present. Capture starts immediately; perform the trial only after the
CSV sample timestamps and `run_time_s` restart near zero.

## Measurements

Every timing quantity has four CSV columns suffixed with `_last_us`, `_mean_us`,
`_wcet_us`, and `_jitter_us`. Jitter is the sample standard deviation since the
last reset. The quantity names are:

| CSV prefix | Paper quantity |
| --- | --- |
| `imu_read` | BNO08x service plus state-estimator update when new IMU reports arrive |
| `encoder_read` | DDSM115 reply decode and wheel-state update |
| `observation` | State/command calculation, observation assembly, normalization and finite-value limiting |
| `inference` | Active X-CUBE-AI network call |
| `action` | Output validation, recurrent-state copy when applicable, sign mapping and safety clamp |
| `motor_write` | Four CAN command submissions plus RS485 current-cycle queueing |
| `total` | Policy control dispatch through motor command submission; experiment metrics and UART logging are excluded |
| `control_period` | Time between TIM4 release interrupts (nominally 15,000 us) |
| `release_latency` | TIM4 release interrupt to start of the control dispatch |
| `rs485_cycle` | Current-cycle queueing to receipt of the second wheel reply |
| `imu_age` | Age of the latest complete IMU service at policy evaluation |
| `wheel_age` | Age of the older of the two wheel feedback channels |
| `leg_age` | Age of the oldest of the four CyberGear feedback channels |

The CSV also includes pitch/roll/yaw and rates, wheel odometry, both wheel encoder
states, command and measured wheel currents, four leg angles, the exact 13
normalized network inputs, two raw network outputs, and the running paper metrics:
initial/max/RMS pitch, RMS measured current, recovery time, final displacement, and
run time. Protocol version 2 additionally logs `yaw_rate_world_radps` (the complete
body gyro vector rotated into world Z and consumed by NN observation 6),
`yaw_rate_from_quat_radps` (an independent
finite difference using SH-2 timestamps), and the left/right wheel velocities after
the policy-layer direction signs. The raw body gyro and fused-yaw derivative remain
diagnostics; the world-Z rate is the policy input required by the training contract.

`flags` is a bit field: bit 0 means valid BNO data, bit 1 fallen, and bit 2
startup strategy active. Bits 8-9 hold the active policy ID (`0` point MLP,
`1` range MLP, `2` range GRU). Bit 3 means the automatic test bench is
running, bit 4 means it is paused (either a manual checkpoint or fall recovery),
and bit 5 distinguishes fall recovery. Bits 16-23 hold its stage ID and bits
24-31 hold the run's saturated fall count. The capture script decodes these into the
human-readable CSV columns `active_policy_name`, `testbench_running`,
`testbench_waiting`, `testbench_recovering`, `testbench_fall_count`,
`testbench_stage`, and `testbench_stage_name`.

## Automatic comparison test bench

Start the serial capture first, then write `DBG_testbench_start = 1` in Live
Expressions. The firmware resets the run/timing accumulators and executes the
same sequence for any of the three compiled policies. Write the variable back
to `0` to abort; it also returns to `0` automatically when the sequence ends.

The test waits at six manual checkpoints before the forward/reverse travel
segments. When `DBG_testbench_waiting` becomes `1`, wait for the robot to stop,
reposition it, then write `DBG_testbench_continue = 1`. Firmware clears the
continue flag automatically. It re-anchors the position and heading once when
entering a checkpoint so the robot holds position, then re-anchors again and
resets recurrent policy state before starting the next segment. Use
`--duration 0` and press Ctrl+C after the complete test because checkpoint time
depends on the operator.

Checkpoint samples remain in the CSV with `testbench_waiting = 1` and a named
checkpoint stage, so exclude them from motion-segment analysis. Repositioning
does not invalidate execution-time measurements. Motion results must be reported
as segmented trials, not as one continuous trajectory, and the same checkpoint
procedure must be used for every policy.

The editable Live Expressions amplitudes are:

- `DBG_testbench_linear_mps` (default `0.5` m/s)
- `DBG_testbench_yaw_rate_radps` (default `1.0` rad/s)
- `DBG_testbench_leg_degrees` (default `35` degrees relative to nominal)

Stage durations and ordering are kept together in `testbench_steps[]` near the
top of `Core/Src/main.c`. The four leg commands use opposed joint directions:
`[+left, -left, +right, -right]`. Logical zero is a physical 10-degree extension
on both legs, including during normal policy operation and manual checkpoints.
The 35-degree test amplitude is relative to this nominal pose, so a fully moving
joint reaches 45 physical degrees. A fall during an active motion marks that
stage as failed (`testbench_recovering = 1`), zeros the velocity and yaw-rate
targets, returns the legs to nominal, and pauses without aborting the run. Stand
the robot upright and write `DBG_testbench_continue = 1`; firmware advances past
the failed stage and resets recurrent policy state. The fall remains identifiable
in the CSV by the recovery flag, stage ID, and incremented fall count. At a manual
checkpoint the robot may be lifted or tilted for repositioning; the next segment
cannot start until it is upright again.

The final `crossed_asymmetric_hold` stage moves the left-back and right-front
actuators by 35 relative degrees over 1 second (45 physical degrees) while the
other two remain at the 10-degree nominal pose, then holds that crossed pose for
4 seconds.

The `sine_legs_forward` and `sine_legs_reverse` stages each complete two
2-second symmetric leg cycles while driving for 4 seconds, with a manual
repositioning checkpoint between directions.

After the crossed pose returns smoothly to nominal, `one_leg_sine_forward` and
`one_leg_sine_reverse` move only the left leg through the same 2-second sine
pattern for 4 seconds in each direction, again with a repositioning checkpoint
between them. The right leg stays at the 10-degree nominal pose.

## Filling Table 9

Use the final row of a clean capture after discarding warm-up. Copy the `_mean_us`,
`_wcet_us`, and `_jitter_us` values for each row. `inference_*` describes whichever
policy is compiled into that firmware image. Therefore, collect the MLP and GRU
inference rows from separate builds, while keeping the same compiler optimization,
clock configuration, telemetry setting, and trial length.

UART formatting/transmission starts only after `total` has been recorded. The
measurement therefore reports controller cost without including the paper logger's
own transport time. `dropped_frames` should remain zero; a nonzero value means the
previous interrupt-driven UART transmission had not completed before the next
sample and the capture is incomplete.
