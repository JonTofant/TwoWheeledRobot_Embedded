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
python Tools/paper_uart_capture.py COM3 --duration 30 --reset `
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
`1` range MLP, `2` range GRU).

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
