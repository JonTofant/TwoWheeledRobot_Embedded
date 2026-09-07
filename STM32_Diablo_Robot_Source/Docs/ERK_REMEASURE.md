# ERK remeasurement — direct BNO08x build

Reviewer feedback asked for measurements longer than 8 s. This build is the ERK
firmware with its ESP32 IMU bridge replaced by a directly-wired BNO08x, so the
5/10/15/20-degree release tests can be repeated without that hardware.

**The control path is unchanged.** Same network, same `ann_in_data[8]` assembly,
same normalization scales, same `PITCH_SIGN`/`YAW_SIGN` constants, same
`STM32_TEST_CURRENT_LIMIT_A = 0.5`, same `LOG_`/`MET_` blocks. Only the source of
`roll_esp32`/`gx_esp32`/`yaw_esp32`/`gz_esp32` changed.

## What changed

| File | Change |
| --- | --- |
| `Core/Inc/bno08x.h`, `Core/Src/bno08x.c` | Copied from `TwoWheeledRobotEmbeddedCore` (I2C3 glue over the SH-2 stack) |
| `Core/Inc/SH2Sensorhub/`, `Core/Src/SH2Sensorhub/` | Vendored CEVA/Hillcrest SH-2 stack, copied unchanged |
| `Core/Src/main.c` | `IMU_UpdateFromBNO()` fills the legacy `*_esp32` globals; `BNO08x_Init()` after the DWT block; `BNO08x_Service()` at the top of the superloop; `HAL_GPIO_EXTI_Callback()` for PA1; USART1 reception left disarmed |
| `.cproject` | `../Core/Inc/SH2Sensorhub` added to the include path of all four build configs |
| `Diablo_robot_source.ioc` + `MX_I2C3_Init()` | I2C3 100 kHz -> 400 kHz (set in both, so a CubeMX regen keeps it) |

Cost: ~15 KB flash (63.6 -> 78.5 KB of 512 KB) and ~2.8 KB RAM (27.4 -> 30.3 KB
of 128 KB).

No new pin configuration was needed — `BNO080_INT` (PA1, EXTI falling),
`BNO080_RST` (PB15) and I2C3 (PA8/PC9) were already in the `.ioc`, and
`EXTI1_IRQHandler` already dispatches to the HAL callback.

## First power-up: bench check before the wheels touch the ground

Watch `LOG_theta_deg` and `bno_data_valid` in STM Studio with the robot held.

1. **Sensor alive** — `bno_data_valid` must go `1` within ~1 s of boot. If it
   stays `0`, the SH-2 stack never got its three reports: check I2C3 wiring and
   that the BNO's ADDR pin matches `BNO08X_I2C_ADDR` (currently `0x4B << 1`).
2. **Zero point** — hold the robot at its true upright balance point.
   `LOG_theta_deg` should read near 0. A standing offset here biases every
   number the paper reports (`rms_theta_deg`, `max_abs_theta_deg`) and shifts
   `isFallen()`'s +/-5 deg upright window, so note the value; shim the sensor if
   it is more than a couple of degrees.
3. **Pitch sign** — this is the one that decides whether the robot balances or
   throws itself over. Tilt the robot nose-down by hand and confirm
   `LOG_theta_deg` and `LOG_action_L` move together (both negative). The
   recorded ERK runs park at -5/-10/-15/-25 deg during the hold phase with both
   actions negative, so the same physical lean must give the same signs here.
   If `LOG_theta_deg` moves opposite to the old data, stop — the inferred mount
   rotation is wrong and the sign block in `main.c` needs revisiting, not a
   quick flip of `PITCH_SIGN`.
4. **Yaw coherence** — the channel that failed on the first attempt
   (2026-09-07). The angle and the rate the policy sees must agree, which you
   can check from any log without watching the robot:

       yaw_err_rad   = LOG_obs[4] * pi
       yaw_rate_rads = LOG_obs[5] * 4
       regress yaw_rate_rads against d(yaw_err_rad)/dt   ->  slope must be ~ +1

   The ERK reference runs give +0.59 .. +0.89. The first direct-BNO attempt gave
   **-0.87** (corr -0.90): raw body-frame `bno_gz` was being fed alongside a
   world-frame quaternion yaw. Fixed by projecting the gyro vector into world Z
   in `IMU_UpdateFromBNO()`. Exclude samples near a +/-pi wrap when regressing.

   A negative slope means the two are in different frames — a firmware bug, not
   a mounting question. A slope near +1 with the robot still spinning up means
   the pair is coherent but the polarity is inverted: flip `YAW_MOUNT_SIGN`.
   Both settings have now been tried on hardware and `+1.0f` is the correct one;
   if a future rewire changes the wheel mapping again, this is the knob.

   Keep the wheels off the ground until this reads ~ +1.

## Recording settings

- Set the STM Studio acquisition period to **15 ms**, matching the `LOG_` update
  tick. The ERK captures sampled at ~0.93 ms, so 91% of their rows are
  duplicates and 63 s cost 11 MB.
- Re-import the `.tsc` against the freshly built `.elf`. The variable addresses
  in the old logs (`0x20000d14`..`0x20000d4c`) have moved.
- Record **30 s per angle**, starting before the release. The old captures gave
  9.1 / 14.2 / 12.7 s of post-release balancing for 5/10/15 deg and no usable
  sustained segment at 20 deg.

## The mapping

Full reasoning and the supporting numbers are in the "BNO08x mount mapping"
comment block in `Core/Src/main.c` next to `PITCH_SIGN`. Short version:

**Pitch (theta, theta_dot) — straight across, no negation.** The old ESP32
reported euler *roll* (+rotation about its X) while the new code computes
`atan2(g_y,-g_z)` (-rotation about its X), and the mounting flipped X, so the
two negations cancel. Confirmed on hardware 2026-09-07: `LOG_obs[2]` matches
`LOG_theta_deg` exactly, and `d(action)/d(theta)` is +1.01 / +1.66, the same
positive sign as the ERK runs (+1.4 .. +4.6). The naming difference is just that
the training contract calls the balance axis "pitch"; it is the same physical
angle the ERK firmware called `roll_esp32`.

**Yaw (yaw, yaw_rate) — world frame, `YAW_MOUNT_SIGN = +1.0f`.** Both are taken
from the BNO's gravity-aligned world frame: quaternion yaw, and the full gyro
vector rotated into world Z (`root_ang_vel_w[:, 2]` in the training contract).
Raw body-frame `bno_gz` is *not* usable here — it is the opposite sense on this
mounting, which made the angle and the rate disagree. `YAW_MOUNT_SIGN` applies to
both together so the pair stays coherent whichever way it is set.

With a coherent pair the loop reduces to `YAW_MOUNT_SIGN * k < 0`, where `k` is
the physical yaw actuation polarity. Two hardware runs on 2026-09-07 fixed
`k = -1`, so `+1.0f` is the stable setting. `k = -1` means this chassis' yaw
response is **inverted relative to the ERK era** — the same finding that made
`TwoWheeledRobotEmbeddedCore` cross its wheel destinations at the actuator.
Crossing there and flipping the sign here are equivalent for yaw, and neither
affects forward drive (`ddsm[0] - ddsm[1] = aL + aR` either way). An earlier
version of this document said not to copy that crossing; that was wrong — the
hardware needs the inversion, this project just applies it on the sensor side.
