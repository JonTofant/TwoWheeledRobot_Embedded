# Payload mass + position estimator (laptop side)

Side quest, branch `sidequest/payload-mass-estimator`. The robot balances on the STM32 with
the ERK range-GRU and streams telemetry over the ST-Link USB port. This tool runs a small
sim-trained network on the laptop and shows **how heavy the object on the deck is and where
it sits**.

Nothing here has run on hardware yet. Everything below was checked in Isaac Sim and on
synthetic protocol frames only.

## What changed on this branch

| change | why |
|---|---|
| `X-CUBE-AI/App/nn_arm_c_range_gru*` restored from `e6d55c5` (2026-08-18) | That is the 08-10 seed-42 range GRU (`fixedstance_range_gru_2026-08-10/model_775.pt`): its weights match the checkpoint exactly. The estimator was trained against this policy. `feature/mdpi-actuators` had replaced it with the C/D-pilot GRU. Same graph and I/O, only the weights differ. |
| Paper telemetry **protocol v3** (`paper_metrics.{c,h}`, `main.c`, `Tools/paper_uart_capture.py`, `Docs/PAPER_TELEMETRY.md`) | Appends `leg_N_torque_Nm` (CyberGear MIT feedback torque) and `leg_N_target_rad` (the position target sent that tick) after `run_time_s`. The payload goes from 412 to 444 bytes, about 4.9 ms per 15 ms tick at 921600 baud. |
| `Tools/payload_estimator/` | This tool, the model and the sim reference data. |

The firmware changes compile cleanly (`arm-none-eabi-gcc -fsyntax-only`, including the
444-byte static assert). They have not been built in CubeIDE or flashed.

## Setup

1. Build and flash this branch's firmware. Keep `ACTIVE_NN_POLICY = POLICY_RANGE_GRU`.
2. On the laptop:

   ```bash
   cd Tools/payload_estimator
   python -m pip install -r requirements.txt
   ```

3. Plug in the Nucleo's ST-Link USB. The port is `/dev/ttyACM0` on Linux or `COM3` (etc.) on
   Windows. The baud rate is 921600, as for the paper capture.

## GUI (everything in one window)

```bash
python payload_gui.py            # or: --port COM7, --replay recordings/<file>.bin
```

Pick the ST-Link port (auto-selected when plugged in) and press **Connect** with an empty deck.
The **Checks** box shows whether v3 frames arrive (bytes without frames = old v2 firmware),
the firmware policy, and whether the robot is fallen. **1. Calibrate** walks through the six
placements below, marking each target spot on the deck view, and grades the result. **Apply,
save & tare** writes `calibration.json` (the old one becomes `calibration_previous.json`).
**2. Live** shows mass and position, with Tare (`t`) and Reset (`r`).

Each serial session is saved to `recordings/<time>.bin` (raw bytes, replayable with
`--replay` or `payload_viewer.py replay`). Next to it, `<time>.events.jsonl` logs every tare,
capture and calibration result with its telemetry sequence number. A calibration also writes
its report and leg map there. The CLI below does the same things without the window.

## 1. Calibrate once (about 3 minutes, needs one known mass, ~1 kg)

The sim knows its legs as front-left/front-right/back-left/back-right with its own signs.
The firmware comments suggest CyberGear index 0 = left-back, 1 = left-front, 2 = right-front
and 3 = right-back, with opposite signs, but that is unverified. Calibration measures the
mapping instead of trusting it.

```bash
python payload_viewer.py calibrate /dev/ttyACM0 --mass 1.0
```

Let the robot balance in place, with no joystick input. The script asks for: an empty deck,
then the mass at the centre, 8 cm forward, 8 cm back, 8 cm left and 8 cm right. FRONT is the
direction the robot drives for a positive velocity command. LEFT and RIGHT are as seen from
behind the robot, facing front. It then tries all 384 index/sign mappings against the sim's
leg-load signature and writes `calibration.json`.

What a good calibration looks like:
- **residual below about 0.3, and the runner-up much larger.** On sim data the right
  mapping scores about 0.05 and the next best about 0.28.
- **hardware torque reads close to 1.0× the sim.** Values far from 1.0 mean the CyberGear
  torque scale or the gearbox differs from the sim. Try `--use-gain`, which rescales by the
  measured gain.
- **effective stiffness close to 30 Nm/rad per joint** (−Δtorque/Δangle). That is the
  firmware kp, so it checks the torque feedback against the encoder. A much lower value
  suggests the torque reading misses gearbox friction.
- **per-placement table:** the hardware columns should follow the sim columns in sign and
  roughly in size.

If the residual is large for every mapping, the leg torque feedback probably does not carry
the load linearly (stiction or hysteresis). That is the main hardware risk identified in sim.

## 2. Run it live

```bash
python payload_viewer.py live /dev/ttyACM0 --record run1.bin
```

- **Start with an empty deck.** The first 3 s are an automatic tare. It maps every leg
  channel's empty reading onto the sim's, which removes CyberGear torque offsets. In
  synthetic tests a ±0.09 Nm offset costs 68 g without the tare and is fully removed by it.
- **Keys:** `t` re-tares (empty the deck first), `r` resets the estimator's memory, `q`
  quits.
- **The view:** top-down deck, FRONT up, the robot's LEFT on the left. The dashed ring is
  the typical sim location error for that mass. The right-hand plots show the mass history
  and the four leg-torque changes in the sim frame. The black marks show what 1 kg centred
  looks like in sim.
- **After a fall,** the estimator memory resets automatically. Re-tare with an empty deck.
- **Recordings:** `--record` writes the raw byte stream. You can replay it, or a CSV from
  `paper_uart_capture.py`, without the robot:

  ```bash
  python payload_viewer.py replay run1.bin
  python payload_viewer.py replay run1.bin --no-gui --out estimates.csv
  ```

## What to expect (sim, closed loop, unseen seed)

| object | mass error | location error (per axis) |
|---|---|---|
| 0.1 kg | ~14 g | ~4 cm (barely better than a guess) |
| 0.25 kg | ~14 g | ~2 cm |
| 0.5 kg | ~13 g | ~1 cm |
| 1 kg | ~13 g | ~0.6 cm |
| 2 kg | ~26 g | ~0.5 cm |

- **Placement:** mass accuracy does not depend on where the object is placed.
- **What "location" means:** the 2D centre of mass of everything on the deck. Two objects read
  as one, located at their common centre.
- **Limits:** above about 1.5 kg near the front or back edge, the balance policy itself starts
  to fall. Objects already on the deck at power-on read low (the tare needs an empty deck).

Full sim write-up: `TwoWheeledRobot` repo,
`docs/experiments/2026-11-03-payload-mass-estimator/README.md`.

## Files

- `payload_viewer.py`: the CLI and GUI (`calibrate`, `live`, `replay`).
- `payload_estimator.py`: the runtime, without the GUI. It handles frame→feature mapping,
  tare, the ONNX step and the calibration fit. It reuses `../paper_uart_capture.py` for frame
  sync and CRC.
- `model/estimator.onnx`: GRU(23→32) plus a 32→32→3 head, 6,627 parameters. Inputs are
  `features (1,23)` and `h_in (1,1,32)`. Outputs are `mass_kg`, `offset_xy_m` and `h_out`.
- `model/sim_reference.json`: feature order, the sim empty-deck means (the tare target), and
  the per-leg load signature fitted on 13.6k sim episodes (R² ≈ 0.99).
- `calibration.json`: written by `calibrate`. It is per robot.

The estimator's 23 inputs, built from each frame:
- `obs_0..12`, exactly as fed to the balance GRU;
- `action_left/right`, the logical current command;
- 4 leg angles (`angle − target`) and 4 leg torques, mapped onto the sim legs by
  `calibration.json`.

The physical `wheel_*_current_command_A` fields are not used: the firmware swaps their
destinations and signs.

Regenerate the model files from the `TwoWheeledRobot` repo (`experiment/payload-mass-estimator`):

```bash
python scripts/payload_mass/export_hardware_kit.py --out <this folder>/model
```
