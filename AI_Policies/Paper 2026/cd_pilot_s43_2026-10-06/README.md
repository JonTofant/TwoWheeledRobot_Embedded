# C/D development pilot — seed 43, 6 October 2026

The training driver finished successfully at 16:48:22 UTC. Both selected policies passed the predetermined validation gates and export checks. This package is ready for **manual CubeAI generation and firmware build**, followed by supervised hardware comparison. It does not contain built or flashed firmware. CubeAI, CubeIDE and `arm-none-eabi-gcc` were not available on the preparation host; STM32 compilation, RAM/flash fit, inference timing and target validation remain unchecked.

Start with [LOADING.md](LOADING.md), then [HARDWARE_TEST_PLAN.md](HARDWARE_TEST_PLAN.md). Simulation results and limitations are in [COMPARISON.md](COMPARISON.md). Run `python verify_package.py` before importing a model. All package files are covered by `SHA256SUMS`.

| Arm | Import into CubeAI | Selected iteration | Network name | Firmware selection |
| --- | --- | ---: | --- | --- |
| C, range GRU | [C_GRU/policy_drive_stm32ai.onnx](C_GRU/policy_drive_stm32ai.onnx) | 1150 | `nn_arm_c_range_gru` | `POLICY_RANGE_GRU` (ID 2) |
| D, wide range MLP | [D_Wide_MLP/policy_drive_stm32ai.onnx](D_Wide_MLP/policy_drive_stm32ai.onnx) | 1245 | `nn_arm_a_range` | `POLICY_RANGE_MLP` (ID 1) |

Both deployment graphs accept 13 float observations and return two wheel currents **already in amperes**, bounded by ±2 A. C additionally accepts and returns 64 float hidden-state values. Its STM32AI graph uses `[1,64]` hidden ports; native GRU ONNX uses `[1,1,64]`. Do not apply another tanh or multiply outputs by 2 in firmware.

Each arm includes the selected training checkpoint and resolved parameters, all five stages' resolved parameters and promotion records, four validation candidates' raw results, independent final/selected range/nominal results, native ONNX and TorchScript, current-output ONNX, deployment ONNX and export logs. `training_source/` snapshots Python/scripts at source commit `98658ede576b64a9d19ad7715622e45a3748e59a`; `firmware_contract_source/` records the actual integration at embedded commit `7ebd151ceab7bae8870ac4fb087947b63954289b`. The source snapshots are evidence, not a replacement project installation. The exact frozen protocol is [protocol.json](protocol.json).

The original hardware-proven GRU is preserved as [Rollback/rollback_old_gru_stm32ai.onnx](Rollback/rollback_old_gru_stm32ai.onnx), SHA256 `d17ac3167ea1e2e5752424cda3f857bd27d39235b1e732662698f5098f371a46`. It is distinct from the later `gru_recovery_2026-10-06` adapted model and this fresh C pilot. Keep your currently working firmware ELF as a separate rollback too. No active project model was replaced during preparation.

This is one development training seed, not the final multi-seed paper comparison. All earlier r2 results, including C42, remain intact. The new C's simulation advantage needs hardware confirmation; the user's previous hardware results belong to their original model identities.
