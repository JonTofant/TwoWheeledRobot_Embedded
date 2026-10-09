# Matched seed42 pilot for hardware comparison — 2026-10-09

Use these exact files for X-CUBE-AI import:

| Controller | Import file | Existing network name | Firmware selector |
|---|---|---|---|
| GRU | [C_GRU/policy_drive_stm32ai.onnx](C_GRU/policy_drive_stm32ai.onnx) | `nn_arm_c_range_gru` | `POLICY_RANGE_GRU` (2) |
| Matched MLP | [D_MLP/policy_drive_stm32ai.onnx](D_MLP/policy_drive_stm32ai.onnx) | `nn_arm_d_range_matchedsize_mlp` | `POLICY_RANGE_WIDE_MLP` (3) |

Regenerate these two networks in the existing CubeIDE/X-CUBE-AI project under the same network names before building and flashing. The generated C networks currently committed to the firmware remain the previous models; copying ONNX files alone does not update them. Preserve the customized integration in `TwoWheeledRobotEmbeddedCore/X-CUBE-AI/App/app_x-cube-ai.c`.

Both use 13 normalized observations and two logical current commands in amperes, already scaled to ±2 A. Do not apply another tanh or current gain. Previous-current inputs9/10 are previous logical commanded currents divided by2. GRU uses persistent64-float hidden state; retain existing rearm/testbench reset handling. MLP has no hidden state. Keep existing physical wheel signs, 15ms inference period and leg gains30/3.

Both spent1250 allocated training updates. Benchmark-selected final checkpoints: GRU999 and MLP1122. Export numerical equivalence passed for both, including1024-step GRU expansion validation. GRU passed independent simulation gates; MLP failed three stationary drift gates and is an exploratory hardware-comparison candidate. Neither this pair's hardware behavior nor a new X-CUBE-AI build is verified yet.

[COMPARISON.md](COMPARISON.md) reports all11 independent scenarios, falls/survival before masked tracking, failed gates, and training-history/critic differences. The D_MLP/comparison_evidence directory includes every stage selection manifest and resolved stage2–5 configurations. Bundle manifests retain original simulator provenance paths. SHA256SUMS verifies every handoff file.
