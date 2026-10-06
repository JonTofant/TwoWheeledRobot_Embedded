# GRU recovery candidate after the r2 hardware regression

The r2 GRU was reported to fall under small pushes, drift in velocity, and
command insufficient current on the real robot. This package adapts the
hardware-proven August GRU while preserving the corrected simulator structure.
Hardware testing of this candidate is pending.

## Model to generate

Select **`policy_drive_stm32ai.onnx`** for X-CUBE-AI generation of the existing
`nn_arm_c_range_gru` network. `policy_drive.onnx` is the native GRU reference.
Regenerate the network, build, and flash through the existing CubeAI workflow.
The firmware interface stays float32 `obs [1,13]`, `h_in [1,64]` to
`current_a [1,2]`, `h_out [1,64]`. Outputs are already logical left/right
amperes with `2*tanh` in the graph. Keep the existing +/-2 A current limit,
physical wiring mapping, and hidden-state threading/reset behavior.

## Recovery recipe

- Warm-start weights from the August hardware-proven `model_775.pt`, originally
  trained with seed 42; adaptation uses seed 43 for 150 stage-5 iterations.
- Restore `action_std_floor` from 0.05 to 0.15. This is training exploration;
  the deterministic ONNX contains the learned actor, without exploration noise.
- Use a fixed 5e-5 learning rate and fresh optimizer state for bounded adaptation.
- Preserve command-feedback observations, corrected mirrored-wheel torque
  limiting per physics substep, per-episode contact randomization, measured
  wheel damping, corrected hold reference, and independent benchmark GRU resets.
- Keep current and current-change reward weights at their August values,
  0.01 and 0.05 respectively.

The floor reduction is a suspected contributor, not an established cause of
hardware failure. Several plant and training choices also changed in r2.
This warm start is a development run with additional training; retain its
provenance separately from the four-arm r2 manuscript comparison.

## Validation records

The manifest records source checkpoint hashes, commands, training settings,
export hashes and pending hardware status. The export/conversion logs contain
PyTorch/native-ONNX and 1024-step native/expanded-GRU equivalence checks.
`focused_results.json` compares the old baseline, r2 C43 and this candidate
under the corrected plant: 64 environments, 15 seconds per scenario, flat
terrain, station keeping, forward/reverse driving, and pushes while stationary
and while driving. Initial random draws are matched across models; autoresets
after failures can change subsequent random consumption.
These simulator checks establish reproducible behavior and numerical integrity;
the physical recovery and tracking result must be established on the robot.
The source snapshot preserves the exact code used for the adaptation.

Run `sha256sum -c SHA256SUMS` inside this folder to verify the package.

## Hardware-proven rollback model

The original deployment graph is also included here as
**`rollback_old_gru_stm32ai.onnx`**. It is byte-identical to the existing embedded
`AI_Policies/Paper 2026/policy_arm_c_range_gru_stm32ai.onnx`.
Its SHA256 is
`d17ac3167ea1e2e5752424cda3f857bd27d39235b1e732662698f5098f371a46`.
Use that graph to regenerate the original network for an immediate return to
the previously tested controller. That restores the original policy weights;
it is separate from adapting those weights under the corrected simulator.
