# r2 seed-43 policies for the four-arm hardware comparison

All four arms use training seed 43 and the final `model_1395.pt` checkpoint at
one common training budget. These exports come from training commit
`d2e3992ec58658e058dbcd1b5839e340160547e5` in TwoWheeledRobot.

| Arm | Policy | ONNX to select for STM32 generation |
| --- | --- | --- |
| A | Point-estimate MLP [64,64] | `arm_A_point_s43.onnx` |
| B | Range-randomized MLP [64,64] | `arm_B_range_s43.onnx` |
| C | Range-randomized GRU(13,64) + MLP [64,64] | `arm_C_range_gru_s43_stm32ai.onnx` |
| D | Range-randomized MLP [145,145], matched to C actor size | `arm_D_range_wide_s43.onnx` |

The native `arm_C_range_gru_s43.onnx` is included as the numerical reference.
Select the `_stm32ai.onnx` version for C: its GRU is expanded into primitive
operations and its hidden-state input/output are flattened for the existing
STM32 interface.

## Interface and integration

- All models take float32 `obs` with shape `[1,13]` and output float32
  `current_a` with shape `[1,2]`, ordered logical left, logical right.
- Outputs are already amperes, with `2*tanh(actor_output)` inside the graph
  and a nominal +/-2 A limit. Do not apply this scaling a second time.
- Previous-current observation features must use the previous clamped logical
  policy commands divided by 2 A, consistent with the r2 training contract.
- The deployment graph for C also takes `h_in [1,64]` and produces `h_out [1,64]`.
  Feed `h_out` into the next `h_in`; reset to zero on boot, re-arm, fall, and
  test-bench checkpoint, rather than on every inference.
- Regenerate the corresponding X-CUBE-AI network code before building/flashing.
  The existing firmware selector has A/B/C; adding D requires its generated
  network and a fourth selector/inference path.

This folder supplies model artifacts; it does not change generated firmware,
select a new active policy, or flash the robot. Hardware validation remains
pending. The older August exports in the parent folder remain available.

## Evidence and provenance

`manifest.json` records checkpoint hashes, model choice, numerical validation,
and the observation/action contract. Absolute source paths in that manifest and
benchmark records identify the original training archive, not required deployment
locations. `interface_validation.json` records checked ONNX shapes and dtypes.
The four `arm_*_under_range_s43.json` records are simulation benchmarks: training
seed is 43, while their evaluation seed is 42.

The STM32-oriented C graph passed a 1024-step comparison against the native
reference: maximum action error 9.95e-6 A and hidden-state error 1.25e-5, both
below 1e-4. This is host numerical validation, not on-target validation.

The complete three-seed primary analysis retains C42. This common-seed hardware
comparison does not remove that failed training run from paper averages.

To check the copied source bundle, run inside this folder:

```sh
sha256sum -c SHA256SUMS
```

The checksum file covers the original 11 export/evidence files; this handoff
README is additional repository documentation.
