# Manual CubeAI generation and loading

## Verified integration and remaining work

Use the embedded `feature/mdpi-actuators` firmware and its deployment contract. The inspected base is `7ebd151ceab7bae8870ac4fb087947b63954289b`. The user's current Projects checkout is on another branch with unrelated uncommitted Results work: do not reset, stash or switch that checkout to obtain this package. Use a separate firmware checkout/worktree or your established working CubeIDE project with the matching integration.

The active firmware is `TwoWheeledRobotEmbeddedCore/Core/Src/main.c`; its named `nn_arm_*` integration uses 13 observations and 2 current outputs. Older 18-observation/6-action guidance in AGENTS.md is stale for this branch. Generated active network files and a Debug build directory are not committed on this branch. The `.ioc` references X-CUBE-AI 8.1.0 and STM32F446RETX. No CubeAI generation, compiler build, target validation or flash was performed here.

## Produce separate C and D builds

1. Verify this package with `python verify_package.py`. Save the current working ELF, CubeAI artifacts, project configuration and hashes before importing either candidate. Keep independent copies of the same firmware project for C and D, with the same build configuration and compiler options.
2. Open the existing `TwoWheeledRobotEmbeddedCore/Diablo_robot_source.ioc` / project in STM32CubeIDE. Use the established X-CUBE-AI workflow. Fix the local model paths in the graphical tool; do not manually change cached model hashes or activation byte counts in the `.ioc`.
3. For C, import `C_GRU/policy_drive_stm32ai.onnx` as **`nn_arm_c_range_gru`**. Keep `ACTIVE_NN_POLICY` set to `POLICY_RANGE_GRU` in `main.c`. For D, import `D_Wide_MLP/policy_drive_stm32ai.onnx` into the existing **`nn_arm_a_range`** slot and set `ACTIVE_NN_POLICY` to `POLICY_RANGE_MLP`. D replaces that slot's model weights and generated sizes; merely selecting the old range model would test B, not D. D will retain legacy telemetry ID 1/name `range_mlp`; the capture sidecar and added CSV arm/hash fields distinguish it.
4. Analyze and host-validate each float32 model in CubeAI, then generate its named network artifacts for the existing target/runtime. Check generated buffer macros and binding order: input 0 = `obs` `[1,13]`; output 0 = `current_a` `[1,2]`. C also requires input 1 = `h_in` `[1,64]`, output 1 = `h_out` `[1,64]`. If the generator changes tensor order or names, inspect its report and reconcile bindings before running. Record the generation report, RAM/flash requirements, runtime/tool version and model hash for each build.
5. Preserve customized `app_x-cube-ai.c/h` and the existing boot/inference glue. Copy only the applicable named network code/data/headers and any required generated weight files after reviewing the diff. All three named network headers are included unconditionally in `main.c`; retain or generate the other established network artifacts required for compilation. Do not replace application glue with a fresh ApplicationTemplate or edit generated makefiles by hand. Let CubeIDE regenerate build metadata and activation sizes.
6. Build both variants with the same Debug/Release configuration. Retain separate `C_s43_i1150.elf` and `D_s43_i1245.elf` files and their SHA256 hashes. Review the actual build diff: policy weights/generated sizes and `ACTIVE_NN_POLICY` are the intended differences. Keep motor tuning, sensor frames, signs, normalization, command slew, reference anti-windup and timing identical.
7. Flash each variant manually via ST-Link. Confirm legacy ID 2 for C and ID 1 for D, finite observations/currents, target inference plus control work within the 15 ms period, and adequate stack/activation memory. Begin with the supervised standing/low-speed check in the test plan. Record timing and any overrun evidence. A successful ONNX host check does not establish target timing or physical stability.

## Contract to preserve

- Control period: 15 ms (66.7 Hz). CyberGear gains: kp 30, kd 3. Preserve the actual firmware's physical wheel routing/signs; the simulator's mirrored left wheel is not a wiring instruction.
- Observation order: position error, velocity, pitch, pitch rate, yaw sine, yaw cosine, yaw rate, velocity command, yaw command, previous logical left/right current, roll, roll rate. Existing scales/clips in `ANN_Run()` belong to both variants.
- Currents are logical left/right, already scaled to ±2 A. Firmware preserves the established actuator-boundary routing and finite checks. Previous-action observations use logical current / 2.
- C keeps 64 hidden floats between control ticks. Boot/rearm, falling-reference reset and testbench start/recovery reset the hidden state. Do not clear it every tick. The UART `R` command resets metrics; it does **not** rearm/reset GRU state.
- Preserve command slew (1 m/s² and 4 rad/s²), position-reference anti-windup ±0.5 m, neutral leg pose and fall logic. Do not tune either arm separately for this comparison.

The branch-specific contract snapshot is [firmware_contract_source/STM32_DEPLOYMENT.md](firmware_contract_source/STM32_DEPLOYMENT.md); `main.c` in the same snapshot records the bindings and reset logic actually audited.

## Rollback and identities

Original hardware-proven GRU deployment SHA256:
`d17ac3167ea1e2e5752424cda3f857bd27d39235b1e732662698f5098f371a46`.

Use `Rollback/rollback_old_gru_stm32ai.onnx` through the GRU slot if a rebuild is needed; prefer your saved known-working ELF for immediate rollback. Keep the recovered/adapted policy under the existing `gru_recovery_2026-10-06` package separately. Do not label that warm-start adaptation as this fresh seed-43 pilot. The full new C/D hashes are in `manifest.json` and `SHA256SUMS`.
