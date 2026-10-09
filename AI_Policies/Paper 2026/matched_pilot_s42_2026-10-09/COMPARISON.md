# Completed seed42 physical-comparison pilot

Both arms spent 1250 allocated PPO updates. The MLP completed all stages, and benchmark selection chose model_1122.pt from stage5; it was not selected from the independent test. Its numeric ONNX equivalence passed (max absolute error 4.7683716e-7, tolerance 1e-4). The GRU passed native equivalence and 1024-step expanded recurrent equivalence. Neither model has been tested on hardware in this experiment.

MLP X-CUBE-AI import: `D_MLP/policy_drive_stm32ai.onnx`
GRU X-CUBE-AI import: `C_GRU/policy_drive_stm32ai.onnx`

MLP: 13 normalized observations and 2 outputs in amperes; no recurrent state. GRU: 13 normalized observations, current outputs and 64-float recurrent state. Preserve the established firmware input normalization, physical wheel signs and GRU reset handling. Inputs9/10 use previous logical commanded current, divided by2; measured motor current remains telemetry. X-CUBE-AI numerical import/build and robot behavior have not been verified here.

## Independent simulation comparison

Seed2001; 64 environments; 1000 steps (15s); identical scenario reseeding and full coverage. Falls and survival precede tracking figures because tracking/attitude metrics are alive-masked. Driving distance is not a station-keeping metric.

| Scenario | GRU falls % | MLP falls % | GRU survival s | MLP survival s | GRU velocity RMS error m/s | MLP velocity RMS error m/s |
|---|---:|---:|---:|---:|---:|---:|
| drive_and_turn | 0 | 0 | 15.000 | 15.000 | 0.0519 | 0.0791 |
| drive_backward | 0 | 0 | 15.000 | 15.000 | 0.0705 | 0.0931 |
| drive_backward_slow | 1.562 | 0 | 14.960 | 15.000 | 0.0548 | 0.0602 |
| drive_forward | 1.562 | 0 | 14.998 | 15.000 | 0.0784 | 0.1252 |
| drive_forward_slow | 0 | 0 | 15.000 | 15.000 | 0.0691 | 0.0572 |
| payload_while_driving | 4.688 | 6.25 | 14.863 | 14.757 | 0.1007 | 0.2070 |
| payload_while_still | 1.562 | 3.125 | 14.949 | 14.808 | 0.0555 | 0.1454 |
| push_while_driving | 6.25 | 0 | 14.816 | 15.000 | 0.0934 | 0.1070 |
| push_while_still | 0 | 0 | 15.000 | 15.000 | 0.0515 | 0.0645 |
| station_keeping | 0 | 0 | 15.000 | 15.000 | 0.0330 | 0.0546 |
| turn_in_place | 0 | 0 | 15.000 | 15.000 | 0.0377 | 0.0607 |

| Hold scenario | GRU drift m | MLP drift m |
|---|---:|---:|
| station_keeping | 0.0417 | 0.6546 |
| push_while_still | 0.0460 | 0.5195 |
| payload_while_still | 0.1397 | 1.4982 |

GRU passed all independent stage5 thresholds. MLP failed these independent thresholds and is a numerically validated comparison candidate, not simulation-qualified:

- station_keeping: world_drift_m 0.655 > 0.500
- push_while_still: world_drift_m 0.519 > 0.500
- payload_while_still: world_drift_m 1.498 > 0.500

The GRU held position and tracked forward commands better in this seed. The MLP had fewer falls under pushes while driving (0% vs6.25%); the outcome does not show uniform GRU superiority. Hardware A/B remains the next measurement.

## Selected checkpoint gate failures retained

Stage1 parent model275 (optimizer retained):
- station_keeping: world_drift_m 0.817 > 0.500

Stage2 model_525.pt: best benchmark-score fallback; all candidate gates failed.
- station_keeping: world_drift_m 0.524 > 0.500

Stage3 model_724.pt: best benchmark-score fallback; all candidate gates failed.
- station_keeping: world_drift_m 0.510 > 0.500
- push_while_still: world_drift_m 0.561 > 0.500

Stage4 model_973.pt: best benchmark-score fallback; all candidate gates failed.
- push_while_still: world_drift_m 0.509 > 0.500
- payload_while_still: world_drift_m 1.046 > 0.500

Stage5 model_1122.pt: best benchmark-score fallback; all candidate gates failed.
- station_keeping: world_drift_m 0.540 > 0.500
- push_while_still: world_drift_m 0.552 > 0.500
- payload_while_still: world_drift_m 1.018 > 0.500
- payload_while_driving: rms_vel_err_mps 0.243 > 0.227

Full manifests in comparison_evidence/stage*_selection.json retain every candidate and every candidate failure, including the original stage1 manifest with no promoted checkpoint.

## Comparison scope and provenance

Actors are parameter matched: GRU23618 vs MLP23492 ([145,145]); critics differ (GRU40129 recurrent vs MLP18433 feedforward). Equal allocated budgets are not identical training histories: GRU resumed model175 after its initial200 stage1 updates for a bounded100-update continuation; MLP completed300 stage1 updates and resumes model275. GRU stage promotion required passing candidates; the user authorized MLP best-score promotion despite failed gates. Later stages resume selected weights/optimizer, so checkpoint iteration labels do not count total allocated work. This is a single-seed development comparison, not a fully isolated test of recurrence or a multi-seed conclusion.

GRU original bundle: `C_GRU`
GRU checkpoint: `/home/jon/Projects/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-10-08_12-52-40_gru_continued_oldplant_floor005_s42_20261008T095911Z_stage5/model_999.pt`
GRU checkpoint SHA256: `a00121d7efd6b3754618bf02ed432dd8db13a79b033e294caf04bacb114db1fe`
MLP checkpoint SHA256: `5da0ebd8a62a7fdeaa7160382a7d7e1e6d8f6be10e32c9d42fa1802b07e61b2e`

All MLP recorded source hashes matched current files after training. Both model and ONNX hashes matched manifests. Stages2–5 resolved environment/agent configs were rechecked and copied here. Original failed artifacts are preserved. comparison_evidence/verification.json records checks and hashes.
