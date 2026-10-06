# C/D simulation comparison and audit

Both selected policies have **zero falls, zero terminations/invalid states, 15.0 s survival and 100% sampled-environment coverage** on all 11 independent scenarios in each motor condition. Each condition covers 64 environments × 11 scenarios (704 environment-scenario trials) per arm, with 1000 steps at 15 ms. Attitude/tracking metrics are alive-masked; here survival coverage is complete. This is not evidence of zero physical falls.

## Selection and fair comparison

Fresh seed-43 training used 4096 environments, shared 64-step rollout, exploration floor 0.15 and stage budgets 300/350/200/250/150 (1250 updates). All 10 stage configurations and promotions were audited: promotion used the final checkpoint. C/D environments match except log directory; PPO algorithm settings match. Actor sizes are C 23,618 and D 23,492 parameters. Critic sizes differ (C 40,129; D 18,433), so the match concerns deployed actor capacity rather than total training capacity.

Four predetermined checkpoints (1096, 1150, 1175, 1245) were evaluated on validation base seed 1001 with canonical scenario seeds, generator terrain and recurrent resets. C passed 3 of 4, selecting iteration **1150** by the existing lowest passing score. D passed only iteration **1245**, selecting that final checkpoint. No gates or candidates were changed. Test base seed 2001 was held out from selection. Range and nominal evaluations use the same scenario seed base under different motor conditions; they are not two independent training-seed replications.

## Independent selected-policy results

Values below are arithmetic means of each scenario's reported RMS, not pooled sample-level RMS. Station drift is the station-keeping scenario's world drift only; commanded travel is not stationary drift.

| Motor condition / metric | C GRU, i1150 | D wide MLP, i1245 |
| --- | ---: | ---: |
| range: Velocity RMS (m/s) | 0.07333 | 0.10422 |
| range: Yaw-rate RMS (rad/s) | 0.06763 | 0.09146 |
| range: Station drift (m) | 0.05287 | 0.05995 |
| range: Current RMS (A) | 0.12494 | 0.11367 |
| nominal: Velocity RMS (m/s) | 0.07312 | 0.10380 |
| nominal: Yaw-rate RMS (rad/s) | 0.06605 | 0.08974 |
| nominal: Station drift (m) | 0.05237 | 0.05962 |
| nominal: Current RMS (A) | 0.12454 | 0.11237 |

Under range conditions, selected C has 29.6% lower mean velocity RMS and 26.1% lower mean yaw-rate RMS, with about 9.9% higher current RMS than D. Higher/lower current alone does not prove recovery strength or efficiency. Nominal results give the same broad tracking ordering.

C passes every gate on the independent selected-policy tests. D, despite passing all validation gates, misses **slow-reverse velocity RMS** (0.135 > 0.120 m/s) and **payload-hold world drift** (0.524 > 0.500 m range; 0.513 > 0.500 m nominal). Keep these test generalization misses. Do not reselect or tune using this held-out result.

C's fixed-budget final i1245 also has zero falls, but fails slow-reverse velocity RMS (0.129 range / 0.128 nominal > 0.120). Its station drift is 0.1131 / 0.1137 m versus selected C's 0.0529 / 0.0524 m. Final C must remain separately reported; D's selected and final checkpoint are the same. Raw final/selected JSON and logs are included for both.

## Scenario detail for selected policies

All rows below have zero falls, 15 s survival and full coverage for both arms. Paired entries are **C / D**. Drift is shown only for zero-command station/push/payload holds; driving displacement includes intended travel.

### Range motors

| Scenario | Velocity RMS (m/s) | Yaw RMS (rad/s) | Current RMS (A) | Hold drift (m) |
| --- | ---: | ---: | ---: | ---: |
| station_keeping | 0.0356 / 0.0362 | 0.0481 / 0.0573 | 0.1137 / 0.0931 | 0.0529 / 0.0600 |
| drive_forward_slow | 0.0845 / 0.1081 | 0.0660 / 0.1130 | 0.1273 / 0.1396 | — |
| drive_backward_slow | 0.0908 / 0.1350 | 0.0736 / 0.1184 | 0.1360 / 0.1654 | — |
| drive_forward | 0.0826 / 0.1105 | 0.0689 / 0.0750 | 0.1249 / 0.0933 | — |
| drive_backward | 0.0674 / 0.0861 | 0.0742 / 0.0794 | 0.1353 / 0.1255 | — |
| turn_in_place | 0.0610 / 0.0736 | 0.0667 / 0.1152 | 0.1333 / 0.0879 | — |
| drive_and_turn | 0.0704 / 0.1209 | 0.0931 / 0.0846 | 0.1285 / 0.0839 | — |
| push_while_still | 0.0509 / 0.0505 | 0.0514 / 0.0789 | 0.1263 / 0.1035 | 0.0579 / 0.0697 |
| push_while_driving | 0.0885 / 0.1390 | 0.0727 / 0.0794 | 0.1117 / 0.0970 | — |
| payload_while_still | 0.0602 / 0.1089 | 0.0572 / 0.1278 | 0.1197 / 0.1629 | 0.2399 / 0.5244 |
| payload_while_driving | 0.1147 / 0.1777 | 0.0722 / 0.0771 | 0.1177 / 0.0982 | — |

### Nominal motors

| Scenario | Velocity RMS (m/s) | Yaw RMS (rad/s) | Current RMS (A) | Hold drift (m) |
| --- | ---: | ---: | ---: | ---: |
| station_keeping | 0.0350 / 0.0339 | 0.0452 / 0.0553 | 0.1117 / 0.0881 | 0.0524 / 0.0596 |
| drive_forward_slow | 0.0846 / 0.1086 | 0.0641 / 0.1105 | 0.1284 / 0.1419 | — |
| drive_backward_slow | 0.0906 / 0.1350 | 0.0716 / 0.1173 | 0.1362 / 0.1652 | — |
| drive_forward | 0.0832 / 0.1099 | 0.0671 / 0.0738 | 0.1249 / 0.0902 | — |
| drive_backward | 0.0669 / 0.0874 | 0.0715 / 0.0782 | 0.1358 / 0.1281 | — |
| turn_in_place | 0.0599 / 0.0730 | 0.0672 / 0.1148 | 0.1332 / 0.0881 | — |
| drive_and_turn | 0.0703 / 0.1210 | 0.0926 / 0.0840 | 0.1280 / 0.0832 | — |
| push_while_still | 0.0504 / 0.0500 | 0.0486 / 0.0750 | 0.1266 / 0.1032 | 0.0487 / 0.0705 |
| push_while_driving | 0.0884 / 0.1392 | 0.0715 / 0.0766 | 0.1108 / 0.0954 | — |
| payload_while_still | 0.0605 / 0.1057 | 0.0550 / 0.1247 | 0.1174 / 0.1564 | 0.2475 / 0.5135 |
| payload_while_driving | 0.1146 / 0.1781 | 0.0721 / 0.0770 | 0.1169 / 0.0964 | — |

## Export and integration checks

The successful driver marker, both selection manifests, selected checkpoint hashes, ten resolved stage configurations and original export logs were checked. Native current-output validation max error: C 7.27e-6 A; D 6.56e-7 A. The C expanded STM32AI graph passed its 1024-step recurrent equivalence check (max current error 6.03e-6 A, hidden-state error 7.00e-6). `graph_audit.json` independently records ONNX checker and runtime interfaces, finite checkpoint weights and 64-step finite-output checks with persistent C state, all within ±2 A. D's STM32AI alias is byte-identical to its native current-output graph.

The actual embedded bindings, 13-observation layout, amperes contract, logical/physical routing, GRU persistence/reset, 15 ms period and motor gains were checked against embedded base 7ebd151. The package's capture helper was tested with simulated recorder output for C/D identity annotation, original telemetry preservation, legacy-ID mismatch detection and overwrite refusal; no serial device was used. No CubeAI toolchain was available. Target build, memory fit, latency, flashing and robot performance remain unverified.

## What can go in the paper

This development pilot supports a capacity-matched actor comparison in which recurrent C tracks commands better on the independent simulator suite, with complete survival coverage. It does not establish that the new C is more robust on real hardware than the original proven GRU, or that D cannot transfer. Tomorrow's matched hardware data can answer those questions. Retain original/recovered references separately and preserve all r2 results including C42; the pilot is not justification to drop an unfavorable seed. A broader architecture claim needs training-seed replication and matched hardware evidence.
