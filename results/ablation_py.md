# Python Reference Pipeline Ablation Results (Gate G2)

Evaluation comparing `naive`, `mahony`, `eskf_basic`, and `eskf_full` across degraded benchmark sequences.

## Per-Sequence Results

| Sequence              | Ablation   | ATE (m) | Blind-5s (m) | Dropout (m) | RPE 1s (m) | Start-Anchored (m) | Rot Err (°) | Objective J |
| --------------------- | ---------- | ------- | ------------ | ----------- | ---------- | ------------------ | ----------- | ----------- |
| circle_degraded       | naive      | 31.2347 | 85.1790      | 63.1271     | 9.3319     | 50.4558            | 25.28       | 1.0000      |
| circle_degraded       | mahony     | 18.3756 | 51.0292      | 35.8830     | 7.1073     | 29.2201            | 28.55       | 0.6145      |
| circle_degraded       | eskf_basic | 4.9672  | 7.1596       | 8.8182      | 2.3282     | 7.5777             | 100.60      | 0.1557      |
| circle_degraded       | eskf_full  | 4.9822  | 7.1299       | 8.7938      | 2.3263     | 7.5614             | 100.60      | 0.1558      |
| figure_eight_degraded | naive      | 19.9201 | 49.9989      | 34.4868     | 6.0926     | 28.9670            | 7.53        | 1.0000      |
| figure_eight_degraded | mahony     | 2.7609  | 6.9245       | 5.4706      | 1.8031     | 5.1256             | 7.10        | 0.1642      |
| figure_eight_degraded | eskf_basic | 4.4615  | 9.8575       | 6.4132      | 2.4733     | 6.0845             | 17.99       | 0.2421      |
| figure_eight_degraded | eskf_full  | 2.1776  | 4.4063       | 4.9983      | 1.1665     | 4.4224             | 20.47       | 0.1210      |

## Overall Summary (Mean Across Sequences)

| Sequence | Ablation   | ATE (m) | Blind-5s (m) | Dropout (m) | RPE 1s (m) | Start-Anchored (m) | Rot Err (°) | Objective J |
| -------- | ---------- | ------- | ------------ | ----------- | ---------- | ------------------ | ----------- | ----------- |
| MEAN     | naive      | 25.5774 | 67.5890      | 48.8069     | 7.7123     | -                  | -           | 1.0000      |
| MEAN     | mahony     | 10.5683 | 28.9769      | 20.6768     | 4.4552     | -                  | -           | 0.3893      |
| MEAN     | eskf_basic | 4.7144  | 8.5085       | 7.6157      | 2.4007     | -                  | -           | 0.1989      |
| MEAN     | eskf_full  | 3.5799  | 5.7681       | 6.8960      | 1.7464     | -                  | -           | 0.1384      |

## Gate G2 Acceptance Verification

- **All output trajectories passed trajectory validation** (`tools/validate_trajectory.py`): PASS
- **`eskf_full` beats `naive` baseline on ATE on all held-out sequences**: PASS
- **`eskf_full` beats `naive` baseline on Blind-5s RMSE on all held-out sequences**: PASS
- **Mean scalar objective J**: `eskf_full` (0.1384) vs `naive` (1.0000)
