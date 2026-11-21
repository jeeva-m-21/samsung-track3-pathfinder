#!/usr/bin/env python3
"""
benchmark_suite.py - Track 3 Automated Evaluation Suite
with Null (Stationary) Baseline Pass/Fail Gate.

Evaluates candidate binary against:
1. Ground Truth using rigid SE(3) Umeyama alignment (evo_ape tum -a)
2. Null (Stationary) Baseline:
   - Null ATE: Spatial spread (radius of gyration) of GT around centroid
   - Null RPE 1s: RMS displacement of vehicle per 1 second
3. Enforces 'Beats Null' pass/fail gate on ATE and RPE.
"""

import subprocess, os, sys
from pathlib import Path
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics

DATASETS = [
    ('MH_01_easy', 'data/euroc/MH_01_easy/mav0/imu0/data.csv', 'data/euroc/gt/MH_01_easy.txt'),
    ('V1_01_easy', 'data/euroc/V1_01_easy/mav0/imu0/data.csv', 'data/euroc/gt/V1_01_easy.txt'),
    ('V2_02_medium', 'data/euroc/V2_02_medium/mav0/imu0/data.csv', 'data/euroc/gt/V2_02_medium.txt'),
    ('circle', 'data/synth/circle.csv', 'data/synth/circle_gt.txt'),
    ('figure_eight', 'data/synth/figure_eight.csv', 'data/synth/figure_eight_gt.txt'),
    ('TUM-VI room1', 'data/tumvi/room1/mav0/imu0/data.csv', 'data/tumvi/gt_room1.txt'),
    ('TUM-VI room2', 'data/tumvi/room2/mav0/imu0/data.csv', 'data/tumvi/gt_room2.txt'),
    ('UZH-FPV fwd7', 'data/uzh_fpv/imu_indoor_forward_7.csv', 'data/uzh_fpv/gt_indoor_forward_7.txt')
]

def compute_null_baseline(gt_path: str) -> tuple:
    """Computes like-with-like Null baselines:
    1. Null Start ATE: RMS distance from initial pose p(0) under start-anchoring.
    2. Null Aligned ATE: GT spatial spread around centroid under SE(3) Umeyama alignment.
    3. Null RPE 1s: RMS 1-second displacement of vehicle.
    """
    gt = pd.read_csv(gt_path, sep=' ', comment='#', header=None)
    p = gt.iloc[:, 1:4].values
    t = gt.iloc[:, 0].values
    
    # 1. Null Start ATE (static predictor fixed at starting point p0)
    p0 = p[0]
    null_start_ate = float(np.sqrt(np.mean(np.sum((p - p0)**2, axis=1))))
    
    # 2. Null Aligned ATE (static predictor globally shifted to centroid)
    centroid = np.mean(p, axis=0)
    null_aligned_ate = float(np.sqrt(np.mean(np.sum((p - centroid)**2, axis=1))))
    
    # 3. Null RPE 1s
    disp_1s = []
    idx2 = 0
    stride = max(1, len(t) // 2000)
    for idx1 in range(0, len(t), stride):
        t_target = t[idx1] + 1.0
        if t_target > t[-1]:
            break
        while idx2 < len(t) and t[idx2] < t_target:
            idx2 += 1
        if idx2 < len(t) and abs(t[idx2] - t_target) < 0.05:
            disp_1s.append(np.linalg.norm(p[idx2] - p[idx1]))
    rms_disp = float(np.sqrt(np.mean(np.array(disp_1s)**2))) if disp_1s else 0.0
    return null_start_ate, null_aligned_ate, rms_disp

def run_suite(binary: str = None):
    if binary is None:
        for candidate in ["main.py", "./main.py", "./main"]:
            if os.path.exists(candidate):
                binary = candidate
                break
        if binary is None:
            binary = "main.py"
    results = {}
    print(f"| {'Sequence':<15} | {'Start ATE (m)':<15} | {'Aligned ATE (m)':<17} | {'RPE 1s (m)':<13} | {'Span (Est/Ref)':<16} | {'Gate (Official)':<15} |")
    print("|" + "-"*17 + "|" + "-"*17 + "|" + "-"*19 + "|" + "-"*15 + "|" + "-"*18 + "|" + "-"*17 + "|")
    
    for name, imu, gt in DATASETS:
        if not os.path.exists(gt):
            continue
        out_file = f"/tmp/eval_{name}.txt"
        cmd = [sys.executable, binary, imu, out_file] if binary.endswith(".py") else [binary, imu, out_file]
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        m = compute_trajectory_metrics(gt, out_file)
        null_start, null_aligned, null_rpe = compute_null_baseline(gt)
        
        start_ate = m["start_anchored_rmse"]
        aligned_ate = m["ate_rmse_aligned"]
        filter_rpe = m["rpe_trans_1s"]
        est_span = m["est_span"]
        ref_span = m["ref_span"]
        span_str = f"{est_span:.1f}m / {ref_span:.1f}m"
        
        start_str = f"{start_ate:.2f} / {null_start:.2f}"
        aligned_str = f"{aligned_ate:.2f} / {null_aligned:.2f}"
        rpe_str = f"{filter_rpe:.2f} / {null_rpe:.2f}"
        
        is_degenerate = (est_span < 0.2 * ref_span) or (est_span < 0.1)
        if is_degenerate:
            gate = "FAIL (DEGEN)"
        elif aligned_ate <= null_aligned and filter_rpe <= null_rpe:
            gate = "PASS"
        elif aligned_ate <= null_aligned:
            gate = "PASS (ATE)"
        elif filter_rpe <= null_rpe:
            gate = "PASS (RPE)"
        else:
            gate = "FAIL"
        
        results[name] = {
            "start_ate": start_ate,
            "null_start": null_start,
            "aligned_ate": aligned_ate,
            "null_aligned": null_aligned,
            "rpe": filter_rpe,
            "null_rpe": null_rpe,
            "est_span": est_span,
            "ref_span": ref_span,
            "gate": gate
        }
        
        print(f"| {name:<15} | {start_str:>15} | {aligned_str:>17} | {rpe_str:>13} | {span_str:>16} | {gate:^15} |")
    
    return results

if __name__ == "__main__":
    run_suite()
