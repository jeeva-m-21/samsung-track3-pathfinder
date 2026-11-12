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
    """Computes GT spatial spread (Null ATE) and 1s RMS displacement (Null RPE)."""
    gt = pd.read_csv(gt_path, sep=' ', comment='#', header=None)
    p = gt.iloc[:, 1:4].values
    t = gt.iloc[:, 0].values
    centroid = np.mean(p, axis=0)
    spread = float(np.sqrt(np.mean(np.sum((p - centroid)**2, axis=1))))
    
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
    return spread, rms_disp

def run_suite(binary: str = "./main"):
    results = {}
    print(f"| {'Sequence':<15} | {'Filter ATE':<11} | {'Null ATE':<10} | {'Filter RPE':<11} | {'Null RPE':<10} | {'Gate (ATE)':<10} | {'Gate (RPE)':<10} |")
    print("|" + "-"*17 + "|" + "-"*13 + "|" + "-"*12 + "|" + "-"*13 + "|" + "-"*12 + "|" + "-"*12 + "|" + "-"*12 + "|")
    
    for name, imu, gt in DATASETS:
        if not os.path.exists(gt):
            continue
        out_file = f"/tmp/eval_{name}.txt"
        subprocess.run([binary, imu, out_file], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        m = compute_trajectory_metrics(gt, out_file)
        null_ate, null_rpe = compute_null_baseline(gt)
        
        filter_ate = m["ate_rmse_aligned"]
        filter_rpe = m["rpe_trans_1s"]
        
        gate_ate = "PASS" if filter_ate <= null_ate else "FAIL"
        gate_rpe = "PASS" if filter_rpe <= null_rpe else "FAIL"
        
        results[name] = {
            "ate": filter_ate,
            "null_ate": null_ate,
            "rpe": filter_rpe,
            "null_rpe": null_rpe,
            "gate_ate": gate_ate,
            "gate_rpe": gate_rpe
        }
        
        print(f"| {name:<15} | {filter_ate:>11.4f} | {null_ate:>10.4f} | {filter_rpe:>11.4f} | {null_rpe:>10.4f} | {gate_ate:^10} | {gate_rpe:^10} |")
    
    return results

if __name__ == "__main__":
    run_suite()
