#!/usr/bin/env python3
import subprocess, os, sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics

DATASETS = [
    ('MH_01_easy', 'data/euroc/MH_01_easy/mav0/imu0/data.csv', 'data/euroc/gt/MH_01_easy.txt'),
    ('V1_01_easy', 'data/euroc/V1_01_easy/mav0/imu0/data.csv', 'data/euroc/gt/V1_01_easy.txt'),
    ('V2_02_medium', 'data/euroc/V2_02_medium/mav0/imu0/data.csv', 'data/euroc/gt/V2_02_medium.txt'),
    ('circle', 'data/synth/circle.csv', 'data/synth/circle_gt.txt'),
    ('figure_eight', 'data/synth/figure_eight.csv', 'data/synth/figure_eight_gt.txt')
]

def run_suite():
    results = {}
    print(f'| {"Sequence":<15} | {"ATE RMSE Aligned (m)":<22} | {"RPE 1s (m)":<12} |')
    print('|' + '-'*17 + '|' + '-'*24 + '|' + '-'*14 + '|')
    for name, imu, gt in DATASETS:
        out_file = f'/tmp/eval_{name}.txt'
        subprocess.run(['./main', imu, out_file], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        m = compute_trajectory_metrics(gt, out_file)
        results[name] = m
        print(f'| {name:<15} | {m["ate_rmse_aligned"]:>22.4f} | {m["rpe_trans_1s"]:>12.4f} |')
    return results

if __name__ == '__main__':
    run_suite()
