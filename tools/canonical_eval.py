#!/usr/bin/env python3
"""tools/canonical_eval.py - Canonical 50-Run Evaluator for Track 3."""

import os
import sys
import time
import subprocess
import tempfile
import argparse
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np

# Ensure tools.metrics is imported from reference worktree
REF_DIR = Path('/home/jeeva/projects/samsung_hackathon_wt/E00')
if str(REF_DIR) not in sys.path:
    sys.path.insert(0, str(REF_DIR))
from tools.metrics import compute_trajectory_metrics

CACHE_DIR = Path('/home/jeeva/projects/samsung_hackathon_wt/E00/reports/runs/degraded_cache')
SEQS = ['MH_01_easy', 'V1_01_easy', 'V2_02_medium', 'room1', 'room2']
SEEDS = list(range(1, 11))

def run_one(args):
    seq, seed, script_path = args
    case_dir = CACHE_DIR / f"{seq}_s{seed}"
    imu = case_dir / "imu0" / "data.csv"
    gt = case_dir / "groundtruth.txt"
    sidecar = case_dir / "degrade_info.json"
    
    with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as tf:
        out_path = tf.name
    try:
        t0 = time.perf_counter()
        cmd = ['python3', script_path, str(imu), out_path]
        res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        t_elapsed = time.perf_counter() - t0
        if res.returncode != 0:
            return None
        
        m = compute_trajectory_metrics(gt, out_path, sidecar_path=sidecar)
        dur = m.get('duration_sec', 140.0)
        ms_per_s = (t_elapsed / max(dur, 1.0)) * 1000.0
        
        return {
            'seq': seq,
            'seed': seed,
            'ate': float(m['ate_rmse_aligned']),
            'blind5': float(m['blind5_rmse']),
            'gap2s': float(m['gap_2s_rmse']),
            'rpe1s': float(m.get('rpe_trans_1s', 0.0)),
            'l_ratio': float(m.get('length_ratio', 1.0)),
            'ms_per_s': ms_per_s
        }
    finally:
        if os.path.exists(out_path):
            os.remove(out_path)

def evaluate(script_path='/home/jeeva/projects/samsung_hackathon/VITC_SingleThread/src/main.py', workers=8):
    tasks = [(s, sd, script_path) for s in SEQS for sd in SEEDS]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(run_one, tasks))
    
    results = [r for r in results if r is not None]
    if len(results) != 50:
        print(f"Warning: only {len(results)}/50 completed successfully!")
    
    ate_mean = float(np.mean([r['ate'] for r in results]))
    blind_mean = float(np.mean([r['blind5'] for r in results]))
    gap_mean = float(np.mean([r['gap2s'] for r in results]))
    rpe_mean = float(np.mean([r['rpe1s'] for r in results]))
    l_mean = float(np.mean([r['l_ratio'] for r in results]))
    lat_mean = float(np.mean([r['ms_per_s'] for r in results]))
    
    per_seq = {}
    print(f"Overall (50 runs): ATE={ate_mean:.3f}m | Blind5s={blind_mean:.3f}m | Gap2s={gap_mean:.3f}m | RPE1s={rpe_mean:.3f}m/s | L_ratio={l_mean:.3f} | Lat={lat_mean:.2f}ms/s")
    for s in SEQS:
        s_res = [r for r in results if r['seq'] == s]
        per_seq[s] = {
            'ate': float(np.mean([r['ate'] for r in s_res])),
            'blind5': float(np.mean([r['blind5'] for r in s_res])),
            'gap2s': float(np.mean([r['gap2s'] for r in s_res])),
            'rpe1s': float(np.mean([r['rpe1s'] for r in s_res])),
            'l_ratio': float(np.mean([r['l_ratio'] for r in s_res]))
        }
        print(f"  {s:15s}: ATE={per_seq[s]['ate']:.3f}m | Blind={per_seq[s]['blind5']:.3f}m | Gap={per_seq[s]['gap2s']:.3f}m | RPE={per_seq[s]['rpe1s']:.3f}m/s | L_ratio={per_seq[s]['l_ratio']:.3f}")
    
    return {
        'ate': ate_mean,
        'blind5': blind_mean,
        'gap2s': gap_mean,
        'rpe1s': rpe_mean,
        'lat_ms_per_s': lat_mean,
        'per_seq': per_seq,
        'results': results
    }

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--script', default='/home/jeeva/projects/samsung_hackathon/VITC_SingleThread/src/main.py')
    parser.add_argument('--workers', type=int, default=8)
    args = parser.parse_args()
    evaluate(args.script, args.workers)
