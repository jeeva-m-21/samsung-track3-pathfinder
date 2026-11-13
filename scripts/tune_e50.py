#!/usr/bin/env python3
"""tune_e50.py - Hyperparameter Optimization & LOOCV Benchmark for EXP-E50.

Frozen estimator structure: optimizes 11 physical parameters across dev sequences.
Follows Track 3 5-fold Leave-One-Out Cross-Validation (LOOCV) protocol on seeds 1-10.
Evaluates composite objective:
  J = 0.30 * (ATE / ATE_0) + 0.20 * (Blind / Blind_0) + 0.10 * (Gap / Gap_0)
where ATE_0 = 2.309264 m, Blind_0 = 4.783184 m, Gap_0 = 6.200439 m (best-006 baselines).
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import optuna

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics
from tools.eval_degraded_harness import REAL_SEQUENCES, DegradedHarness

# Baseline denominators from best-006
ATE_0 = 2.309264
BLIND_0 = 4.783184
GAP_0 = 6.200439

DEV_SEEDS = list(range(1, 11))
DEV_NAMES = ["MH_01_easy", "V1_01_easy", "V2_02_medium", "room1", "room2"]

CACHE_DIR = Path("/tmp/degraded_dev_cache")

# Parameter definitions and bounds
PARAM_BOUNDS = {
    "shrink_v": (0.5, 1.0),
    "tau_v": (1.0, 5.0),
    "stationary_v_thresh": (0.01, 0.15),
    "rotor_drag_kd": (0.05, 0.5),
    "rotor_drag_sigma": (0.2, 1.5),
    "ped_sigma_lat": (0.2, 1.0),
    "ped_sigma_vert": (0.1, 0.8),
    "max_velocity_mps": (2.0, 4.5),
    "velocity_bounding_sigma": (0.1, 0.5),
    "tilt_tau_a": (0.1, 0.5),
    "tilt_sigma": (0.01, 0.08),
}


def load_base_params() -> Dict[str, Any]:
    params_path = REPO_ROOT / "configs" / "params.json"
    with open(params_path, "r", encoding="utf-8") as f:
        return json.load(f)


def export_and_build(params_dict: Dict[str, Any]) -> None:
    """Export params to params.json and src/params.h, then compile binary."""
    params_json = REPO_ROOT / "configs" / "params.json"
    with open(params_json, "w", encoding="utf-8") as f:
        json.dump(params_dict, f, indent=2)

    export_script = REPO_ROOT / "tools" / "export_params.py"
    subprocess.run(
        [sys.executable, str(export_script), "--input", str(params_json), "--output", str(REPO_ROOT / "src" / "params.h")],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    src_dir = REPO_ROOT / "src"
    subprocess.run(["make", "-j8"], cwd=str(src_dir), check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_single_eval_worker(args: Tuple[str, int]) -> Tuple[float, float, float]:
    seq_name, seed = args
    case_dir = CACHE_DIR / f"{seq_name}_s{seed}"
    imu = case_dir / "imu0" / "data.csv"
    gt = case_dir / "groundtruth.txt"
    sidecar = case_dir / "degrade_info.json"

    with tempfile.TemporaryDirectory() as td:
        out_traj = Path(td) / "eval_est.txt"
        cmd = [str(REPO_ROOT / "main"), str(imu), str(out_traj), "--append-blind"]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        m = compute_trajectory_metrics(gt, out_traj, sidecar_path=sidecar)
        return float(m["ate_rmse_aligned"]), float(m["blind5_rmse"]), float(m["gap_2s_rmse"])


def evaluate_cases(cases: List[Tuple[str, int]], pool: ProcessPoolExecutor) -> Tuple[float, float, float, float]:
    """Evaluate a list of (seq_name, seed) pairs and return (mean_ate, mean_blind, mean_gap, composite_J)."""
    results = list(pool.map(run_single_eval_worker, cases))

    ates = [r[0] for r in results]
    blinds = [r[1] for r in results]
    gaps = [r[2] for r in results]

    mean_ate = float(np.mean(ates))
    mean_blind = float(np.mean(blinds))
    mean_gap = float(np.mean(gaps))

    J = 0.30 * (mean_ate / ATE_0) + 0.20 * (mean_blind / BLIND_0) + 0.10 * (mean_gap / GAP_0)
    return mean_ate, mean_blind, mean_gap, J


def run_loocv_evaluation(params_dict: Dict[str, Any], pool: ProcessPoolExecutor) -> Dict[str, Any]:
    """Run full 5-fold Leave-One-Out Cross-Validation.
    
    Each fold holds out 1 sequence (seeds 1..10) as test, and uses other 4 sequences as train.
    """
    export_and_build(params_dict)

    fold_results = []

    for fold_idx, test_seq in enumerate(DEV_NAMES):
        train_seqs = [s for s in DEV_NAMES if s != test_seq]
        train_cases = [(s, seed) for s in train_seqs for seed in range(1, 9)]  # seeds 1..8
        test_cases = [(test_seq, seed) for seed in range(1, 11)]  # test sequence all 10 seeds

        tr_ate, tr_blind, tr_gap, tr_J = evaluate_cases(train_cases, pool)
        te_ate, te_blind, te_gap, te_J = evaluate_cases(test_cases, pool)

        fold_results.append({
            "fold": fold_idx + 1,
            "held_out_sequence": test_seq,
            "train_sequences": train_seqs,
            "train_ate_mean": tr_ate,
            "train_blind5_mean": tr_blind,
            "train_gap2s_mean": tr_gap,
            "train_J": tr_J,
            "test_ate_mean": te_ate,
            "test_blind5_mean": te_blind,
            "test_gap2s_mean": te_gap,
            "test_J": te_J,
        })

    # Overall 50-run evaluation
    all_cases = [(name, seed) for name in DEV_NAMES for seed in DEV_SEEDS]
    all_ate, all_blind, all_gap, all_J = evaluate_cases(all_cases, pool)

    return {
        "folds": fold_results,
        "overall_50": {
            "ate_rmse_aligned": all_ate,
            "blind5_rmse": all_blind,
            "gap_2s_rmse": all_gap,
            "J": all_J,
        }
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Tune EXP-E50 parameters with Optuna and LOOCV")
    parser.add_argument("--n-trials", type=int, default=30, help="Number of Optuna trials")
    parser.add_argument("--n-workers", type=int, default=8, help="Parallel evaluation workers")
    args = parser.parse_args()

    base_params = load_base_params()

    # Pre-generate cache if missing
    harness = DegradedHarness(binary_path=str(REPO_ROOT / "main"), verbose=False)
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    for s in REAL_SEQUENCES:
        for seed in DEV_SEEDS:
            target = CACHE_DIR / f"{s['name']}_s{seed}"
            harness.prepare_degraded_dataset(s, seed, target)

    print(f"Starting parameter search with {args.n_trials} trials...")

    pool = ProcessPoolExecutor(max_workers=args.n_workers)

    try:
        # Let's run a baseline verification first
        export_and_build(base_params)
        all_dev_cases = [(name, seed) for name in DEV_NAMES for seed in DEV_SEEDS]
        base_ate, base_blind, base_gap, base_J = evaluate_cases(all_dev_cases, pool)
        print(f"Baseline (50 runs): ATE={base_ate:.4f}m, Blind={base_blind:.4f}m, Gap={base_gap:.4f}m, J={base_J:.5f}")

        # Optimization objective on dev seeds 1..8 across all 5 sequences (40 runs per trial)
        tune_cases = [(name, seed) for name in DEV_NAMES for seed in range(1, 9)]

        study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=42))

        # Add baseline parameters as trial 0
        study.enqueue_trial({k: base_params[k] for k in PARAM_BOUNDS})

        def objective(trial: optuna.Trial) -> float:
            candidate_params = copy.deepcopy(base_params)
            for k, (low, high) in PARAM_BOUNDS.items():
                candidate_params[k] = trial.suggest_float(k, low, high)

            export_and_build(candidate_params)
            ate, blind, gap, J = evaluate_cases(tune_cases, pool)
            trial.set_user_attr("ate", ate)
            trial.set_user_attr("blind", blind)
            trial.set_user_attr("gap", gap)
            print(f"Trial #{trial.number:2d}: J={J:.5f} | ATE={ate:.4f}m | Blind={blind:.4f}m | Gap={gap:.4f}m")
            return J

        t_start = time.time()
        study.optimize(objective, n_trials=args.n_trials)
        t_opt = time.time() - t_start

        print(f"\n=== Optimization Complete in {t_opt:.1f}s ===")
        print(f"Best Trial #{study.best_trial.number}: J={study.best_value:.5f}")
        for k, v in study.best_params.items():
            print(f"  {k}: {v:.6f} (baseline was {base_params[k]})")

        # Save best parameters to a JSON file
        best_all_params = copy.deepcopy(base_params)
        best_all_params.update(study.best_params)
        out_best_json = REPO_ROOT / "reports" / "best_e50_params.json"
        with open(out_best_json, "w", encoding="utf-8") as f:
            json.dump(best_all_params, f, indent=2)
        print(f"Saved best params to {out_best_json}")

        # Run full LOOCV on the best parameters
        print("\nRunning full 5-fold LOOCV evaluation on candidate parameters...")
        cand_loocv = run_loocv_evaluation(best_all_params, pool)

        print("\nRunning full 5-fold LOOCV evaluation on baseline parameters...")
        base_loocv = run_loocv_evaluation(base_params, pool)

        loocv_comparison = {
            "candidate_params": study.best_params,
            "baseline_params": {k: base_params[k] for k in PARAM_BOUNDS},
            "candidate_loocv": cand_loocv,
            "baseline_loocv": base_loocv,
        }

        out_loocv_json = REPO_ROOT / "reports" / "e50_loocv_comparison.json"
        with open(out_loocv_json, "w", encoding="utf-8") as f:
            json.dump(loocv_comparison, f, indent=2)
        print(f"Saved LOOCV comparison to {out_loocv_json}")

    finally:
        pool.shutdown(wait=False)


if __name__ == "__main__":
    main()
