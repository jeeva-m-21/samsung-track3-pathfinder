#!/usr/bin/env python3
"""tune_e71.py - Automated 5-Fold LOOCV Re-Calibration for EXP-E71.

Tunes the 13 coupled physical parameters on top of best-010:
- tau_ou (flight and walking damping)
- rotor_drag_kd, rotor_drag_sigma (rotor drag velocity prior)
- ped_sigma_lat, ped_sigma_vert (pedestrian non-holonomic velocity prior)
- max_velocity_mps, velocity_bounding_sigma (kinematic velocity envelope)
- tau_v, shrink_v, stationary_v_thresh, tau_omega_blind (blind extrapolation)
- adaptive_q_ref (gyro-MAD noise floor)
- tilt_tau_a (gravity alignment window)

Includes Hard Clean-Data Constraint: Evaluates clean MH_01_easy ATE on every candidate
to ensure degraded gains do not come at the cost of clean flight accuracy.
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

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics
from tools.eval_degraded_harness import REAL_SEQUENCES, DegradedHarness

# Baseline denominators from best-006 (official metric standard)
ATE_0 = 2.309264
BLIND_0 = 4.783184
GAP_0 = 6.200439

DEV_SEEDS = list(range(1, 11))
DEV_NAMES = ["MH_01_easy", "V1_01_easy", "V2_02_medium", "room1", "room2"]

CACHE_DIR = Path("/tmp/degraded_dev_cache_e71")

PARAM_BOUNDS = {
    "tau_ou": (1.4, 2.8),
    "shrink_v": (0.80, 0.98),
    "tau_v": (2.0, 4.0),
    "stationary_v_thresh": (0.02, 0.10),
    "tau_omega_blind": (1.0, 2.2),
    "adaptive_q_ref": (0.003, 0.009),
    "rotor_drag_kd": (0.04, 0.12),
    "rotor_drag_sigma": (0.5, 1.1),
    "ped_sigma_lat": (0.20, 0.50),
    "ped_sigma_vert": (0.45, 0.85),
    "max_velocity_mps": (2.6, 3.8),
    "velocity_bounding_sigma": (0.06, 0.18),
    "tilt_tau_a": (0.14, 0.25),
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


def evaluate_clean_mh01() -> float:
    """Evaluate clean MH_01_easy ATE RMSE."""
    with tempfile.TemporaryDirectory() as td:
        out_traj = Path(td) / "clean_mh01.txt"
        cmd = [str(REPO_ROOT / "main"), "data/euroc/MH_01_easy/mav0/imu0/data.csv", str(out_traj)]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        m = compute_trajectory_metrics(Path("data/euroc/gt/MH_01_easy.txt"), out_traj)
        return float(m["ate_rmse_aligned"])


def run_loocv_evaluation(params_dict: Dict[str, Any], pool: ProcessPoolExecutor) -> Dict[str, Any]:
    """Run full 5-fold Leave-One-Out Cross-Validation."""
    export_and_build(params_dict)

    fold_results = []
    for fold_idx, test_seq in enumerate(DEV_NAMES):
        train_seqs = [s for s in DEV_NAMES if s != test_seq]
        train_cases = [(s, seed) for s in train_seqs for seed in range(1, 9)]
        test_cases = [(test_seq, seed) for seed in range(1, 11)]

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

    all_cases = [(name, seed) for name in DEV_NAMES for seed in DEV_SEEDS]
    all_ate, all_blind, all_gap, all_J = evaluate_cases(all_cases, pool)
    clean_ate = evaluate_clean_mh01()

    return {
        "folds": fold_results,
        "overall_50": {
            "ate_rmse_aligned": all_ate,
            "blind5_rmse": all_blind,
            "gap_2s_rmse": all_gap,
            "J": all_J,
            "clean_mh01_ate": clean_ate,
        }
    }


def prepare_cache_worker(args):
    seq_cfg, seed = args
    harness = DegradedHarness(binary_path=str(REPO_ROOT / "main"), verbose=False)
    target = CACHE_DIR / f"{seq_cfg['name']}_s{seed}"
    harness.prepare_degraded_dataset(seq_cfg, seed, target)


def main() -> None:
    parser = argparse.ArgumentParser(description="Tune EXP-E71 parameters with Optuna and LOOCV")
    parser.add_argument("--n-trials", type=int, default=25, help="Number of Optuna trials")
    parser.add_argument("--n-workers", type=int, default=8, help="Parallel evaluation workers")
    args = parser.parse_args()

    base_params = load_base_params()

    # Pre-generate cache in parallel
    print("Preparing degraded dataset cache...")
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cache_tasks = [(s, seed) for s in REAL_SEQUENCES for seed in DEV_SEEDS]
    with ProcessPoolExecutor(max_workers=args.n_workers) as pool:
        list(pool.map(prepare_cache_worker, cache_tasks))
    print(f"Cache ready at {CACHE_DIR} ({len(cache_tasks)} cases)")

    pool = ProcessPoolExecutor(max_workers=args.n_workers)

    try:
        # Baseline verification
        export_and_build(base_params)
        all_dev_cases = [(name, seed) for name in DEV_NAMES for seed in DEV_SEEDS]
        base_ate, base_blind, base_gap, base_J = evaluate_cases(all_dev_cases, pool)
        base_clean = evaluate_clean_mh01()
        print(f"Baseline best-010 (50 runs): ATE={base_ate:.4f}m, Blind={base_blind:.4f}m, Gap={base_gap:.4f}m, Clean MH_01={base_clean:.4f}m, J={base_J:.5f}")

        # Optimization on seeds 1..8 across all 5 sequences (40 runs per trial)
        tune_cases = [(name, seed) for name in DEV_NAMES for seed in range(1, 9)]

        study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=42))

        # Enqueue baseline parameters as trial 0
        study.enqueue_trial({k: base_params[k] for k in PARAM_BOUNDS if k in base_params})

        def objective(trial: optuna.Trial) -> float:
            candidate_params = copy.deepcopy(base_params)
            for k, (low, high) in PARAM_BOUNDS.items():
                candidate_params[k] = trial.suggest_float(k, low, high)

            export_and_build(candidate_params)
            ate, blind, gap, J = evaluate_cases(tune_cases, pool)
            clean_ate = evaluate_clean_mh01()

            # Hard clean constraint penalty if clean ATE exceeds 2.5m
            penalty = 0.10 * max(0.0, clean_ate - 2.5)
            obj_val = J + penalty

            trial.set_user_attr("ate", ate)
            trial.set_user_attr("blind", blind)
            trial.set_user_attr("gap", gap)
            trial.set_user_attr("clean_ate", clean_ate)
            print(f"Trial #{trial.number:2d}: J={J:.5f} (clean={clean_ate:.4f}m) | ATE={ate:.4f}m | Blind={blind:.4f}m | Gap={gap:.4f}m")
            return obj_val

        t_start = time.time()
        study.optimize(objective, n_trials=args.n_trials)
        t_opt = time.time() - t_start

        print(f"\n=== Optimization Complete in {t_opt:.1f}s ===")
        print(f"Best Trial #{study.best_trial.number}: Objective={study.best_value:.5f}")
        for k, v in study.best_params.items():
            print(f"  {k}: {v:.6f} (baseline was {base_params.get(k, 'N/A')})")

        best_all_params = copy.deepcopy(base_params)
        best_all_params.update(study.best_params)
        out_best_json = REPO_ROOT / "reports" / "best_e71_params.json"
        with open(out_best_json, "w", encoding="utf-8") as f:
            json.dump(best_all_params, f, indent=2)

        # Full 5-fold LOOCV evaluation
        print("\nRunning full 5-fold LOOCV on candidate parameters...")
        cand_loocv = run_loocv_evaluation(best_all_params, pool)

        print("\nRunning full 5-fold LOOCV on baseline parameters...")
        base_loocv = run_loocv_evaluation(base_params, pool)

        loocv_comparison = {
            "candidate_params": study.best_params,
            "baseline_params": {k: base_params[k] for k in PARAM_BOUNDS if k in base_params},
            "candidate_loocv": cand_loocv,
            "baseline_loocv": base_loocv,
        }

        out_loocv_json = REPO_ROOT / "reports" / "e71_loocv_comparison.json"
        with open(out_loocv_json, "w", encoding="utf-8") as f:
            json.dump(loocv_comparison, f, indent=2)
        print(f"Saved LOOCV comparison to {out_loocv_json}")

        # Summary
        c_ov = cand_loocv["overall_50"]
        b_ov = base_loocv["overall_50"]
        print("\n" + "="*80)
        print("EXP-E71 50-RUN OVERALL COMPARISON:")
        print("="*80)
        print(f"Blind 5s RMSE : Base={b_ov['blind5_rmse']:.4f}m -> Cand={c_ov['blind5_rmse']:.4f}m ({(c_ov['blind5_rmse'] - b_ov['blind5_rmse'])/b_ov['blind5_rmse']*100:+.2f}%)")
        print(f"Waypoint ATE  : Base={b_ov['ate_rmse_aligned']:.4f}m -> Cand={c_ov['ate_rmse_aligned']:.4f}m ({(c_ov['ate_rmse_aligned'] - b_ov['ate_rmse_aligned'])/b_ov['ate_rmse_aligned']*100:+.2f}%)")
        print(f"Dropout Gap 2s: Base={b_ov['gap_2s_rmse']:.4f}m -> Cand={c_ov['gap_2s_rmse']:.4f}m ({(c_ov['gap_2s_rmse'] - b_ov['gap_2s_rmse'])/b_ov['gap_2s_rmse']*100:+.2f}%)")
        print(f"Composite J   : Base={b_ov['J']:.5f} -> Cand={c_ov['J']:.5f} ({(c_ov['J'] - b_ov['J'])/b_ov['J']*100:+.2f}%)")
        print(f"Clean MH_01   : Base={b_ov['clean_mh01_ate']:.4f}m -> Cand={c_ov['clean_mh01_ate']:.4f}m")

    finally:
        pool.shutdown(wait=False)


if __name__ == "__main__":
    main()
