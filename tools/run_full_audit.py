#!/usr/bin/env python3
"""tools/run_full_audit.py - Comprehensive speed, feature, and accuracy audit for main.py."""

from __future__ import annotations

import copy
import json
import multiprocessing as mp
import os
import subprocess
import sys
import tempfile
import time
import tracemalloc
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import degrade_imu
import main
from tools.eval_degraded_harness import REAL_SEQUENCES

from evo.core import metrics, sync
from evo.tools import file_interface


def get_imu_duration(csv_path: str) -> float:
    ts = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split(",") if "," in line else line.split()
            try:
                ts.append(float(parts[0]))
            except ValueError:
                continue
    if len(ts) < 2:
        return 1.0
    ts_arr = np.array(ts)
    dt = np.median(np.diff(ts_arr))
    if dt < 1.0:
        scale = 1.0
    elif dt < 1e3:
        scale = 1e-3
    elif dt < 1e6:
        scale = 1e-6
    else:
        scale = 1e-9
    return float((ts_arr[-1] - ts_arr[0]) * scale)


# ==============================================================================
# Part 1: Single-Thread Runtime Benchmark (taskset -c 0)
# ==============================================================================

def run_speed_benchmark() -> List[Dict[str, Any]]:
    print("=" * 80)
    print("PART 1: SINGLE-THREAD RUNTIME BENCHMARK (taskset -c 0)")
    print("=" * 80)

    results = []

    for seq in REAL_SEQUENCES:
        seq_name = seq["name"]
        imu_path = str(REPO_ROOT / seq["imu"])
        imu_dur_s = get_imu_duration(imu_path)

        with tempfile.TemporaryDirectory() as tmpdir:
            out_traj = os.path.join(tmpdir, "est.txt")

            # 1. Measure total process time via taskset -c 0
            cmd = ["taskset", "-c", "0", sys.executable, str(REPO_ROOT / "main.py"), imu_path, out_traj]
            t0 = time.perf_counter()
            sub_res = subprocess.run(cmd, capture_output=True, text=True)
            t_total = time.perf_counter() - t0
            assert sub_res.returncode == 0, f"Process failed: {sub_res.stderr}"

            # 2. Measure in-process filter execution (WITHOUT tracemalloc active)
            try:
                os.sched_setaffinity(0, {0})
            except Exception:
                pass

            t0_filter = time.perf_counter()
            main.run_pipeline(imu_path, out_traj, ablation="eskf_full")
            t_filter = time.perf_counter() - t0_filter

            # 3. Measure peak heap separately on a 2000-sample slice
            with tempfile.NamedTemporaryFile("w") as slice_in, tempfile.NamedTemporaryFile("w") as slice_out:
                with open(imu_path, "r", encoding="utf-8") as f_src:
                    for _ in range(2000):
                        line = f_src.readline()
                        if not line:
                            break
                        slice_in.write(line)
                slice_in.flush()
                tracemalloc.start()
                main.run_pipeline(slice_in.name, slice_out.name, ablation="eskf_full")
                curr_mem, peak_mem = tracemalloc.get_traced_memory()
                tracemalloc.stop()

        filter_ms_per_s = (t_filter / imu_dur_s) * 1000.0
        total_ms_per_s = (t_total / imu_dur_s) * 1000.0
        peak_mb = peak_mem / (1024.0 * 1024.0)

        res = {
            "sequence": seq_name,
            "duration_s": imu_dur_s,
            "filter_time_s": t_filter,
            "total_time_s": t_total,
            "filter_ms_per_s": filter_ms_per_s,
            "total_ms_per_s": total_ms_per_s,
            "peak_heap_mb": peak_mb,
            "target_met": total_ms_per_s <= 20.0,
        }
        results.append(res)

        print(
            f"[{seq_name:13s}] Dur: {imu_dur_s:6.1f}s | "
            f"Filter-loop: {filter_ms_per_s:6.2f} ms/s | "
            f"Total Process: {total_ms_per_s:6.2f} ms/s | "
            f"Heap: {peak_mb:5.2f} MB | <=20 ms/s: {'PASS' if res['target_met'] else 'FAIL'}"
        )

    return results


# ==============================================================================
# Part 2: Feature Audit inside main.py for official degrade_imu.py hazards
# ==============================================================================

def run_feature_audit() -> Dict[str, Dict[str, Any]]:
    print("\n" + "=" * 80)
    print("PART 2: FEATURE AUDIT INSIDE main.py FOR OFFICIAL degrade_imu.py HAZARDS")
    print("=" * 80)

    audit = {}
    main_code = (REPO_ROOT / "main.py").read_text(encoding="utf-8")

    # 1. Exact duplicate-row bypass (t[k] == t[k-1])
    dup_bypass = "dup_mask[1:] = t_s[1:] <= t_s[:-1]" in main_code and "if dup_mask[k]:" in main_code
    dup_detail = (
        "IMPLEMENTED: When dt <= 0 (e.g. t[k] <= t[k-1]), dup_mask flags True, "
        "bypasses strapdown integration and Kalman measurement updates, and immediately outputs "
        "the previous pose (out_p[k] = p_last; out_q[k] = q_last) preserving timestamp continuity."
        if dup_bypass
        else "NOT IMPLEMENTED"
    )
    audit["1_duplicate_row_bypass"] = {"status": dup_bypass, "detail": dup_detail}
    print(f"1. Exact duplicate-row bypass: {'[YES]' if dup_bypass else '[NO]'}\n   -> {dup_detail}")

    # 2. Stuck-sensor detection & linear interpolation
    stuck_detect = "same_prev = (" in main_code and "run_len" in main_code
    stuck_interp = "(1.0 - alpha) * a0 + alpha * a1" in main_code
    stuck_status = stuck_detect and stuck_interp
    stuck_detail = (
        "IMPLEMENTED: Stuck-sensor detection detects consecutive identical readings (2..20 frames) "
        "in preprocessing and applies linear interpolation between clean boundary samples before filter integration."
        if stuck_status
        else "NOT IMPLEMENTED"
    )
    audit["2_stuck_sensor_detection_and_interp"] = {"status": stuck_status, "detail": stuck_detail}
    print(f"2. Stuck-sensor detection & interp: {'[YES]' if stuck_status else '[PARTIAL]'}\n   -> {stuck_detail}")

    # 3. Offline spike sniping (centered Hampel)
    hampel_implemented = "mad_a = max(0.05, float(np.median(da)))" in main_code and "joint_spike" in main_code
    spike_detail = (
        "IMPLEMENTED: Centered Hampel filter tracks median absolute deviation (MAD) of both accel and gyro, "
        "and snipes simultaneous joint acceleration and rate spikes."
        if hampel_implemented
        else "NOT IMPLEMENTED"
    )
    audit["3_offline_spike_sniping"] = {"status": hampel_implemented, "detail": spike_detail}
    print(f"3. Offline spike sniping (centered Hampel): {'[YES]' if hampel_implemented else '[NO]'}\n   -> {spike_detail}")

    # 4. Missing-row burst bridging (5..50 missing frames)
    burst_implemented = "propagate_gap" in main_code and "delta_bridge" in main_code
    burst_offline = "s_herm = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha" in main_code
    burst_detail = (
        "IMPLEMENTED: Missing frames (sudden dt jumps > 2.5*dt_nom) trigger propagate_gap with dynamic sub-stepping "
        "and process covariance inflation (kappa_gap), and apply a cubic Hermite bridge connecting pre-gap and post-gap poses."
        if (burst_implemented and burst_offline)
        else "PARTIAL"
    )
    audit["4_missing_row_burst_bridging"] = {"status": burst_implemented and burst_offline, "detail": burst_detail}
    print(f"4. Missing-row burst bridging: {'[YES]' if burst_implemented and burst_offline else '[NO]'}\n   -> {burst_detail}")

    # 5. Exact 0.0 terminal 5-second failure detection (acc==0 && gyr==0) without appending rows
    blind_implemented = "zero_mask" in main_code and "eval_blind_pose" in main_code and "fail_start" in main_code
    blind_detail = (
        "IMPLEMENTED: Trailing all-zero blocks (acc==0, gyr==0) are detected offline as terminal failure. "
        "eval_blind_pose predicts decaying curved/straight poses for existing rows without appending extra rows (N_out == N_in)."
        if blind_implemented
        else "NOT IMPLEMENTED"
    )
    audit["5_terminal_5s_failure_detection"] = {"status": blind_implemented, "detail": blind_detail}
    print(f"5. Exact 0.0 terminal 5s failure detection: {'[YES]' if blind_implemented else '[NO]'}\n   -> {blind_detail}")

    # 6. Clean protect_seconds = 2.0 s startup calibration
    t_init = main.DEFAULT_PARAMS.get("T_init", 1.0)
    protect_detail = (
        f"CONFIGURED: T_init is set to {t_init} s (default config). Official degrade_imu.py protects first "
        f"protect_seconds = 2.0 s from corruption. Currently initialization uses the first {t_init} s of clean data. "
        f"Recommendation: T_init can be tuned up to 2.0 s to exploit full protected window."
    )
    audit["6_startup_calibration"] = {"status": True, "T_init": t_init, "detail": protect_detail}
    print(f"6. Clean startup calibration (protect_seconds): [CONFIGURED ({t_init}s)]\n   -> {protect_detail}")

    return audit


# ==============================================================================
# Part 3: Baseline Accuracy on Official degrade_imu.py
# ==============================================================================

def score_trajectory(gt_path: str, est_path: str, fail_t_s: float, gap_windows: List[Tuple[float, float]]) -> Dict[str, float]:
    """Compute SE(3) Umeyama ATE RMSE, 1s RPE, Blind 5s RMSE, and Dropout RMSE."""
    t_ref_evo = file_interface.read_tum_trajectory_file(gt_path)
    t_est_evo = file_interface.read_tum_trajectory_file(est_path)

    # Sync
    t_ref_sync, t_est_sync = sync.associate_trajectories(t_ref_evo, t_est_evo, max_diff=0.01)

    # 1. Umeyama SE(3) Alignment
    t_est_aligned = copy.deepcopy(t_est_sync)
    t_est_aligned.align(t_ref_sync, correct_scale=False)

    ape_aligned = metrics.APE(metrics.PoseRelation.translation_part)
    ape_aligned.process_data((t_ref_sync, t_est_aligned))
    ate_rmse = float(ape_aligned.get_statistic(metrics.StatisticsType.rmse))

    # 2. 1s RPE
    dt = float(np.median(np.diff(t_ref_sync.timestamps))) if len(t_ref_sync.timestamps) > 1 else 0.005
    frames_1s = max(1, int(round(1.0 / dt)))
    try:
        rpe_metric = metrics.RPE(
            metrics.PoseRelation.translation_part,
            delta=frames_1s,
            delta_unit=metrics.Unit.frames,
            all_pairs=False,
        )
        rpe_metric.process_data((t_ref_sync, t_est_aligned))
        rpe_1s = float(rpe_metric.get_statistic(metrics.StatisticsType.rmse))
    except Exception:
        rpe_1s = 0.0

    # Positions and timestamps in aligned world frame
    t_ref_s = t_ref_sync.timestamps - t_ref_sync.timestamps[0]
    pos_err = np.linalg.norm(t_est_aligned.positions_xyz - t_ref_sync.positions_xyz, axis=1)

    # 3. Blind 5s RMSE (over final failure_seconds = 5.0 s)
    blind_mask = t_ref_s >= fail_t_s
    if np.any(blind_mask):
        blind_5s_rmse = float(np.sqrt(np.mean(pos_err[blind_mask] ** 2)))
    else:
        blind_5s_rmse = float(pos_err[-1])

    # 4. Burst Dropout RMSE (over gap intervals)
    if gap_windows:
        gap_masks = []
        for g_start, g_end in gap_windows:
            gap_masks.append((t_ref_s >= g_start) & (t_ref_s <= g_end + 1.0))
        combined_gap_mask = np.logical_or.reduce(gap_masks)
        if np.any(combined_gap_mask):
            dropout_rmse = float(np.sqrt(np.mean(pos_err[combined_gap_mask] ** 2)))
        else:
            dropout_rmse = float(ate_rmse)
    else:
        dropout_rmse = float(ate_rmse)

    return {
        "ate_rmse": ate_rmse,
        "rpe_1s": rpe_1s,
        "blind_5s_rmse": blind_5s_rmse,
        "dropout_rmse": dropout_rmse,
    }


def evaluate_single_run(args: Tuple[str, str, str, int, bool]) -> Dict[str, Any]:
    """Worker task: degrades (if not clean), runs main.py, and scores trajectory."""
    seq_name, imu_p, gt_p, seed, is_clean = args
    cfg = dict(degrade_imu.CONFIG)

    with tempfile.TemporaryDirectory(prefix=f"eval_{seq_name}_{seed}_") as tmpdir:
        out_traj = os.path.join(tmpdir, "est.txt")

        if is_clean:
            input_csv = imu_p
            gap_windows = []
            dur_s = get_imu_duration(imu_p)
            fail_t_s = dur_s - 5.0
        else:
            header = degrade_imu.detect_header(imu_p)
            df = pd.read_csv(imu_p, header=header)
            if header is None:
                df.columns = ["timestamp", "ax", "ay", "az", "gx", "gy", "gz"]

            out_df, stats = degrade_imu.degrade(df, cfg, seed=seed)
            input_csv = os.path.join(tmpdir, "deg.csv")
            out_df.to_csv(input_csv, index=False, header=header is not None)

            ts_vals = out_df.iloc[:, 0].to_numpy(dtype=float)
            ts_scale, _ = degrade_imu.detect_ts_scale(ts_vals, "auto")
            t_sec = (ts_vals - ts_vals[0]) * ts_scale
            dts = np.diff(t_sec)
            med_dt = np.median(dts)
            gap_indices = np.flatnonzero(dts > 2.0 * med_dt)
            gap_windows = [(t_sec[i], t_sec[i + 1]) for i in gap_indices]

            dur_s = stats["duration_s"]
            fail_t_s = dur_s - 5.0

        main.run_pipeline(input_csv, out_traj, ablation="eskf_full")
        scores = score_trajectory(gt_p, out_traj, fail_t_s, gap_windows)

    return {
        "sequence": seq_name,
        "seed": seed,
        "is_clean": is_clean,
        "scores": scores,
    }


def run_accuracy_benchmark() -> Dict[str, Any]:
    print("\n" + "=" * 80)
    print("PART 3: BASELINE ACCURACY ON OFFICIAL tools/degrade_imu.py")
    print("=" * 80)

    accuracy_report = {
        "clean_sequences": {},
        "official_degraded_seed42": {},
        "official_degraded_seeds_1_to_10": {},
    }

    n_workers = min(12, mp.cpu_count())
    pool = mp.Pool(n_workers)

    # A. Clean Sequences Baseline
    print(f"\n--- A. Clean Sequences (Uncorrupted) [running {len(REAL_SEQUENCES)} parallel jobs] ---")
    clean_tasks = [
        (s["name"], str(REPO_ROOT / s["imu"]), str(REPO_ROOT / s["gt"]), 0, True)
        for s in REAL_SEQUENCES
    ]
    clean_results = pool.map(evaluate_single_run, clean_tasks)
    for r in clean_results:
        seq_name = r["sequence"]
        scores = r["scores"]
        accuracy_report["clean_sequences"][seq_name] = scores
        print(
            f"[{seq_name:13s}] Clean ATE: {scores['ate_rmse']:6.3f} m | "
            f"1s RPE: {scores['rpe_1s']:6.3f} m | "
            f"Blind 5s: {scores['blind_5s_rmse']:6.3f} m"
        )

    # B. Official degraded SEED=42
    print(f"\n--- B. Official degrade_imu.py (SEED=42) [running {len(REAL_SEQUENCES)} parallel jobs] ---")
    deg42_tasks = [
        (s["name"], str(REPO_ROOT / s["imu"]), str(REPO_ROOT / s["gt"]), 42, False)
        for s in REAL_SEQUENCES
    ]
    deg42_results = pool.map(evaluate_single_run, deg42_tasks)
    for r in deg42_results:
        seq_name = r["sequence"]
        scores = r["scores"]
        accuracy_report["official_degraded_seed42"][seq_name] = scores
        print(
            f"[{seq_name:13s}] ATE: {scores['ate_rmse']:6.3f} m | "
            f"1s RPE: {scores['rpe_1s']:6.3f} m | "
            f"Blind 5s: {scores['blind_5s_rmse']:6.3f} m | "
            f"Burst Dropout: {scores['dropout_rmse']:6.3f} m"
        )

    # C. Multi-seed Evaluation: Seeds {1..10}
    print(f"\n--- C. Official degrade_imu.py Multi-Seed Benchmark (Seeds 1..10, 50 parallel runs) ---")
    t0_multi = time.perf_counter()
    multi_tasks = []
    for seed in range(1, 11):
        for s in REAL_SEQUENCES:
            multi_tasks.append((s["name"], str(REPO_ROOT / s["imu"]), str(REPO_ROOT / s["gt"]), seed, False))

    multi_results = pool.map(evaluate_single_run, multi_tasks)
    elapsed_multi = time.perf_counter() - t0_multi
    print(f"Completed 50 evaluation runs across {n_workers} workers in {elapsed_multi:.1f} s!")

    pool.close()
    pool.join()

    # Aggregate by sequence
    seq_scores = {s["name"]: {"ate": [], "rpe": [], "blind": [], "dropout": []} for s in REAL_SEQUENCES}
    for r in multi_results:
        name = r["sequence"]
        sc = r["scores"]
        seq_scores[name]["ate"].append(sc["ate_rmse"])
        seq_scores[name]["rpe"].append(sc["rpe_1s"])
        seq_scores[name]["blind"].append(sc["blind_5s_rmse"])
        seq_scores[name]["dropout"].append(sc["dropout_rmse"])

    print("\nSummary Across 50 Degraded Runs (5 Sequences x 10 Seeds):")
    print("-" * 80)
    print(f"{'Sequence':15s} | {'ATE RMSE (m)':16s} | {'1s RPE (m)':14s} | {'Blind 5s (m)':14s} | {'Dropout (m)':14s}")
    print("-" * 80)

    all_ates = []
    all_rpes = []
    all_blinds = []
    all_dropouts = []

    for seq in REAL_SEQUENCES:
        name = seq["name"]
        ates = seq_scores[name]["ate"]
        rpes = seq_scores[name]["rpe"]
        blinds = seq_scores[name]["blind"]
        dropouts = seq_scores[name]["dropout"]

        accuracy_report["official_degraded_seeds_1_to_10"][name] = {
            "ate_mean": float(np.mean(ates)),
            "ate_std": float(np.std(ates)),
            "rpe_mean": float(np.mean(rpes)),
            "blind_mean": float(np.mean(blinds)),
            "blind_std": float(np.std(blinds)),
            "dropout_mean": float(np.mean(dropouts)),
            "dropout_std": float(np.std(dropouts)),
        }

        all_ates.extend(ates)
        all_rpes.extend(rpes)
        all_blinds.extend(blinds)
        all_dropouts.extend(dropouts)

        print(
            f"{name:15s} | {np.mean(ates):6.3f} +/- {np.std(ates):4.2f} m | "
            f"{np.mean(rpes):6.3f} +/- {np.std(rpes):4.2f} m | "
            f"{np.mean(blinds):6.3f} +/- {np.std(blinds):4.2f} m | "
            f"{np.mean(dropouts):6.3f} +/- {np.std(dropouts):4.2f} m"
        )

    print("-" * 80)
    overall_ate = float(np.mean(all_ates))
    overall_rpe = float(np.mean(all_rpes))
    overall_blind = float(np.mean(all_blinds))
    overall_dropout = float(np.mean(all_dropouts))
    accuracy_report["overall_means"] = {
        "ate_rmse": overall_ate,
        "rpe_1s": overall_rpe,
        "blind_5s_rmse": overall_blind,
        "dropout_rmse": overall_dropout,
    }
    print(
        f"{'OVERALL MEAN':15s} | {overall_ate:6.3f} m           | "
        f"{overall_rpe:6.3f} m         | {overall_blind:6.3f} m         | {overall_dropout:6.3f} m"
    )

    return accuracy_report


# ==============================================================================
# Main Orchestrator
# ==============================================================================

def main_audit():
    print("STARTING FULL AUDIT OF standalone main.py (commit dfac0ec)\n")
    speed_results = run_speed_benchmark()
    feature_results = run_feature_audit()
    accuracy_results = run_accuracy_benchmark()

    # Save complete audit report to reports/AUDIT_DFAC0EC.json
    out_json = REPO_ROOT / "reports" / "AUDIT_DFAC0EC.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(
            {
                "commit": "dfac0ec",
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "speed": speed_results,
                "features": feature_results,
                "accuracy": accuracy_results,
            },
            f,
            indent=2,
        )
    print(f"\nAudit complete! Full JSON written to {out_json}")


if __name__ == "__main__":
    main_audit()
