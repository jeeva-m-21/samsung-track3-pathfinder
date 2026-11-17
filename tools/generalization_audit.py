#!/usr/bin/env python3
"""
generalization_audit.py - Generalization Audit of submission-v2 (a3eb654).
Evaluates local datasets (held-out and dev benchmarks) under:
1. Clean Evaluation: evo_ape tum -a, evo_rpe tum -a --delta 1 --delta_unit f, start-anchored ATE, span
2. Degraded Evaluation (configs/degrade_default.json, Seeds 1..5): Waypoint ATE, Blind 5s, Gap 2s, Rot Error
3. Timing-Shifted Blackout Check on Dev Seed 1: Gap at t = 12s, t = 25s (baseline), t = 40s
"""

import os
import sys
import json
import tempfile
import subprocess
import numpy as np
import pandas as pd
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.degrade import write_degraded_dataset
from tools.metrics import compute_trajectory_metrics
from tools.eval_degraded_harness import REAL_SEQUENCES

BINARY = str(REPO_ROOT / "main")
CONFIG_DEFAULT = REPO_ROOT / "configs/degrade_default.json"

ALL_DATASETS = [
    # Dev Tuning Benchmark (5 sequences)
    {"name": "MH_01_easy", "category": "Dev Tuning", "imu": "data/euroc/MH_01_easy/mav0/imu0/data.csv", "gt": "data/euroc/gt/MH_01_easy.txt"},
    {"name": "V1_01_easy", "category": "Dev Tuning", "imu": "data/euroc/V1_01_easy/mav0/imu0/data.csv", "gt": "data/euroc/gt/V1_01_easy.txt"},
    {"name": "V2_02_medium", "category": "Dev Tuning", "imu": "data/euroc/V2_02_medium/mav0/imu0/data.csv", "gt": "data/euroc/gt/V2_02_medium.txt"},
    {"name": "TUM-VI room1", "category": "Dev Tuning", "imu": "data/tumvi/room1/mav0/imu0/data.csv", "gt": "data/tumvi/gt_room1.txt"},
    {"name": "TUM-VI room2", "category": "Dev Tuning", "imu": "data/tumvi/room2/mav0/imu0/data.csv", "gt": "data/tumvi/gt_room2.txt"},
    # Held-Out / Diagnostic Sequences
    {"name": "UZH-FPV fwd7", "category": "Held-Out", "imu": "data/uzh_fpv/imu_indoor_forward_7.csv", "gt": "data/uzh_fpv/gt_indoor_forward_7.txt"},
    {"name": "circle", "category": "Held-Out (Synth)", "imu": "data/synth/circle.csv", "gt": "data/synth/circle_gt.txt"},
    {"name": "figure_eight", "category": "Held-Out (Synth)", "imu": "data/synth/figure_eight.csv", "gt": "data/synth/figure_eight_gt.txt"},
]


def run_evo_metric(cmd):
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        # Parse rmse from output
        for line in res.stdout.splitlines():
            line_str = line.strip()
            if line_str.startswith("rmse"):
                parts = line_str.split()
                return float(parts[1])
        return np.nan
    except Exception as e:
        return np.nan


def audit_clean():
    print("\n" + "=" * 90)
    print("SECTION 1: CLEAN GENERALIZATION EVALUATION (submission-v2, commit a3eb654)")
    print("=" * 90)
    print(f"| {'Sequence':<15} | {'Category':<16} | {'evo APE (m)':<12} | {'evo RPE (mm/f)':<15} | {'ATE Start (m)':<13} | {'Span (Est/Ref)':<16} |")
    print("|" + "-"*17 + "|" + "-"*18 + "|" + "-"*14 + "|" + "-"*17 + "|" + "-"*15 + "|" + "-"*18 + "|")

    clean_results = {}
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        for ds in ALL_DATASETS:
            name = ds["name"]
            imu_p = str(REPO_ROOT / ds["imu"])
            gt_p = str(REPO_ROOT / ds["gt"])
            if not os.path.exists(gt_p) or not os.path.exists(imu_p):
                continue
            
            est_p = str(tmp / f"clean_{name}.txt")
            cmd = [BINARY, imu_p, est_p]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # evo_ape tum -a
            ape_rmse = run_evo_metric(["evo_ape", "tum", gt_p, est_p, "-a"])
            # evo_rpe tum -a -d 1 -u f (convert m to mm)
            rpe_rmse_m = run_evo_metric(["evo_rpe", "tum", gt_p, est_p, "-a", "-d", "1", "-u", "f"])
            rpe_rmse_mm = rpe_rmse_m * 1000.0 if not np.isnan(rpe_rmse_m) else np.nan
            
            # Native trajectory metrics
            m = compute_trajectory_metrics(gt_p, est_p)
            start_ate = m.get("start_anchored_rmse", np.nan)
            est_span = m.get("est_span", np.nan)
            ref_span = m.get("ref_span", np.nan)
            span_str = f"{est_span:.1f}m / {ref_span:.1f}m"
            
            clean_results[name] = {
                "category": ds["category"],
                "ape_rmse": ape_rmse,
                "rpe_rmse_mm": rpe_rmse_mm,
                "start_ate": start_ate,
                "est_span": est_span,
                "ref_span": ref_span
            }
            print(f"| {name:<15} | {ds['category']:<16} | {ape_rmse:>12.3f} | {rpe_rmse_mm:>15.2f} | {start_ate:>13.2f} | {span_str:>16} |")

    return clean_results


def audit_degraded():
    print("\n" + "=" * 90)
    print("SECTION 2: DEGRADED EVALUATION ACROSS SEEDS 1..5 (configs/degrade_default.json)")
    print("=" * 90)
    print(f"| {'Sequence':<15} | {'Category':<16} | {'ATE RMSE (m)':<13} | {'Blind 5s (m)':<13} | {'Gap 2s (m)':<13} | {'Rot Err (deg)':<13} |")
    print("|" + "-"*17 + "|" + "-"*18 + "|" + "-"*15 + "|" + "-"*15 + "|" + "-"*15 + "|" + "-"*15 + "|")

    seeds = list(range(1, 6))
    degraded_results = {}

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        for ds in ALL_DATASETS:
            name = ds["name"]
            imu_p = REPO_ROOT / ds["imu"]
            gt_p = REPO_ROOT / ds["gt"]
            if not gt_p.exists() or not imu_p.exists():
                continue

            rates = []
            blind5s = []
            gap2s = []
            rot_errs = []

            for seed in seeds:
                deg_dir = tmp / f"deg_{name}_s{seed}"
                deg_imu = deg_dir / "imu0" / "data.csv"
                deg_gt = deg_dir / "groundtruth.txt"
                sidecar = deg_dir / "degrade_info.json"
                try:
                    write_degraded_dataset(imu_p, gt_p, deg_dir, CONFIG_DEFAULT, seed=seed)
                    est_p = tmp / f"est_{name}_s{seed}.txt"
                    cmd = [BINARY, str(deg_imu), str(est_p), "--append-blind"]
                    res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if res.returncode == 0:
                        m = compute_trajectory_metrics(deg_gt, est_p, sidecar_path=sidecar)
                        rates.append(m.get("ate_rmse_aligned", np.nan))
                        blind5s.append(m.get("blind5_rmse", np.nan))
                        gap2s.append(m.get("gap_2s_rmse", np.nan))
                        rot_errs.append(m.get("rot_err_deg", np.nan))
                except Exception as e:
                    pass
                finally:
                    import shutil
                    shutil.rmtree(deg_dir, ignore_errors=True)

            mean_ate = float(np.nanmean(rates)) if rates else np.nan
            mean_b5 = float(np.nanmean(blind5s)) if blind5s else np.nan
            mean_gap = float(np.nanmean(gap2s)) if gap2s else np.nan
            mean_rot = float(np.nanmean(rot_errs)) if rot_errs else np.nan

            degraded_results[name] = {
                "category": ds["category"],
                "ate_mean": mean_ate,
                "blind5_mean": mean_b5,
                "gap2s_mean": mean_gap,
                "rot_mean": mean_rot,
                "ate_seeds": rates,
                "blind5_seeds": blind5s,
                "gap2s_seeds": gap2s,
            }
            print(f"| {name:<15} | {ds['category']:<16} | {mean_ate:>13.3f} | {mean_b5:>13.3f} | {mean_gap:>13.3f} | {mean_rot:>13.2f} |")

    return degraded_results


def audit_timing_shift():
    print("\n" + "=" * 90)
    print("SECTION 3: TIMING-SHIFTED BLACKOUT CHECK (Dev Seed 1, Gap at t=12s, t=25s, t=40s)")
    print("=" * 90)
    print(f"| {'Sequence':<15} | {'Gap @ t=12s (m)':<16} | {'Gap @ t=25s [Base]':<18} | {'Gap @ t=40s (m)':<16} | {'Delta 12 vs 25':<15} | {'Delta 40 vs 25':<15} |")
    print("|" + "-"*17 + "|" + "-"*18 + "|" + "-"*20 + "|" + "-"*18 + "|" + "-"*17 + "|" + "-"*17 + "|")

    # Construct configs for shift
    with open(CONFIG_DEFAULT) as f:
        cfg_base = json.load(f)

    # Shift configs:
    # Baseline: t = 25s gap of 2.0s
    # Shift A: t = 12s gap of 2.0s
    cfg_12 = json.loads(json.dumps(cfg_base))
    for d in cfg_12["dropouts"]:
        if d.get("duration_s") == 2.0:
            d["start_s"] = 12.0

    # Shift B: t = 40s gap of 2.0s
    cfg_40 = json.loads(json.dumps(cfg_base))
    for d in cfg_40["dropouts"]:
        if d.get("duration_s") == 2.0:
            d["start_s"] = 40.0

    shift_results = {}
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        cfg_12_p = tmp / "degrade_t12.json"
        cfg_25_p = CONFIG_DEFAULT
        cfg_40_p = tmp / "degrade_t40.json"
        with open(cfg_12_p, "w") as f: json.dump(cfg_12, f)
        with open(cfg_40_p, "w") as f: json.dump(cfg_40, f)

        for ds in ALL_DATASETS:
            name = ds["name"]
            imu_p = REPO_ROOT / ds["imu"]
            gt_p = REPO_ROOT / ds["gt"]
            if not gt_p.exists() or not imu_p.exists():
                continue

            gap_metrics = {}
            for label, c_path in [("t12", cfg_12_p), ("t25", cfg_25_p), ("t40", cfg_40_p)]:
                deg_dir = tmp / f"shift_{name}_{label}"
                deg_imu = deg_dir / "imu0" / "data.csv"
                deg_gt = deg_dir / "groundtruth.txt"
                sidecar = deg_dir / "degrade_info.json"
                try:
                    write_degraded_dataset(imu_p, gt_p, deg_dir, c_path, seed=1)
                    est_p = tmp / f"shift_est_{name}_{label}.txt"
                    cmd = [BINARY, str(deg_imu), str(est_p), "--append-blind"]
                    res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if res.returncode == 0:
                        m = compute_trajectory_metrics(deg_gt, est_p, sidecar_path=sidecar)
                        gap_metrics[label] = m.get("gap_2s_rmse", np.nan)
                    else:
                        gap_metrics[label] = np.nan
                except Exception:
                    gap_metrics[label] = np.nan
                finally:
                    import shutil
                    shutil.rmtree(deg_dir, ignore_errors=True)

            g12 = gap_metrics.get("t12", np.nan)
            g25 = gap_metrics.get("t25", np.nan)
            g40 = gap_metrics.get("t40", np.nan)

            d12 = f"{(g12 - g25) / g25 * 100.0:+.1f}%" if not np.isnan(g12) and not np.isnan(g25) and g25 > 0 else "N/A"
            d40 = f"{(g40 - g25) / g25 * 100.0:+.1f}%" if not np.isnan(g40) and not np.isnan(g25) and g25 > 0 else "N/A"

            shift_results[name] = {"gap_t12": g12, "gap_t25": g25, "gap_t40": g40, "d12": d12, "d40": d40}
            print(f"| {name:<15} | {g12:>16.3f} | {g25:>18.3f} | {g40:>16.3f} | {d12:>15} | {d40:>15} |")

    return shift_results


def main():
    clean_res = audit_clean()
    deg_res = audit_degraded()
    shift_res = audit_timing_shift()

    # Save full report
    report = {
        "commit": "a3eb654",
        "clean": clean_res,
        "degraded_seeds_1_5": deg_res,
        "timing_shift_seed_1": shift_res,
    }
    out_json = REPO_ROOT / "reports" / "generalization_audit.json"
    with open(out_json, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nFull generalization audit results saved to {out_json}")


if __name__ == "__main__":
    main()
