#!/usr/bin/env python3
"""
tools/diagnose_spans.py - Comprehensive Diagnostic and Oracle Headroom Suite.

Diagnoses trajectory span anomalies and error mechanisms on:
- MH_01_easy (EuroC: 3.1m est vs 13.6m ref span)
- TUM-VI room1 (TUM-VI: 63.6m est vs 4.7m ref span)
- UZH-FPV fwd7 (UZH-FPV: 65.2m est vs 24.5m ref span)

Outputs:
1. Multi-panel diagnostic plots against ground truth:
   - Trajectory comparison & spatial spans
   - Velocity error vs ground truth velocity
   - Attitude errors (yaw, pitch, roll)
   - Estimated sensor biases (b_a, b_g)
   - Active regime flags (ZUPT, tilt updates, velocity bounding, kappa, quality)
2. Oracle experiments measuring headroom ceilings:
   - Oracle 1: GT Yaw injection
   - Oracle 2: GT Biases injection
   - Oracle 3: GT Initial Velocity v0 injection
   - Oracle 4: Full Oracle (GT Yaw + Bias + v0)
   - Oracle 5: GT Attitude (Full 3D orientation)
   - Ablations: Disabling tau_ou, ZUPT, Tilt, Vel-bounding
"""

import os
import sys
import subprocess
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics
from tools.benchmark_suite import compute_null_baseline

DIAG_DATASETS = [
    ('MH_01_easy', 'data/euroc/MH_01_easy/mav0/imu0/data.csv', 'data/euroc/gt/MH_01_easy.txt'),
    ('TUM-VI room1', 'data/tumvi/room1/mav0/imu0/data.csv', 'data/tumvi/gt_room1.txt'),
    ('UZH-FPV fwd7', 'data/uzh_fpv/imu_indoor_forward_7.csv', 'data/uzh_fpv/gt_indoor_forward_7.txt')
]

def quat_to_rot_matrix(q):
    """q is [w, x, y, z]"""
    w, x, y, z = q
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z + w*y)],
        [2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y), 2.0*(y*z + w*x), 1.0 - 2.0*(x*x + y*y)]
    ])

def rot_matrix_to_euler(R):
    """Z-Y-X Euler angles: yaw (psi), pitch (theta), roll (phi)"""
    r20 = np.clip(R[2, 0], -1.0, 1.0)
    theta = -np.arcsin(r20)
    phi = np.arctan2(R[2, 1], R[2, 2])
    psi = np.arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi

def run_diagnostics_and_plots(name, imu_path, gt_path, binary="./main", out_dir="reports"):
    os.makedirs(out_dir, exist_ok=True)
    clean_name = name.replace(" ", "_").replace("-", "_")
    traj_out = f"/tmp/diag_{clean_name}_traj.txt"
    tel_out = f"/tmp/diag_{clean_name}_tel.csv"

    # Step 1: Run baseline with telemetry
    cmd = [binary, imu_path, traj_out, "--telemetry", tel_out]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Step 2: Load telemetry and ground truth
    tel = pd.read_csv(tel_out)
    gt = pd.read_csv(gt_path, sep=' ', comment='#', header=None)
    gt_t = gt.iloc[:, 0].values
    gt_p = gt.iloc[:, 1:4].values
    # GT quat in TUM is [x, y, z, w]
    gt_q_xyzw = gt.iloc[:, 4:8].values
    gt_q = np.column_stack([gt_q_xyzw[:, 3], gt_q_xyzw[:, 0], gt_q_xyzw[:, 1], gt_q_xyzw[:, 2]]) # [w, x, y, z]

    # Compute GT velocity by finite differences
    dt_gt = np.diff(gt_t)
    dt_gt_safe = np.where(dt_gt <= 0, 1e-6, dt_gt)
    gt_v = np.diff(gt_p, axis=0) / dt_gt_safe[:, None]
    gt_v = np.vstack([gt_v[0:1], gt_v]) # pad first

    # Telemetry timestamps in seconds
    tel_t_sec = tel['t_ns'].values * 1e-9
    t_start = max(tel_t_sec[0], gt_t[0])
    t_end = min(tel_t_sec[-1], gt_t[-1])

    # Filter telemetry to overlapping window
    mask_tel = (tel_t_sec >= t_start) & (tel_t_sec <= t_end)
    tel = tel[mask_tel].reset_index(drop=True)
    tel_t_sec = tel['t_ns'].values * 1e-9
    t_rel = tel_t_sec - tel_t_sec[0]

    # Interpolate GT position and velocity at telemetry timestamps
    gt_p_interp = np.zeros((len(tel), 3))
    gt_v_interp = np.zeros((len(tel), 3))
    for c in range(3):
        gt_p_interp[:, c] = np.interp(tel_t_sec, gt_t, gt_p[:, c])
        gt_v_interp[:, c] = np.interp(tel_t_sec, gt_t, gt_v[:, c])

    # Compute GT orientation at t0 to align GT world to estimator frame
    idx_gt0 = np.searchsorted(gt_t, tel_t_sec[0])
    idx_gt0 = min(idx_gt0, len(gt_t)-1)
    R_gt0 = quat_to_rot_matrix(gt_q[idx_gt0])
    
    q_est0 = np.array([tel['qw'].iloc[0], tel['qx'].iloc[0], tel['qy'].iloc[0], tel['qz'].iloc[0]])
    R_est0 = quat_to_rot_matrix(q_est0)
    R_align = R_est0 @ R_gt0.T

    # Align GT position and velocity to estimator frame
    gt_p_aligned = (R_align @ (gt_p_interp - gt_p_interp[0]).T).T
    gt_v_aligned = (R_align @ gt_v_interp.T).T

    # Estimator values
    est_p = tel[['px', 'py', 'pz']].values
    est_p_shifted = est_p - est_p[0]
    est_v = tel[['vx', 'vy', 'vz']].values

    # Velocity errors
    v_err = est_v - gt_v_aligned
    v_err_norm = np.linalg.norm(v_err, axis=1)
    gt_v_norm = np.linalg.norm(gt_v_aligned, axis=1)
    est_v_norm = np.linalg.norm(est_v, axis=1)

    # Attitude errors
    yaw_err_deg = []
    pitch_err_deg = []
    roll_err_deg = []
    
    stride = max(1, len(tel) // 2000)
    eval_indices = np.arange(0, len(tel), stride)
    
    for i in eval_indices:
        t_i = tel_t_sec[i]
        idx_gt = np.searchsorted(gt_t, t_i)
        idx_gt = min(idx_gt, len(gt_t)-1)
        R_gt_i = quat_to_rot_matrix(gt_q[idx_gt])
        R_gt_w = R_align @ R_gt_i

        q_est_i = np.array([tel['qw'].iloc[i], tel['qx'].iloc[i], tel['qy'].iloc[i], tel['qz'].iloc[i]])
        R_est_i = quat_to_rot_matrix(q_est_i)

        R_err = R_est_i.T @ R_gt_w
        phi_err, th_err, psi_err = rot_matrix_to_euler(R_err)
        yaw_err_deg.append(np.degrees(np.abs(psi_err)))
        pitch_err_deg.append(np.degrees(np.abs(th_err)))
        roll_err_deg.append(np.degrees(np.abs(phi_err)))

    yaw_err_arr = np.array(yaw_err_deg)
    pitch_err_arr = np.array(pitch_err_deg)
    roll_err_arr = np.array(roll_err_deg)
    t_eval = t_rel[eval_indices]

    # Spans
    est_span = np.linalg.norm(np.ptp(est_p, axis=0))
    ref_span = np.linalg.norm(np.ptp(gt_p_aligned, axis=0))

    # Diagnostic Statistics
    zupt_fraction = np.mean(tel['is_stat'].values) * 100.0
    tilt_fraction = np.mean(tel['tilt_applied'].values) * 100.0
    vel_bound_fraction = np.mean(tel['vel_bounded'].values) * 100.0
    mean_kappa = np.mean(tel['kappa'].values)
    mean_quality = np.mean(tel['quality'].values)

    print(f"\n=======================================================")
    print(f"DIAGNOSTIC SUMMARY: {name}")
    print(f"=======================================================")
    print(f"  Duration:              {t_rel[-1]:.1f} s ({len(tel)} samples)")
    print(f"  Trajectory Span:       Est = {est_span:.2f} m | Ref = {ref_span:.2f} m (Ratio: {est_span/ref_span:.2f}x)")
    print(f"  Max Velocity:          Est = {np.max(est_v_norm):.2f} m/s | GT = {np.max(gt_v_norm):.2f} m/s")
    print(f"  RMS Velocity Error:    {np.sqrt(np.mean(v_err_norm**2)):.2f} m/s (Peak: {np.max(v_err_norm):.2f} m/s)")
    print(f"  Mean Yaw Error:        {np.mean(yaw_err_arr):.2f}° (Peak: {np.max(yaw_err_arr):.2f}°)")
    print(f"  Mean Pitch Error:      {np.mean(pitch_err_arr):.2f}° (Peak: {np.max(pitch_err_arr):.2f}°)")
    print(f"  Mean Roll Error:       {np.mean(roll_err_arr):.2f}° (Peak: {np.max(roll_err_arr):.2f}°)")
    print(f"  Active Regime Flags:")
    print(f"    - ZUPT active:       {zupt_fraction:.1f}% of total samples")
    print(f"    - Tilt applied:      {tilt_fraction:.1f}% of total samples")
    print(f"    - Vel bound active:  {vel_bound_fraction:.1f}% of total samples")
    print(f"    - Mean Kappa:        {mean_kappa:.2f}")
    print(f"    - Mean Quality:      {mean_quality:.2f}")

    # Generate 5-Panel Diagnostic Figure
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=False)
    fig.suptitle(f"Diagnostic Profile: {name} (Est Span: {est_span:.1f}m vs Ref: {ref_span:.1f}m)", fontsize=14, fontweight='bold')

    # Panel 1: XY Trajectory
    axes[0].plot(gt_p_aligned[:, 0], gt_p_aligned[:, 1], 'g-', label='Ground Truth (Ref)', linewidth=2.0)
    axes[0].plot(est_p_shifted[:, 0], est_p_shifted[:, 1], 'r--', label='Estimated Trajectory', linewidth=1.5)
    axes[0].plot(0, 0, 'ko', markersize=8, label='Start Point')
    axes[0].set_title("1. Horizontal Trajectory (XY Plane)", fontsize=11, fontweight='bold')
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].legend(loc='best')
    axes[0].grid(True, linestyle=':', alpha=0.6)
    axes[0].axis('equal')

    # Panel 2: Velocity vs Time
    axes[1].plot(t_rel, gt_v_norm, 'g-', label='GT Speed ||v||', linewidth=1.5, alpha=0.8)
    axes[1].plot(t_rel, est_v_norm, 'r--', label='Est Speed ||v||', linewidth=1.5)
    axes[1].plot(t_rel, v_err_norm, 'm:', label='Speed Error ||v_est - v_gt||', linewidth=1.2)
    axes[1].set_title("2. Vehicle Speed and Velocity Error vs Time", fontsize=11, fontweight='bold')
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].legend(loc='upper right')
    axes[1].grid(True, linestyle=':', alpha=0.6)

    # Panel 3: Attitude Errors
    axes[2].plot(t_eval, yaw_err_arr, 'b-', label='Yaw Error (deg)', linewidth=1.5)
    axes[2].plot(t_eval, pitch_err_arr, 'orange', label='Pitch Error (deg)', linewidth=1.2)
    axes[2].plot(t_eval, roll_err_arr, 'cyan', label='Roll Error (deg)', linewidth=1.2)
    axes[2].set_title("3. Attitude Errors vs Ground Truth", fontsize=11, fontweight='bold')
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Error (deg)")
    axes[2].legend(loc='upper right')
    axes[2].grid(True, linestyle=':', alpha=0.6)

    # Panel 4: Sensor Biases
    axes[3].plot(t_rel, tel['bax'], label='ba_x (m/s²)', linewidth=1.2)
    axes[3].plot(t_rel, tel['bay'], label='ba_y (m/s²)', linewidth=1.2)
    axes[3].plot(t_rel, tel['baz'], label='ba_z (m/s²)', linewidth=1.2)
    axes[3].plot(t_rel, tel['bgx']*10, '--', label='bg_x*10 (rad/s)', linewidth=1.0)
    axes[3].plot(t_rel, tel['bgy']*10, '--', label='bg_y*10 (rad/s)', linewidth=1.0)
    axes[3].plot(t_rel, tel['bgz']*10, '--', label='bg_z*10 (rad/s)', linewidth=1.0)
    axes[3].set_title("4. Estimated Biases vs Time (Accelerometer & Gyroscope)", fontsize=11, fontweight='bold')
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Bias")
    axes[3].legend(loc='upper right', ncol=2)
    axes[3].grid(True, linestyle=':', alpha=0.6)

    # Panel 5: Active Regime Flags & Quality
    axes[4].plot(t_rel, tel['is_stat'], 'm-', label='ZUPT Active', linewidth=1.2)
    axes[4].plot(t_rel, tel['tilt_applied'], 'c--', label='Tilt Update Applied', linewidth=1.0, alpha=0.7)
    axes[4].plot(t_rel, tel['vel_bounded'], 'r:', label='Vel Bounded (>3m/s)', linewidth=1.2)
    axes[4].plot(t_rel, tel['quality'], 'g-', label='Quality Metric', linewidth=1.5)
    axes[4].set_title("5. Filter Active Flags, Gates & Sensor Quality", fontsize=11, fontweight='bold')
    axes[4].set_xlabel("Time (s)")
    axes[4].set_ylabel("Flag / Value")
    axes[4].legend(loc='upper right', ncol=2)
    axes[4].grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    plot_path = os.path.join(out_dir, f"diag_{clean_name}.png")
    plt.savefig(plot_path, dpi=200)
    plt.close()
    print(f"  Saved diagnostic plot: {plot_path}")

    return {
        "name": name,
        "est_span": est_span,
        "ref_span": ref_span,
        "span_ratio": est_span / ref_span,
        "zupt_pct": zupt_fraction,
        "tilt_pct": tilt_fraction,
        "vel_bound_pct": vel_bound_fraction,
        "yaw_err_deg": np.mean(yaw_err_arr),
        "v_rms": np.sqrt(np.mean(v_err_norm**2))
    }

def run_oracle_matrix(name, imu_path, gt_path, binary="./main"):
    clean_name = name.replace(" ", "_").replace("-", "_")
    null_start, null_aligned, null_rpe = compute_null_baseline(gt_path)

    configs = [
        ("Current Baseline", []),
        ("Oracle 1 (GT Yaw)", ["--oracle-gt", gt_path, "--oracle-yaw"]),
        ("Oracle 2 (GT Bias)", ["--oracle-gt", gt_path, "--oracle-bias"]),
        ("Oracle 3 (GT v0)", ["--oracle-gt", gt_path, "--oracle-v0"]),
        ("Oracle 4 (GT Yaw + Bias + v0)", ["--oracle-gt", gt_path, "--oracle-all"]),
        ("Oracle 5 (GT Attitude R+P+Y)", ["--oracle-gt", gt_path, "--oracle-att"]),
        ("Ablation: No tau_ou (tau=0)", ["--disable-tau-ou"]),
        ("Ablation: No ZUPT", ["--disable-zupt"]),
        ("Ablation: No Tilt Update", ["--disable-tilt"]),
        ("Ablation: No Vel Bounding", ["--disable-vel-bound"]),
    ]

    results = []
    for cfg_name, flags in configs:
        out_file = f"/tmp/eval_{clean_name}_{cfg_name.replace(' ', '_').replace('(', '').replace(')', '').replace(':', '')}.txt"
        cmd = [binary, imu_path, out_file] + flags
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        m = compute_trajectory_metrics(gt_path, out_file)
        start_ate = m["start_anchored_rmse"]
        aligned_ate = m["ate_rmse_aligned"]
        rpe = m["rpe_trans_1s"]
        est_span = m["est_span"]
        ref_span = m["ref_span"]
        ratio = est_span / ref_span if ref_span > 0 else 0.0

        results.append({
            "config": cfg_name,
            "start_ate": start_ate,
            "aligned_ate": aligned_ate,
            "rpe": rpe,
            "est_span": est_span,
            "ref_span": ref_span,
            "ratio": ratio
        })

    return results, (null_start, null_aligned, null_rpe)

def main():
    print("=" * 80)
    print("TRACK 3: GROUND TRUTH DIAGNOSTIC & ORACLE HEADROOM ANALYSIS")
    print("=" * 80)

    diag_summaries = []
    for name, imu_path, gt_path in DIAG_DATASETS:
        summary = run_diagnostics_and_plots(name, imu_path, gt_path)
        diag_summaries.append(summary)

    print("\n" + "=" * 80)
    print("RUNNING ORACLE EXPERIMENTS & HEADROOM CEILING MATRIX")
    print("=" * 80)

    for name, imu_path, gt_path in DIAG_DATASETS:
        results, nulls = run_oracle_matrix(name, imu_path, gt_path)
        null_s, null_a, null_r = nulls
        print(f"\n### Dataset: {name}")
        print(f"Null Baseline: Start ATE = {null_s:.2f} m | Aligned ATE = {null_a:.2f} m | RPE 1s = {null_r:.2f} m\n")
        print(f"| {'Experiment Configuration':<32} | {'Start ATE (m)':<15} | {'Aligned ATE (m)':<17} | {'RPE 1s (m)':<12} | {'Span (Est/Ref)':<16} | {'Span Ratio':<10} |")
        print("|" + "-"*34 + "|" + "-"*17 + "|" + "-"*19 + "|" + "-"*14 + "|" + "-"*18 + "|" + "-"*12 + "|")
        for r in results:
            span_str = f"{r['est_span']:.1f}m / {r['ref_span']:.1f}m"
            print(f"| {r['config']:<32} | {r['start_ate']:>15.2f} | {r['aligned_ate']:>17.2f} | {r['rpe']:>12.2f} | {span_str:>16} | {r['ratio']:>10.2f}x |")

if __name__ == "__main__":
    main()
