#!/usr/bin/env python3
"""
main.py - Standalone Single-File 6-DoF Inertial Trajectory Estimator
Samsung XR Hackathon Track 3: Path Finder (VITC_SingleThread)

Fully self-contained:
  1. Universal Ingestion: handles .csv, .zip, directories (EuRoC, TUM-VI, or custom)
  2. Hazard Pre-Cleaner: dt<=0 duplicate handling, stuck-sensor interpolation,
     joint (acc+gyr) Hampel spike sniping, and trailing 0.0 failure detection
  3. Clean Startup Initializer: dynamic motion vs static rest calibration
  4. Unrolled Sparse 15-State ESKF:
     - InEKF SE_2(3) exact gravity transition coupling
     - High-frequency differential degradation monitor (kappa, quality)
     - Adaptive OU velocity damping and dynamic velocity envelope
     - Joseph-form covariance updates (tilt, ZUPT, rotor drag, pedestrian NHC, velocity bound)
  5. Terminal Blind Predictor: Simpson numerical curved/straight trajectory extrapolation
  6. Output: exact TUM trajectory (timestamp tx ty tz qx qy qz qw) with N_out == N_in
"""

import argparse
import collections
import io
import math
import os
import sys
import zipfile
import numpy as np

G_STD = 9.80665

# Calibrated production parameters (LOOCV baseline matching C++ release)
DEFAULT_PARAMS = {
    "T_init": 1.0,
    "T_init_fallback": 0.5,
    "g_mag_clamp_min": 9.0,
    "g_mag_clamp_max": 10.5,
    "sigma_theta_0": 0.02,
    "sigma_v_0": 0.05,
    "sigma_p_0": 0.01,
    "sigma_bg_0": 0.02,
    "sigma_ba_0": 0.20,
    "motion_detect_gyro_std": 0.20,
    "motion_detect_acc_std": 0.50,
    "motion_detect_sigma_theta_mult": 5.0,
    "noise_mult_gyro": 1.0,
    "noise_mult_acc": 1.0,
    "sigma_g_base": 0.015,
    "sigma_a_base": 0.080,
    "sigma_bg_base": 0.0001,
    "sigma_ba_base": 0.0010,
    "tau_ou": 1.9457419085342305,
    "enable_inekf_transition": True,
    "enable_adaptive_q": True,
    "adaptive_q_alpha": 0.005,
    "adaptive_q_ref": 0.0033119250641368016,
    "adaptive_q_max": 25.0,
    "enable_spike_filter": True,
    "enable_rotor_drag": True,
    "rotor_drag_kd": 0.09669368905032731,
    "rotor_drag_sigma": 0.6226596252563928,
    "rotor_drag_gate": 12.0,
    "flight_vibe_thresh": 0.40,
    "ped_energy_thresh": 0.20,
    "enable_pedestrian_prior": True,
    "ped_sigma_lat": 0.21472410214586019,
    "ped_sigma_vert": 0.6478380742938604,
    "ped_gate_threshold": 9.0,
    "ped_max_velocity_mps": 3.5,
    "enable_velocity_bounding": True,
    "max_velocity_mps": 2.7961217917434604,
    "vel_bound_sigma": 0.12749291902103269,
    "velocity_bounding_sigma": 0.12749291902103269,
    "tilt_tau_a": 0.1888081177663604,
    "tilt_omega_max": 2.2,
    "tilt_sigma": 0.02431134926953681,
    "sigma_tilt": 0.02431134926953681,
    "tilt_alpha_a": 2.0,
    "tilt_a_scale": 0.21426320932326784,
    "tilt_alpha_omega": 8.0,
    "tilt_omega_scale": 2.2,
    "zupt_window_s": 0.25,
    "zupt_gyro_mean": 0.02,
    "zupt_acc_dev": 0.15,
    "zupt_sigma": 0.01427568922533429,
    "sigma_zupt": 0.01427568922533429,
    "zupt_min_quality": 0.30,
    "monitor_alpha": 0.01,
    "kappa_max": 50.0,
    "sigma_bgm": 0.005,
    "gap_substep_max": 800,
    "tau_gap": 2.0,
    "kappa_gap": 5.0,
    "recover_steps": 100,
    "T_fail": 1.0,
    "smooth_omega_tau": 0.25,
    "ring_buffer_size": 256,
    "tau_v": 3.367753801188697,
    "tau_omega": 1.5,
    "shrink_v": 0.8573318952578088,
    "enable_regime_curvature": True,
    "regime_curvature_min_w": 0.50,
    "tau_omega_blind": 1.8265880306839262,
    "blind_window_s": 1.0,
    "enable_student_t": False,
    "nu_tilt": 8.0,
    "nu_vel": 8.0,
}


# ---------------------------------------------------------------------------
# Fast Math Primitives (Pure Python Scalars)
# ---------------------------------------------------------------------------

def rotvec_to_quat(rx: float, ry: float, rz: float):
    th2 = rx * rx + ry * ry + rz * rz
    if th2 < 1e-12:
        qw = 1.0 - 0.125 * th2
        s = 0.5 - th2 / 48.0
    else:
        th = math.sqrt(th2)
        half = 0.5 * th
        qw = math.cos(half)
        s = math.sin(half) / th
    qx = rx * s
    qy = ry * s
    qz = rz * s
    inv_n = 1.0 / math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qw * inv_n, qx * inv_n, qy * inv_n, qz * inv_n)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_norm_align(q, q_ref):
    qw, qx, qy, qz = q
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-15:
        return (1.0, 0.0, 0.0, 0.0)
    inv_n = 1.0 / n
    qw *= inv_n
    qx *= inv_n
    qy *= inv_n
    qz *= inv_n
    if qw * q_ref[0] + qx * q_ref[1] + qy * q_ref[2] + qz * q_ref[3] < 0.0:
        return (-qw, -qx, -qy, -qz)
    return (qw, qx, qy, qz)


def quat_to_rot(q):
    qw, qx, qy, qz = q
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return (
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
    )


def inv3x3(s00, s01, s02, s10, s11, s12, s20, s21, s22):
    c00 = s11 * s22 - s12 * s21
    c01 = s12 * s20 - s10 * s22
    c02 = s10 * s21 - s11 * s20
    det = s00 * c00 + s01 * c01 + s02 * c02
    if abs(det) < 1e-18:
        return None
    inv_d = 1.0 / det
    c10 = s02 * s21 - s01 * s22
    c11 = s00 * s22 - s02 * s20
    c12 = s01 * s20 - s00 * s21
    c20 = s01 * s12 - s02 * s11
    c21 = s02 * s10 - s00 * s12
    c22 = s00 * s11 - s01 * s10
    return (
        c00 * inv_d, c10 * inv_d, c20 * inv_d,
        c01 * inv_d, c11 * inv_d, c21 * inv_d,
        c02 * inv_d, c12 * inv_d, c22 * inv_d,
    )


def median3(a: float, b: float, c: float) -> float:
    if a > b:
        if b > c:
            return b
        return c if a > c else a
    if a > c:
        return a
    return c if b > c else b


# ---------------------------------------------------------------------------
# Universal Ingestion & Column/Unit Auto-Detection
# ---------------------------------------------------------------------------

def _open_imu_lines(input_path: str):
    if os.path.isdir(input_path):
        candidates = [
            os.path.join(input_path, "imu0", "data.csv"),
            os.path.join(input_path, "mav0", "imu0", "data.csv"),
            os.path.join(input_path, "data.csv"),
        ]
        csv_path = None
        for c in candidates:
            if os.path.isfile(c):
                csv_path = c
                break
        if csv_path is None:
            for root_dir, _, files in os.walk(input_path):
                for fn in sorted(files):
                    if fn.endswith(".csv"):
                        csv_path = os.path.join(root_dir, fn)
                        break
                if csv_path:
                    break
        if csv_path is None:
            raise FileNotFoundError(f"No IMU CSV found inside directory: {input_path}")
        with open(csv_path, "r", encoding="utf-8-sig") as f:
            return f.read().splitlines()

    if zipfile.is_zipfile(input_path):
        with zipfile.ZipFile(input_path, "r") as zf:
            names = [n for n in zf.namelist() if n.endswith(".csv")]
            target = None
            for n in names:
                if n.endswith("imu0/data.csv"):
                    target = n
                    break
            if target is None and names:
                target = sorted(names)[0]
            if target is None:
                raise FileNotFoundError(f"No CSV found in zip: {input_path}")
            raw = zf.read(target).decode("utf-8-sig")
            return raw.splitlines()

    with open(input_path, "r", encoding="utf-8-sig") as f:
        return f.read().splitlines()


def load_imu_arrays(input_path: str):
    lines = _open_imu_lines(input_path)
    if not lines:
        raise ValueError("Empty IMU file")

    header_cols = None
    start_idx = 0
    for idx, line in enumerate(lines):
        s = line.strip()
        if not s:
            continue
        parts = [p.strip() for p in s.split(",")]
        try:
            float(parts[0])
            start_idx = idx
            break
        except ValueError:
            header_cols = parts
            start_idx = idx + 1
            break

    data_rows = []
    ts_strs = []
    for line in lines[start_idx:]:
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        parts = [p.strip() for p in (s.split(",") if "," in s else s.split())]
        if not parts or not parts[0]:
            continue
        try:
            t_val = float(parts[0])
        except ValueError:
            continue

        row_vals = [np.nan] * 6
        for k in range(6):
            if k + 1 < len(parts):
                try:
                    v = float(parts[k + 1])
                    if math.isfinite(v):
                        row_vals[k] = v
                except ValueError:
                    pass
        data_rows.append((t_val, *row_vals))
        ts_strs.append(parts[0])

    n = len(data_rows)
    if n == 0:
        raise ValueError("No valid IMU numeric rows parsed")

    arr = np.array(data_rows, dtype=np.float64)
    ts_raw = arr[:, 0]
    c1 = arr[:, 1:4]
    c2 = arr[:, 4:7]

    # Timestamp scale detection (nanoseconds vs seconds)
    first_token = ts_strs[0]
    if "." not in first_token and abs(ts_raw[0]) >= 1e8:
        ts_scale = 1e-9
    else:
        ts_scale = 1.0

    if ts_scale == 1e-9:
        try:
            t0_int = int(ts_strs[0])
            t_s = np.array([float(int(s) - t0_int) * 1e-9 for s in ts_strs], dtype=np.float64)
        except ValueError:
            t_s = (ts_raw - ts_raw[0]) * 1e-9
    else:
        t_s = ts_raw * ts_scale

    # Column order auto-detection: accel-first (degrade_imu.py) vs gyro-first (EuRoC)
    gyro_first = False
    if header_cols is not None and len(header_cols) >= 7:
        h1 = header_cols[1].lower()
        h4 = header_cols[4].lower()
        if ("w_" in h1 or "gyr" in h1 or "gx" in h1) and ("a_" in h4 or "acc" in h4 or "ax" in h4):
            gyro_first = True
        elif ("a_" in h1 or "acc" in h1 or "ax" in h1) and ("w_" in h4 or "gyr" in h4 or "gx" in h4):
            gyro_first = False
        else:
            gyro_first = _detect_gyro_first_by_norm(c1, c2)
    else:
        gyro_first = _detect_gyro_first_by_norm(c1, c2)

    if gyro_first:
        gyr = c1.copy()
        acc = c2.copy()
    else:
        acc = c1.copy()
        gyr = c2.copy()

    # Auto-detect accel unit: m/s^2 vs g
    acc_norms = np.linalg.norm(acc, axis=1)
    valid_norms = acc_norms[np.isfinite(acc_norms) & (acc_norms > 0.1)]
    if len(valid_norms) > 0:
        med_norm = float(np.median(valid_norms))
        if 0.5 < med_norm < 2.0:
            acc *= G_STD

    return ts_strs, t_s, acc, gyr, ts_scale


def _detect_gyro_first_by_norm(c1: np.ndarray, c2: np.ndarray) -> bool:
    n1 = np.linalg.norm(c1, axis=1)
    n2 = np.linalg.norm(c2, axis=1)
    v1 = n1[np.isfinite(n1) & (n1 > 1e-5)]
    v2 = n2[np.isfinite(n2) & (n2 > 1e-5)]
    m1 = float(np.median(v1)) if len(v1) > 0 else 0.0
    m2 = float(np.median(v2)) if len(v2) > 0 else 0.0
    d1 = min(abs(m1 - G_STD), abs(m1 - 1.0) * G_STD)
    d2 = min(abs(m2 - G_STD), abs(m2 - 1.0) * G_STD)
    return d2 < d1


# ---------------------------------------------------------------------------
# Vectorized Offline Pre-Cleaner for degrade_imu.py Hazards
# ---------------------------------------------------------------------------

def preprocess_offline(t_s: np.ndarray, acc: np.ndarray, gyr: np.ndarray, params: dict):
    n = len(t_s)
    dup_mask = np.zeros(n, dtype=bool)
    if n > 1:
        dup_mask[1:] = t_s[1:] <= t_s[:-1]

    finite_mask = np.all(np.isfinite(acc), axis=1) & np.all(np.isfinite(gyr), axis=1)
    acc_clean = np.where(np.isfinite(acc), acc, 0.0)
    gyr_clean = np.where(np.isfinite(gyr), gyr, 0.0)

    zero_mask = (
        (np.max(np.abs(acc_clean), axis=1) < 1e-6) &
        (np.max(np.abs(gyr_clean), axis=1) < 1e-6)
    )
    # Detect trailing contiguous [0..0] failure segment (degrade_imu.py failure_seconds)
    fail_start = n
    for k in range(n - 1, -1, -1):
        if zero_mask[k] or not finite_mask[k]:
            fail_start = k
        else:
            break

    # Extreme shock mask (> 160 m/s^2 or > 35 rad/s)
    extreme_mask = (
        (np.max(np.abs(acc_clean), axis=1) > 160.0) |
        (np.max(np.abs(gyr_clean), axis=1) > 35.0)
    )
    invalid_mask = (~finite_mask) | zero_mask | extreme_mask

    # Work on unique valid interior indices for stuck interpolation & joint spike sniping
    active_idx = np.flatnonzero((~dup_mask) & (~invalid_mask) & (np.arange(n) < fail_start))
    m = len(active_idx)
    if m >= 3:
        a_act = acc_clean[active_idx].copy()
        g_act = gyr_clean[active_idx].copy()
        t_act = t_s[active_idx]

        # 1. Stuck sensor run detection & linear interpolation
        same_prev = (
            np.all(np.abs(a_act[1:] - a_act[:-1]) < 1e-10, axis=1) &
            np.all(np.abs(g_act[1:] - g_act[:-1]) < 1e-10, axis=1)
        )
        i = 0
        while i < m - 1:
            if same_prev[i]:
                j = i + 1
                while j < m - 1 and same_prev[j]:
                    j += 1
                run_len = j - i
                if i >= 0 and (j + 1) < m and (t_act[j + 1] - t_act[i]) < 0.25 and run_len <= 20:
                    a0, a1 = a_act[i], a_act[j + 1]
                    g0, g1 = g_act[i], g_act[j + 1]
                    dt_span = t_act[j + 1] - t_act[i]
                    if dt_span > 1e-9:
                        for p_k in range(i + 1, j + 1):
                            alpha = (t_act[p_k] - t_act[i]) / dt_span
                            a_act[p_k] = (1.0 - alpha) * a0 + alpha * a1
                            g_act[p_k] = (1.0 - alpha) * g0 + alpha * g1
                i = j + 1
            else:
                i += 1

        # 2. Joint single-sample spike sniping (centered Hampel)
        if m >= 5:
            a_mid = 0.5 * (a_act[:-2] + a_act[2:])
            g_mid = 0.5 * (g_act[:-2] + g_act[2:])
            da = np.linalg.norm(a_act[1:-1] - a_mid, axis=1)
            dg = np.linalg.norm(g_act[1:-1] - g_mid, axis=1)
            da_nbr = np.linalg.norm(a_act[2:] - a_act[:-2], axis=1)
            dg_nbr = np.linalg.norm(g_act[2:] - g_act[:-2], axis=1)

            mad_a = max(0.05, float(np.median(da)))
            mad_g = max(0.005, float(np.median(dg)))
            joint_spike = (
                (da > max(1.8, 6.0 * mad_a)) & (da > 1.8 * da_nbr + 0.5) &
                (dg > max(0.25, 6.0 * mad_g)) & (dg > 1.8 * dg_nbr + 0.08)
            )
            gyr_only_spike = (dg > max(0.42, 10.0 * mad_g)) & (dg > 2.5 * dg_nbr + 0.15)
            spike_mask = joint_spike | gyr_only_spike
            if np.any(spike_mask):
                s_idx = np.flatnonzero(spike_mask) + 1
                a_act[s_idx] = 0.5 * (a_act[s_idx - 1] + a_act[s_idx + 1])
                g_act[s_idx] = 0.5 * (g_act[s_idx - 1] + g_act[s_idx + 1])

        acc_clean[active_idx] = a_act
        gyr_clean[active_idx] = g_act

    # 3. Pre-median gyro noise tracker (Adaptive Q)
    noise_mult = np.ones(n, dtype=np.float64)
    if params["enable_adaptive_q"] and m >= 4:
        g_raw_act = gyr_clean[active_idx]
        d2 = g_raw_act[2:] - 2.0 * g_raw_act[1:-1] + g_raw_act[:-2]
        d_bar = np.mean(np.abs(d2), axis=1)
        d_clamped = np.minimum(d_bar, 0.25)
        alpha_q = params["adaptive_q_alpha"]
        ref_g = params["adaptive_q_ref"]
        max_q = params["adaptive_q_max"]
        c_mad = math.sqrt(math.pi / 12.0) * 1.4364
        ewma_d = float(d_clamped[0])
        mult_act = np.ones(m, dtype=np.float64)
        for idx_k in range(len(d_clamped)):
            ewma_d = (1.0 - alpha_q) * ewma_d + alpha_q * float(d_clamped[idx_k])
            sig_g = ewma_d * c_mad
            kq = sig_g / ref_g
            if kq < 1.0:
                kq = 1.0
            elif kq > max_q:
                kq = max_q
            mult_act[idx_k + 2] = kq
        mult_act[0] = mult_act[2] if m > 2 else 1.0
        mult_act[1] = mult_act[2] if m > 2 else 1.0
        noise_mult[active_idx] = mult_act

    # 4. Causal 3-tap median filter on gyro
    if params["enable_spike_filter"] and m >= 3:
        g_src = gyr_clean[active_idx]
        g_med = g_src.copy()
        t_act = t_s[active_idx]
        for idx_k in range(1, m - 1):
            if (t_act[idx_k] - t_act[idx_k - 1] <= 0.05) and (t_act[idx_k + 1] - t_act[idx_k] <= 0.05):
                g_med[idx_k, 0] = median3(float(g_src[idx_k - 1, 0]), float(g_src[idx_k, 0]), float(g_src[idx_k + 1, 0]))
                g_med[idx_k, 1] = median3(float(g_src[idx_k - 1, 1]), float(g_src[idx_k, 1]), float(g_src[idx_k + 1, 1]))
                g_med[idx_k, 2] = median3(float(g_src[idx_k - 1, 2]), float(g_src[idx_k, 2]), float(g_src[idx_k + 1, 2]))
        gyr_clean[active_idx] = g_med

    # Nominal dt
    valid_dts = np.diff(t_s[active_idx]) if m >= 2 else np.array([0.005])
    pos_dts = valid_dts[(valid_dts > 1e-5) & (valid_dts < 0.1)]
    dt_nom = float(np.median(pos_dts)) if len(pos_dts) > 0 else 0.005

    return acc_clean, gyr_clean, dup_mask, invalid_mask, fail_start, noise_mult, dt_nom


# ---------------------------------------------------------------------------
# Unrolled Sparse 15-State ESKF
# ---------------------------------------------------------------------------

class DegradationMonitor:
    __slots__ = (
        "alpha", "kappa_max", "sample_count",
        "prev_a", "prev_w", "has_prev",
        "var_acc", "var_gyr", "var_acc_ref", "var_gyr_ref",
        "calibrated", "quality", "kappa"
    )

    def __init__(self, alpha: float = 0.01, kappa_max: float = 50.0):
        self.alpha = alpha
        self.kappa_max = kappa_max
        self.reset()

    def reset(self):
        self.sample_count = 0
        self.prev_a = (0.0, 0.0, 0.0)
        self.prev_w = (0.0, 0.0, 0.0)
        self.has_prev = False
        self.var_acc = 0.0
        self.var_gyr = 0.0
        self.var_acc_ref = 0.0
        self.var_gyr_ref = 0.0
        self.calibrated = False
        self.quality = 1.0
        self.kappa = 1.0

    def step(self, t_elapsed: float, ax: float, ay: float, az: float, gx: float, gy: float, gz: float, t_init: float):
        if not self.has_prev:
            self.prev_a = (ax, ay, az)
            self.prev_w = (gx, gy, gz)
            self.has_prev = True
            self.sample_count = 1
            return

        # High-frequency differential residual between adjacent samples
        da_x = ax - self.prev_a[0]
        da_y = ay - self.prev_a[1]
        da_z = az - self.prev_a[2]
        r_k = math.sqrt(da_x * da_x + da_y * da_y + da_z * da_z)

        dw_x = gx - self.prev_w[0]
        dw_y = gy - self.prev_w[1]
        dw_z = gz - self.prev_w[2]
        rho_k = math.sqrt(dw_x * dw_x + dw_y * dw_y + dw_z * dw_z)

        self.prev_a = (ax, ay, az)
        self.prev_w = (gx, gy, gz)

        # Exponential moving variance of residuals
        alpha = self.alpha
        if self.sample_count == 1:
            self.var_acc = r_k * r_k
            self.var_gyr = rho_k * rho_k
        else:
            self.var_acc = (1.0 - alpha) * self.var_acc + alpha * (r_k * r_k)
            self.var_gyr = (1.0 - alpha) * self.var_gyr + alpha * (rho_k * rho_k)
        self.sample_count += 1

        # Calibration window check
        if t_elapsed < t_init:
            self.var_acc_ref = self.var_acc
            self.var_gyr_ref = self.var_gyr
            self.quality = 1.0
            self.kappa = 1.0
            self.calibrated = False
        else:
            if not self.calibrated:
                self.calibrated = True
                if self.var_acc_ref <= 0.0:
                    self.var_acc_ref = self.var_acc
                if self.var_gyr_ref <= 0.0:
                    self.var_gyr_ref = self.var_gyr

            sig_a_now = math.sqrt(max(0.0, self.var_acc))
            sig_g_now = math.sqrt(max(0.0, self.var_gyr))
            sig_a_ref = math.sqrt(max(0.0, self.var_acc_ref))
            sig_g_ref = math.sqrt(max(0.0, self.var_gyr_ref))

            n_acc = (sig_a_now / sig_a_ref) if sig_a_ref > 1e-12 else 1.0
            n_gyr = (sig_g_now / sig_g_ref) if sig_g_ref > 1e-12 else 1.0
            noise_ratio = max(1.0, max(n_acc, n_gyr))

            self.quality = max(0.01, min(1.0, 1.0 / noise_ratio))
            self.kappa = max(1.0, min(self.kappa_max, noise_ratio))


class SparseESKF:
    __slots__ = (
        "p", "v", "q", "bg", "ba", "P", "g_mag", "params",
        "vibe_ema", "ped_energy", "gyro_ema", "dyn_acc_ema",
        "prev_a_raw", "has_prev_raw",
        "recover_counter", "win_w", "win_a_norm",
        "ring", "ring_cap", "monitor", "q_scale_g", "q_scale_a",
        "fp", "hp0", "hp1", "hp2", "k0", "k1", "k2", "w0", "w1", "w2", "dx"
    )

    def __init__(self, params: dict):
        self.params = params
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.bg = [0.0, 0.0, 0.0]
        self.ba = [0.0, 0.0, 0.0]
        self.P = [[0.0] * 15 for _ in range(15)]
        self.g_mag = G_STD

        self.vibe_ema = 0.0
        self.ped_energy = 0.0
        self.gyro_ema = 0.0
        self.dyn_acc_ema = 0.0
        self.prev_a_raw = [0.0, 0.0, 0.0]
        self.has_prev_raw = False
        self.recover_counter = 0

        self.ring_cap = params.get("ring_buffer_size", 256)
        self.win_w = collections.deque(maxlen=50)
        self.win_a_norm = collections.deque(maxlen=50)
        self.ring = collections.deque(maxlen=self.ring_cap)

        self.monitor = DegradationMonitor(
            alpha=params.get("monitor_alpha", 0.01),
            kappa_max=params.get("kappa_max", 50.0)
        )
        self.q_scale_g = 1.0
        self.q_scale_a = 1.0

        # Preallocated scratch buffers (zero per-frame heap allocation)
        self.fp = [[0.0] * 15 for _ in range(9)]
        self.hp0 = [0.0] * 15
        self.hp1 = [0.0] * 15
        self.hp2 = [0.0] * 15
        self.k0 = [0.0] * 15
        self.k1 = [0.0] * 15
        self.k2 = [0.0] * 15
        self.w0 = [0.0] * 15
        self.w1 = [0.0] * 15
        self.w2 = [0.0] * 15
        self.dx = [0.0] * 15

    def initialize(self, t_arr, acc_arr, gyr_arr, dup_mask, invalid_mask, end_init_idx: int):
        wx_sum = wy_sum = wz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        cnt = 0
        ws = []
        anorms = []
        first_valid_idx = -1

        for k in range(end_init_idx):
            if dup_mask[k] or invalid_mask[k]:
                continue
            if first_valid_idx < 0:
                first_valid_idx = k
            ax, ay, az = float(acc_arr[k, 0]), float(acc_arr[k, 1]), float(acc_arr[k, 2])
            gx, gy, gz = float(gyr_arr[k, 0]), float(gyr_arr[k, 1]), float(gyr_arr[k, 2])
            wx_sum += gx
            wy_sum += gy
            wz_sum += gz
            ax_sum += ax
            ay_sum += ay
            az_sum += az
            ws.append((gx, gy, gz))
            anorms.append(math.sqrt(ax * ax + ay * ay + az * az))
            cnt += 1

        motion_detected = False
        if cnt < 2:
            if cnt == 1 and first_valid_idx >= 0:
                a_ref = (float(acc_arr[first_valid_idx, 0]),
                         float(acc_arr[first_valid_idx, 1]),
                         float(acc_arr[first_valid_idx, 2]))
                self.bg = [0.0, 0.0, 0.0]
                self.g_mag = G_STD
            else:
                self.q = (1.0, 0.0, 0.0, 0.0)
                self.g_mag = G_STD
                self._set_p0(False)
                return
        else:
            inv_c = 1.0 / cnt
            w_bar = (wx_sum * inv_c, wy_sum * inv_c, wz_sum * inv_c)
            a_bar = (ax_sum * inv_c, ay_sum * inv_c, az_sum * inv_c)

            w_var = sum(
                (g[0] - w_bar[0]) ** 2 + (g[1] - w_bar[1]) ** 2 + (g[2] - w_bar[2]) ** 2
                for g in ws
            ) / (3.0 * cnt)
            a_mean_n = sum(anorms) * inv_c
            a_var = sum((an - a_mean_n) ** 2 for an in anorms) / cnt
            sigma_w = math.sqrt(w_var)
            sigma_a = math.sqrt(a_var)
            w_bar_mag = math.sqrt(w_bar[0] ** 2 + w_bar[1] ** 2 + w_bar[2] ** 2)

            motion_detected = (
                (sigma_w > self.params["motion_detect_gyro_std"]) or
                (sigma_a > self.params["motion_detect_acc_std"]) or
                (w_bar_mag > 0.20)
            )

            # Reference accel for initial tilt quaternion
            if motion_detected and first_valid_idx >= 0:
                a_ref = (float(acc_arr[first_valid_idx, 0]),
                         float(acc_arr[first_valid_idx, 1]),
                         float(acc_arr[first_valid_idx, 2]))
                self.bg = [0.0, 0.0, 0.0]
                self.g_mag = G_STD
            else:
                a_ref = a_bar
                self.bg = [w_bar[0], w_bar[1], w_bar[2]]
                a_norm = math.sqrt(a_ref[0] ** 2 + a_ref[1] ** 2 + a_ref[2] ** 2)
                self.g_mag = max(self.params["g_mag_clamp_min"], min(self.params["g_mag_clamp_max"], a_norm))

            scale_g = (sigma_w / 0.020) if sigma_w > 1e-5 else 0.2
            scale_a = (sigma_a / 0.300) if sigma_a > 1e-5 else 0.2
            self.q_scale_g = max(0.2, min(1.0, scale_g))
            self.q_scale_a = max(0.2, min(1.0, scale_a))

        a_ref_n = math.sqrt(a_ref[0] ** 2 + a_ref[1] ** 2 + a_ref[2] ** 2)
        if a_ref_n < 1e-6:
            a_ref_n = G_STD

        vx, vy, vz = a_ref[0] / a_ref_n, a_ref[1] / a_ref_n, a_ref[2] / a_ref_n
        if 1.0 + vz < 1e-4:
            self.q = (0.0, 1.0, 0.0, 0.0)
        else:
            qw = 1.0 + vz
            qx = vy
            qy = -vx
            qz = 0.0
            inv_qn = 1.0 / math.sqrt(qw * qw + qx * qx + qy * qy)
            self.q = (qw * inv_qn, qx * inv_qn, qy * inv_qn, 0.0)

        self.ba = [0.0, 0.0, 0.0]
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self._set_p0(motion_detected)

    def _set_p0(self, motion_detected: bool):
        P = self.P
        for i in range(15):
            row = P[i]
            for j in range(15):
                row[j] = 0.0
        sig_th = self.params["sigma_theta_0"] * (self.params["motion_detect_sigma_theta_mult"] if motion_detected else 1.0)
        sig_v = self.params["sigma_v_0"]
        sig_p = self.params["sigma_p_0"]
        sig_bg = self.params["sigma_bg_0"]
        sig_ba = self.params["sigma_ba_0"]

        for i in range(3):
            P[i][i] = sig_th * sig_th
            P[i + 3][i + 3] = sig_v * sig_v
            P[i + 6][i + 6] = sig_p * sig_p
            P[i + 9][i + 9] = sig_bg * sig_bg
            P[i + 12][i + 12] = sig_ba * sig_ba

    def get_w_walk(self) -> float:
        if self.vibe_ema > self.params["flight_vibe_thresh"]:
            vibe_score = max(0.0, 1.0 - (self.vibe_ema - 0.40) / 0.40)
        else:
            vibe_score = 1.0
        spd = math.sqrt(self.v[0] ** 2 + self.v[1] ** 2 + self.v[2] ** 2)
        if spd < 0.15:
            speed_score = 0.0
        elif spd < 0.40:
            speed_score = (spd - 0.15) / 0.25
        elif spd <= 2.30:
            speed_score = 1.0
        else:
            speed_score = max(0.0, 1.0 - (spd - 2.30) / 1.00)
        ped_score = min(1.0, self.ped_energy / 0.45)
        return max(0.0, min(1.0, vibe_score * speed_score * (0.2 + 0.8 * ped_score)))

    def step(self, dt: float, ax: float, ay: float, az: float, gx: float, gy: float, gz: float,
             noise_mult: float, t_elapsed: float):
        if dt <= 0.0:
            return
        pr = self.params
        bg = self.bg
        ba = self.ba
        wx = gx - bg[0]
        wy = gy - bg[1]
        wz = gz - bg[2]
        acx = ax - ba[0]
        acy = ay - ba[1]
        acz = az - ba[2]

        w_mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)

        # Update degradation monitor
        self.monitor.step(t_elapsed, ax, ay, az, gx, gy, gz, pr["T_init"])
        quality = self.monitor.quality
        kappa = self.monitor.kappa

        # EMA trackers for flight/pedestrian regime and dynamic velocity envelope
        if not self.has_prev_raw:
            self.prev_a_raw = [ax, ay, az]
            self.has_prev_raw = True
        else:
            da_x = ax - self.prev_a_raw[0]
            da_y = ay - self.prev_a_raw[1]
            da_z = az - self.prev_a_raw[2]
            d_a = math.sqrt(da_x * da_x + da_y * da_y + da_z * da_z)
            self.vibe_ema = 0.98 * self.vibe_ema + 0.02 * d_a
            dev_a = abs(a_norm - self.g_mag)
            self.ped_energy = 0.99 * self.ped_energy + 0.01 * (dev_a * dev_a)
            if quality >= 0.5 and dev_a < 12.0:
                a_excess = max(0.0, a_norm - self.g_mag)
                self.dyn_acc_ema = 0.95 * self.dyn_acc_ema + 0.05 * a_excess
            self.prev_a_raw = [ax, ay, az]

        self.gyro_ema = 0.98 * self.gyro_ema + 0.02 * w_mag
        is_flight = (self.vibe_ema > pr["flight_vibe_thresh"])
        is_pedestrian = (not is_flight and self.ped_energy > pr["ped_energy_thresh"])

        # Calm flight adaptive OU time constant
        cur_tau_ou = pr["tau_ou"]
        if is_flight and (self.gyro_ema < 0.40):
            cur_tau_ou = pr["tau_ou"] / max(1.0, min(2.5, noise_mult))

        # Dynamic velocity bounding envelope
        dyn_envelope = pr["max_velocity_mps"] + (1.0 if is_flight else 0.0) * self.dyn_acc_ema
        if dyn_envelope > 15.0:
            dyn_envelope = 15.0
        max_v = dyn_envelope if is_flight else (pr.get("ped_max_velocity_mps", 3.5) if is_pedestrian else dyn_envelope)

        # 1. Midpoint strapdown propagation
        q_cur = self.q
        q_half = rotvec_to_quat(0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt)
        q_mid = quat_mul(q_cur, q_half)
        rm00, rm01, rm02, rm10, rm11, rm12, rm20, rm21, rm22 = quat_to_rot(q_mid)

        q_step = rotvec_to_quat(wx * dt, wy * dt, wz * dt)
        q_new = quat_norm_align(quat_mul(q_cur, q_step), q_cur)

        awx = rm00 * acx + rm01 * acy + rm02 * acz
        awy = rm10 * acx + rm11 * acy + rm12 * acz
        awz = rm20 * acx + rm21 * acy + rm22 * acz - self.g_mag

        gam = math.exp(-dt / cur_tau_ou)
        vx_old, vy_old, vz_old = self.v
        vx_new = gam * vx_old + awx * dt
        vy_new = gam * vy_old + awy * dt
        vz_new = gam * vz_old + awz * dt

        half_dt = 0.5 * dt
        self.p[0] += (vx_old + vx_new) * half_dt
        self.p[1] += (vy_old + vy_new) * half_dt
        self.p[2] += (vz_old + vz_new) * half_dt
        self.v[0] = vx_new
        self.v[1] = vy_new
        self.v[2] = vz_new
        self.q = q_new

        # 2. Sparse Covariance Propagation
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(q_cur)
        f00, f01, f02, f10, f11, f12, f20, f21, f22 = quat_to_rot(
            rotvec_to_quat(-wx * dt, -wy * dt, -wz * dt)
        )

        gdt = self.g_mag * dt
        if pr["enable_inekf_transition"]:
            ft00, ft01, ft02 = gdt * r10, gdt * r11, gdt * r12
            ft10, ft11, ft12 = -gdt * r00, -gdt * r01, -gdt * r02
            ft20, ft21, ft22 = 0.0, 0.0, 0.0
        else:
            ax_s, ay_s, az_s = acx, acy, acz
            ft00 = -(r01 * az_s - r02 * ay_s) * dt
            ft01 = -(-r00 * az_s + r02 * ax_s) * dt
            ft02 = -(r00 * ay_s - r01 * ax_s) * dt
            ft10 = -(r11 * az_s - r12 * ay_s) * dt
            ft11 = -(-r10 * az_s + r12 * ax_s) * dt
            ft12 = -(r10 * ay_s - r11 * ax_s) * dt
            ft20 = -(r21 * az_s - r22 * ay_s) * dt
            ft21 = -(-r20 * az_s + r22 * ax_s) * dt
            ft22 = -(r20 * ay_s - r21 * ax_s) * dt

        mult_g = max(noise_mult * self.q_scale_g, kappa)
        mult_a = max(self.q_scale_a, kappa)
        q_th = (pr["sigma_g_base"] * mult_g) ** 2 * dt
        q_v = (pr["sigma_a_base"] * mult_a) ** 2 * dt
        q_bg = (pr["sigma_bg_base"] ** 2) * dt
        q_ba = (pr["sigma_ba_base"] ** 2) * dt

        self._propagate_cov(
            dt, gam,
            f00, f01, f02, f10, f11, f12, f20, f21, f22,
            ft00, ft01, ft02, ft10, ft11, ft12, ft20, ft21, ft22,
            r00, r01, r02, r10, r11, r12, r20, r21, r22,
            q_th, q_v, q_bg, q_ba,
        )

        r_scale = 1.0
        if self.recover_counter > 0:
            r_scale = 1.0 + 10.0 * (self.recover_counter / float(pr["recover_steps"]))
            self.recover_counter -= 1

        # 3. Stationary Detection (ZUPT)
        self.win_w.append(w_mag)
        self.win_a_norm.append(a_norm)

        is_zupt = False
        if len(self.win_w) >= 20 and quality >= pr.get("zupt_min_quality", 0.30):
            inv_k = 1.0 / len(self.win_w)
            w_mean_n = sum(self.win_w) * inv_k
            a_mean_n = sum(self.win_a_norm) * inv_k
            sum_var = 0.0
            for an in self.win_a_norm:
                d = an - a_mean_n
                sum_var += d * d
            a_var_n = sum_var * inv_k
            zupt_lim = pr.get("zupt_omega_thresh", 0.02)
            if (w_mean_n < zupt_lim and
                self.win_w[-1] <= zupt_lim * 1.5 and
                abs(a_mean_n - self.g_mag) < pr["zupt_acc_dev"] and
                a_var_n < (pr["zupt_acc_dev"] ** 2)):
                is_zupt = True

        # 4. Regime Updates
        if is_zupt:
            sig_z = pr["sigma_zupt"]
            r_z = (sig_z * sig_z) * r_scale
            self._update_block3(
                3, -self.v[0], -self.v[1], -self.v[2],
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                r_z, r_z, r_z, gate_threshold=1e9, clear_pv=False,
                m_dim=3, nu=pr["nu_vel"]
            )
        elif is_flight and pr["enable_rotor_drag"]:
            if w_mag <= 2.5 and (3.0 <= a_norm <= 35.0):
                r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(self.q)
                vx, vy, vz = self.v
                vbx = r00 * vx + r10 * vy + r20 * vz
                vby = r01 * vx + r11 * vy + r21 * vz
                kd = pr["rotor_drag_kd"]
                y0 = acx + kd * vbx
                y1 = acy + kd * vby
                sd = pr["rotor_drag_sigma"]
                rd = (sd * sd) * r_scale
                self._update_block3(
                    3, y0, y1, 0.0,
                    -kd * r00, -kd * r10, -kd * r20,
                    -kd * r01, -kd * r11, -kd * r21,
                    0.0, 0.0, 0.0,
                    rd, rd, 1e8, gate_threshold=pr.get("rotor_drag_gate", 12.0), clear_pv=True,
                    m_dim=2, nu=pr["nu_vel"]
                )
        elif is_pedestrian and pr["enable_pedestrian_prior"]:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(self.q)
            vx, vy, vz = self.v
            vby = r01 * vx + r11 * vy + r21 * vz
            rlat = (pr["ped_sigma_lat"] ** 2) * r_scale
            rvert = (pr["ped_sigma_vert"] ** 2) * r_scale
            self._update_block3(
                3, -vby, -vz, 0.0,
                r01, r11, r21,
                0.0, 0.0, 1.0,
                0.0, 0.0, 0.0,
                rlat, rvert, 1e8, gate_threshold=pr.get("ped_gate_threshold", 9.0), clear_pv=True,
                m_dim=2, nu=pr["nu_vel"]
            )

        # 5. Tilt update
        tau_base = pr["tilt_tau_a"] * max(1.0, min(3.0, noise_mult))
        q_clamped = max(0.0, min(1.0, quality))
        tau_eff = tau_base * (1.0 + 5.0 * (1.0 - q_clamped))
        a_hat_norm = math.sqrt(acx * acx + acy * acy + acz * acz)

        if w_mag < pr["tilt_omega_max"] and abs(a_hat_norm - self.g_mag) < tau_eff and a_hat_norm > 1e-6:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(self.q)
            inv_an = 1.0 / a_hat_norm
            zx, zy, zz = acx * inv_an, acy * inv_an, acz * inv_an
            hx, hy, hz = r20, r21, r22

            sigma_tilt = pr["tilt_sigma"] / max(0.1, quality)
            var_tilt = (sigma_tilt * sigma_tilt) * r_scale
            a_ratio = abs(a_hat_norm - self.g_mag) / pr["tilt_a_scale"]
            w_ratio = w_mag / pr["tilt_omega_scale"]
            dyn_scale = 1.0 + pr["tilt_alpha_a"] * (a_ratio * a_ratio) + pr["tilt_alpha_omega"] * (w_ratio * w_ratio)
            base_rt = var_tilt * min(1e6, dyn_scale)

            self._update_block3(
                0, zx - hx, zy - hy, zz - hz,
                0.0, -hz, hy,
                hz, 0.0, -hx,
                -hy, hx, 0.0,
                base_rt, base_rt, base_rt, gate_threshold=1e9, clear_pv=False,
                m_dim=3, nu=pr["nu_tilt"]
            )

        # 6. Velocity Bounding
        if pr["enable_velocity_bounding"]:
            vx, vy, vz = self.v
            vmag = math.sqrt(vx * vx + vy * vy + vz * vz)
            if vmag > max_v and vmag > 1e-9:
                sc = max_v / vmag - 1.0
                rv = (pr["velocity_bounding_sigma"] ** 2) * r_scale
                self._update_block3(
                    3, vx * sc, vy * sc, vz * sc,
                    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                    rv, rv, rv, gate_threshold=1e9, clear_pv=True,
                    m_dim=3, nu=pr["nu_vel"]
                )

        # 7. Reliable state buffering for blind window
        w_walk = self.get_w_walk()
        self.ring.append((
            self.p[0], self.p[1], self.p[2],
            self.v[0], self.v[1], self.v[2],
            self.q[0], self.q[1], self.q[2], self.q[3],
            wx, wy, wz, w_walk
        ))
        if len(self.ring) > self.ring_cap:
            self.ring.pop(0)

    def propagate_gap(self, gap_dt: float, dt_nom: float, wx_s: float, wy_s: float, wz_s: float):
        if gap_dt <= 0.0:
            return
        pr = self.params
        n_sub = max(1, int(round(gap_dt / max(dt_nom, 1e-4))))
        dt_sub = gap_dt / n_sub
        dec = math.exp(-dt_sub / pr["tau_gap"])
        half_dt = 0.5 * dt_sub

        q_cur = self.q
        q_step = rotvec_to_quat(wx_s * dt_sub, wy_s * dt_sub, wz_s * dt_sub)
        vx, vy, vz = self.v
        px, py, pz = self.p
        for _ in range(n_sub):
            vx_n = vx * dec
            vy_n = vy * dec
            vz_n = vz * dec
            px += (vx + vx_n) * half_dt
            py += (vy + vy_n) * half_dt
            pz += (vz + vz_n) * half_dt
            vx, vy, vz = vx_n, vy_n, vz_n
            q_cur = quat_norm_align(quat_mul(q_cur, q_step), q_cur)

        self.p[0], self.p[1], self.p[2] = px, py, pz
        self.v[0], self.v[1], self.v[2] = vx, vy, vz
        self.q = q_cur

        P = self.P
        k_gap = pr["kappa_gap"]
        q_th = (pr["sigma_g_base"] ** 2) * k_gap * gap_dt
        q_v = (pr["sigma_a_base"] ** 2) * k_gap * gap_dt
        q_p = 0.01 * gap_dt
        for i in range(3):
            P[i][i] = min(1e6, P[i][i] + q_th)
            P[i + 3][i + 3] = min(1e6, P[i + 3][i + 3] + q_v)
            P[i + 6][i + 6] = min(1e6, P[i + 6][i + 6] + q_p)
        self.recover_counter = pr["recover_steps"]

    def _propagate_cov(
        self, dt, gam,
        f00, f01, f02, f10, f11, f12, f20, f21, f22,
        ft00, ft01, ft02, ft10, ft11, ft12, ft20, ft21, ft22,
        r00, r01, r02, r10, r11, r12, r20, r21, r22,
        q_th, q_v, q_bg, q_ba,
    ):
        P = self.P
        p0, p1, p2 = P[0], P[1], P[2]
        p3, p4, p5 = P[3], P[4], P[5]
        p6, p7, p8 = P[6], P[7], P[8]
        p9, p10, p11 = P[9], P[10], P[11]
        p12, p13, p14 = P[12], P[13], P[14]

        fps = self.fp
        fp0, fp1, fp2, fp3, fp4, fp5, fp6, fp7, fp8 = fps

        rdt00, rdt01, rdt02 = r00 * dt, r01 * dt, r02 * dt
        rdt10, rdt11, rdt12 = r10 * dt, r11 * dt, r12 * dt
        rdt20, rdt21, rdt22 = r20 * dt, r21 * dt, r22 * dt

        for j in range(15):
            a0, a1, a2 = p0[j], p1[j], p2[j]
            v0, v1, v2 = p3[j], p4[j], p5[j]
            bg0, bg1, bg2 = p9[j], p10[j], p11[j]
            ba0, ba1, ba2 = p12[j], p13[j], p14[j]

            fp0[j] = f00 * a0 + f01 * a1 + f02 * a2 - dt * bg0
            fp1[j] = f10 * a0 + f11 * a1 + f12 * a2 - dt * bg1
            fp2[j] = f20 * a0 + f21 * a1 + f22 * a2 - dt * bg2

            fp3[j] = ft00 * a0 + ft01 * a1 + ft02 * a2 + gam * v0 - (rdt00 * ba0 + rdt01 * ba1 + rdt02 * ba2)
            fp4[j] = ft10 * a0 + ft11 * a1 + ft12 * a2 + gam * v1 - (rdt10 * ba0 + rdt11 * ba1 + rdt12 * ba2)
            fp5[j] = ft20 * a0 + ft21 * a1 + ft22 * a2 + gam * v2 - (rdt20 * ba0 + rdt21 * ba1 + rdt22 * ba2)

            fp6[j] = p6[j] + dt * v0
            fp7[j] = p7[j] + dt * v1
            fp8[j] = p8[j] + dt * v2

        fps = (fp0, fp1, fp2, fp3, fp4, fp5, fp6, fp7, fp8, p9, p10, p11, p12, p13, p14)

        for i in range(9):
            row = fps[i]
            out = P[i]
            c0, c1, c2 = row[0], row[1], row[2]
            cv0, cv1, cv2 = row[3], row[4], row[5]
            cbg0, cbg1, cbg2 = row[9], row[10], row[11]
            cba0, cba1, cba2 = row[12], row[13], row[14]

            out[0] = c0 * f00 + c1 * f01 + c2 * f02 - dt * cbg0
            out[1] = c0 * f10 + c1 * f11 + c2 * f12 - dt * cbg1
            out[2] = c0 * f20 + c1 * f21 + c2 * f22 - dt * cbg2

            out[3] = c0 * ft00 + c1 * ft01 + c2 * ft02 + gam * cv0 - (rdt00 * cba0 + rdt01 * cba1 + rdt02 * cba2)
            out[4] = c0 * ft10 + c1 * ft11 + c2 * ft12 + gam * cv1 - (rdt10 * cba0 + rdt11 * cba1 + rdt12 * cba2)
            out[5] = c0 * ft20 + c1 * ft21 + c2 * ft22 + gam * cv2 - (rdt20 * cba0 + rdt21 * cba1 + rdt22 * cba2)

            out[6] = row[6] + dt * cv0
            out[7] = row[7] + dt * cv1
            out[8] = row[8] + dt * cv2

            out[9], out[10], out[11] = cbg0, cbg1, cbg2
            out[12], out[13], out[14] = cba0, cba1, cba2

        for i in range(9, 15):
            out = P[i]
            for j in range(9):
                out[j] = P[j][i]

        for i in range(3):
            P[i][i] += q_th
            P[i + 3][i + 3] += q_v
            P[i + 9][i + 9] += q_bg
            P[i + 12][i + 12] += q_ba

        for i in range(15):
            row_i = P[i]
            for j in range(i + 1, 15):
                sym = 0.5 * (row_i[j] + P[j][i])
                row_i[j] = sym
                P[j][i] = sym
            if row_i[i] < 1e-12:
                row_i[i] = 1e-12
            elif row_i[i] > 1e6:
                sc = math.sqrt(1e6 / row_i[i])
                for j in range(15):
                    row_i[j] *= sc
                for j in range(15):
                    P[j][i] *= sc

    def _update_block3(
        self, blk: int, y0: float, y1: float, y2: float,
        h00: float, h01: float, h02: float,
        h10: float, h11: float, h12: float,
        h20: float, h21: float, h22: float,
        r0: float, r1: float, r2: float,
        gate_threshold: float = 1e9, clear_pv: bool = False,
        m_dim: int = 3, nu: float = 8.0
    ):
        P = self.P
        if clear_pv:
            for ii in range(6, 9):
                row_p = P[ii]
                row_p[3] = row_p[4] = row_p[5] = 0.0
                P[3][ii] = P[4][ii] = P[5][ii] = 0.0

        pb0 = P[blk]
        pb1 = P[blk + 1]
        pb2 = P[blk + 2]

        hp0 = self.hp0
        hp1 = self.hp1
        hp2 = self.hp2
        for j in range(15):
            v0, v1, v2 = pb0[j], pb1[j], pb2[j]
            hp0[j] = h00 * v0 + h01 * v1 + h02 * v2
            hp1[j] = h10 * v0 + h11 * v1 + h12 * v2
            hp2[j] = h20 * v0 + h21 * v1 + h22 * v2

        b0, b1, b2 = blk, blk + 1, blk + 2
        s00 = hp0[b0] * h00 + hp0[b1] * h01 + hp0[b2] * h02 + r0
        s01 = hp0[b0] * h10 + hp0[b1] * h11 + hp0[b2] * h12
        s02 = hp0[b0] * h20 + hp0[b1] * h21 + hp0[b2] * h22
        s10 = hp1[b0] * h00 + hp1[b1] * h01 + hp1[b2] * h02
        s11 = hp1[b0] * h10 + hp1[b1] * h11 + hp1[b2] * h12 + r1
        s12 = hp1[b0] * h20 + hp1[b1] * h21 + hp1[b2] * h22
        s20 = hp2[b0] * h00 + hp2[b1] * h01 + hp2[b2] * h02
        s21 = hp2[b0] * h10 + hp2[b1] * h11 + hp2[b2] * h12
        s22 = hp2[b0] * h20 + hp2[b1] * h21 + hp2[b2] * h22 + r2

        sinv = inv3x3(s00, s01, s02, s10, s11, s12, s20, s21, s22)
        if sinv is None:
            return
        si00, si01, si02, si10, si11, si12, si20, si21, si22 = sinv

        t0 = si00 * y0 + si01 * y1 + si02 * y2
        t1 = si10 * y0 + si11 * y1 + si12 * y2
        t2 = si20 * y0 + si21 * y1 + si22 * y2
        d2 = y0 * t0 + y1 * t1 + y2 * t2

        if not math.isfinite(d2):
            return

        if self.params.get("enable_student_t", False):
            if d2 > 1e6:
                return
            scale = (nu + d2) / (nu + float(m_dim))
            if scale < 1.0:
                scale = 1.0
            elif scale > 100.0:
                scale = 100.0
            if scale > 1.0:
                r0 *= scale
                r1 *= scale
                if m_dim == 3:
                    r2 *= scale
                s00 = hp0[b0] * h00 + hp0[b1] * h01 + hp0[b2] * h02 + r0
                s11 = hp1[b0] * h10 + hp1[b1] * h11 + hp1[b2] * h12 + r1
                s22 = hp2[b0] * h20 + hp2[b1] * h21 + hp2[b2] * h22 + r2
                sinv = inv3x3(s00, s01, s02, s10, s11, s12, s20, s21, s22)
                if sinv is None:
                    return
                si00, si01, si02, si10, si11, si12, si20, si21, si22 = sinv
        else:
            if d2 > gate_threshold:
                return

        k0 = self.k0
        k1 = self.k1
        k2 = self.k2
        dx = self.dx
        for i in range(15):
            a, b, c = hp0[i], hp1[i], hp2[i]
            ki0 = a * si00 + b * si10 + c * si20
            ki1 = a * si01 + b * si11 + c * si21
            ki2 = a * si02 + b * si12 + c * si22
            k0[i], k1[i], k2[i] = ki0, ki1, ki2
            dx[i] = ki0 * y0 + ki1 * y1 + ki2 * y2

        w0 = self.w0
        w1 = self.w1
        w2 = self.w2
        hb0, hb1, hb2 = hp0[b0], hp1[b0], hp2[b0]
        hc0, hc1, hc2 = hp0[b1], hp1[b1], hp2[b1]
        hd0, hd1, hd2 = hp0[b2], hp1[b2], hp2[b2]
        for i in range(15):
            row_i = P[i]
            ki0, ki1, ki2 = k0[i], k1[i], k2[i]
            ub0 = row_i[b0] - (ki0 * hb0 + ki1 * hb1 + ki2 * hb2)
            ub1 = row_i[b1] - (ki0 * hc0 + ki1 * hc1 + ki2 * hc2)
            ub2 = row_i[b2] - (ki0 * hd0 + ki1 * hd1 + ki2 * hd2)
            uh0 = ub0 * h00 + ub1 * h01 + ub2 * h02
            uh1 = ub0 * h10 + ub1 * h11 + ub2 * h12
            uh2 = ub0 * h20 + ub1 * h21 + ub2 * h22
            w0[i] = ki0 * r0 - uh0
            w1[i] = ki1 * r1 - uh1
            w2[i] = ki2 * r2 - uh2

        for i in range(15):
            row_i = P[i]
            ki0, ki1, ki2 = k0[i], k1[i], k2[i]
            wi0, wi1, wi2 = w0[i], w1[i], w2[i]
            for j in range(i, 15):
                val = (
                    row_i[j]
                    - (ki0 * hp0[j] + ki1 * hp1[j] + ki2 * hp2[j])
                    + (wi0 * k0[j] + wi1 * k1[j] + wi2 * k2[j])
                )
                row_i[j] = val
                P[j][i] = val

        dth0, dth1, dth2 = dx[0], dx[1], dx[2]
        self.q = quat_norm_align(quat_mul(self.q, rotvec_to_quat(dth0, dth1, dth2)), self.q)
        self.v[0] += dx[3]
        self.v[1] += dx[4]
        self.v[2] += dx[5]
        self.p[0] += dx[6]
        self.p[1] += dx[7]
        self.p[2] += dx[8]
        self.bg[0] += dx[9]
        self.bg[1] += dx[10]
        self.bg[2] += dx[11]
        self.ba[0] += dx[12]
        self.ba[1] += dx[13]
        self.ba[2] += dx[14]

        if abs(dth0) + abs(dth1) + abs(dth2) > 1e-12:
            g01, g02 = 0.5 * dth2, -0.5 * dth1
            g10, g12 = -0.5 * dth2, 0.5 * dth0
            g20, g21 = 0.5 * dth1, -0.5 * dth0
            p0, p1, p2 = P[0], P[1], P[2]
            for j in range(15):
                a0, a1, a2 = p0[j], p1[j], p2[j]
                p0[j] = a0 + g01 * a1 + g02 * a2
                p1[j] = g10 * a0 + a1 + g12 * a2
                p2[j] = g20 * a0 + g21 * a1 + a2
            for i in range(15):
                ri = P[i]
                a0, a1, a2 = ri[0], ri[1], ri[2]
                ri[0] = a0 + a1 * g01 + a2 * g02
                ri[1] = a0 * g10 + a1 + a2 * g12
                ri[2] = a0 * g20 + a1 * g21 + a2

        for i in range(15):
            if P[i][i] < 1e-12:
                P[i][i] = 1e-12


# ---------------------------------------------------------------------------
# Terminal 5s Blind Predictor
# ---------------------------------------------------------------------------

def make_blind_anchor(eskf: SparseESKF, params: dict):
    ring = eskf.ring
    if not ring:
        return (
            (eskf.p[0], eskf.p[1], eskf.p[2]),
            (eskf.v[0], eskf.v[1], eskf.v[2]),
            eskf.q,
            (0.0, 0.0, 0.0),
            eskf.get_w_walk(),
        )
    cnt = min(len(ring), max(1, int(round(params["blind_window_s"] * 200.0))))
    sub = list(ring)[-cnt:]
    last = sub[-1]
    pf = (last[0], last[1], last[2])
    qf = (last[6], last[7], last[8], last[9])
    inv_c = 1.0 / cnt
    vx = sum(s[3] for s in sub) * inv_c
    vy = sum(s[4] for s in sub) * inv_c
    vz = sum(s[5] for s in sub) * inv_c
    wx = sum(s[10] for s in sub) * inv_c
    wy = sum(s[11] for s in sub) * inv_c
    wz = sum(s[12] for s in sub) * inv_c
    w_walk = last[13]
    return pf, (vx, vy, vz), qf, (wx, wy, wz), w_walk


def eval_blind_pose(tau: float, anchor, params: dict):
    pf, vf, qf, wf, w_walk = anchor
    if tau <= 0.0:
        return pf, qf

    tau_v = params["tau_v"]
    shrink_v = params["shrink_v"]
    scale_s = shrink_v * tau_v * (1.0 - math.exp(-tau / tau_v))
    p_str = (pf[0] + vf[0] * scale_s, pf[1] + vf[1] * scale_s, pf[2] + vf[2] * scale_s)

    if not params["enable_regime_curvature"] or w_walk < params["regime_curvature_min_w"]:
        return p_str, qf

    r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(qf)
    vb0x = r00 * vf[0] + r10 * vf[1] + r20 * vf[2]
    vb0y = r01 * vf[0] + r11 * vf[1] + r21 * vf[2]
    vb0z = r02 * vf[0] + r12 * vf[1] + r22 * vf[2]

    tau_w = params["tau_omega_blind"]
    wx, wy, wz = wf
    n_seg = 50
    ds = tau / n_seg
    sx = sy = sz = 0.0
    q_end = qf

    for j in range(n_seg + 1):
        s = j * ds
        ang_scale = tau_w * (1.0 - math.exp(-s / tau_w))
        q_s = quat_mul(qf, rotvec_to_quat(wx * ang_scale, wy * ang_scale, wz * ang_scale))
        if j == n_seg:
            q_end = quat_norm_align(q_s, qf)
        rs00, rs01, rs02, rs10, rs11, rs12, rs20, rs21, rs22 = quat_to_rot(q_s)
        v_dec = shrink_v * math.exp(-s / tau_v)
        vwx = (rs00 * vb0x + rs01 * vb0y + rs02 * vb0z) * v_dec
        vwy = (rs10 * vb0x + rs11 * vb0y + rs12 * vb0z) * v_dec
        vwz = (rs20 * vb0x + rs21 * vb0y + rs22 * vb0z) * v_dec
        wt = 1.0 if (j == 0 or j == n_seg) else (4.0 if (j & 1) else 2.0)
        sx += wt * vwx
        sy += wt * vwy
        sz += wt * vwz

    ds3 = ds / 3.0
    p_curv = (pf[0] + sx * ds3, pf[1] + sy * ds3, pf[2] + sz * ds3)
    return (
        (1.0 - w_walk) * p_str[0] + w_walk * p_curv[0],
        (1.0 - w_walk) * p_str[1] + w_walk * p_curv[1],
        (1.0 - w_walk) * p_str[2] + w_walk * p_curv[2],
    ), q_end


# ---------------------------------------------------------------------------
# Pipeline Orchestrator (Full Stream & Formatting)
# ---------------------------------------------------------------------------

def run_pipeline(input_path: str, output_path: str, params: dict = None, ablation: str = "eskf_full", append_blind: bool = False):
    pr = dict(DEFAULT_PARAMS)
    if params is not None:
        pr.update(params)

    if ablation == "naive":
        pr["enable_rotor_drag"] = False
        pr["enable_pedestrian_prior"] = False
        pr["enable_velocity_bounding"] = False
        pr["enable_tilt_update"] = False
        pr["enable_adaptive_q"] = False
        pr["enable_spike_filter"] = False
        pr["enable_student_t"] = False
    elif ablation == "mahony":
        pr["enable_rotor_drag"] = False
        pr["enable_pedestrian_prior"] = False
        pr["enable_velocity_bounding"] = False
        pr["enable_adaptive_q"] = False
        pr["enable_spike_filter"] = False
    elif ablation == "eskf_basic":
        pr["enable_adaptive_q"] = False
        pr["enable_spike_filter"] = False
        pr["enable_rotor_drag"] = False
        pr["enable_pedestrian_prior"] = False
        pr["enable_velocity_bounding"] = False

    ts_strs, t_s, acc, gyr, ts_scale = load_imu_arrays(input_path)
    n = len(t_s)

    if ablation == "naive":
        # Pure naive dead-reckoning strapdown baseline (SPEC §11.5)
        out_p = np.zeros((n, 3), dtype=np.float64)
        out_q = np.zeros((n, 4), dtype=np.float64)
        init_mask = (t_s - t_s[0]) <= 1.0
        if np.sum(init_mask) < 2:
            init_mask[: min(n, 20)] = True
        a_mean = np.mean(acc[init_mask], axis=0)
        g_raw = float(np.linalg.norm(a_mean))
        g_mag = float(np.clip(g_raw, 9.0, 10.5))
        v_dir = a_mean / (g_raw if g_raw > 1e-6 else 1.0)
        cos_th = float(v_dir[2])
        if cos_th > 0.999999:
            q0 = (1.0, 0.0, 0.0, 0.0)
        elif cos_th < -0.999999:
            q0 = (0.0, 1.0, 0.0, 0.0)
        else:
            ax = v_dir[1]
            ay = -v_dir[0]
            an = math.sqrt(ax * ax + ay * ay)
            if an > 1e-9:
                ax /= an
                ay /= an
                ang = math.acos(max(-1.0, min(1.0, cos_th)))
                s = math.sin(0.5 * ang)
                q0 = (math.cos(0.5 * ang), ax * s, ay * s, 0.0)
            else:
                q0 = (1.0, 0.0, 0.0, 0.0)
        g_w = (0.0, 0.0, -g_mag)

        p = [0.0, 0.0, 0.0]
        v = [0.0, 0.0, 0.0]
        q = q0
        for k in range(n):
            if k > 0:
                dt = t_s[k] - t_s[k - 1]
            else:
                dt = 0.0
            ak = acc[k]
            wk = gyr[k]
            if 0.0 < dt < 10.0 and np.all(np.isfinite(ak)) and np.all(np.isfinite(wk)):
                dq = rotvec_to_quat(wk[0] * dt, wk[1] * dt, wk[2] * dt)
                q_new = quat_norm_align(quat_mul(q, dq), q)
                r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(q)
                awx = r00 * ak[0] + r01 * ak[1] + r02 * ak[2] + g_w[0]
                awy = r10 * ak[0] + r11 * ak[1] + r12 * ak[2] + g_w[1]
                awz = r20 * ak[0] + r21 * ak[1] + r22 * ak[2] + g_w[2]
                p[0] += v[0] * dt + 0.5 * awx * dt * dt
                p[1] += v[1] * dt + 0.5 * awy * dt * dt
                p[2] += v[2] * dt + 0.5 * awz * dt * dt
                v[0] += awx * dt
                v[1] += awy * dt
                v[2] += awz * dt
                q = q_new
            out_p[k] = p
            out_q[k] = q

        out_dir = os.path.dirname(os.path.abspath(output_path))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for k in range(n):
                ts_str = ts_strs[k]
                if ts_scale == 1e-9:
                    try:
                        t_int = int(ts_str)
                        sec = t_int // 1_000_000_000
                        nsec = t_int % 1_000_000_000
                        out_t = f"{sec}.{nsec:09d}"
                    except ValueError:
                        out_t = f"{float(ts_str) * 1e-9:.9f}"
                else:
                    out_t = f"{float(ts_str):.9f}"
                pos = out_p[k]
                quat = out_q[k]
                f.write(f"{out_t} {pos[0]:.9f} {pos[1]:.9f} {pos[2]:.9f} {quat[1]:.9f} {quat[2]:.9f} {quat[3]:.9f} {quat[0]:.9f}\n")
            if append_blind and n > 0:
                dt_b = 0.005
                dt_b_ns = 5_000_000
                n_blind = int(round(5.0 / dt_b))
                try:
                    last_t_ns = int(ts_strs[-1]) if ts_scale == 1e-9 else int(round(float(ts_strs[-1]) * 1e9))
                except Exception:
                    last_t_ns = 0
                for step in range(1, n_blind + 1):
                    tau = step * dt_b
                    pb = (p[0] + v[0] * tau, p[1] + v[1] * tau, p[2] + v[2] * tau)
                    if ts_scale == 1e-9 and last_t_ns > 0:
                        t_step_ns = last_t_ns + step * dt_b_ns
                        sec = t_step_ns // 1_000_000_000
                        nsec = t_step_ns % 1_000_000_000
                        out_t = f"{sec}.{nsec:09d}"
                    else:
                        out_t = f"{float(ts_strs[-1]) + tau:.9f}"
                    f.write(f"{out_t} {pb[0]:.9f} {pb[1]:.9f} {pb[2]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} {q[0]:.9f}\n")
        return
    acc_clean, gyr_clean, dup_mask, invalid_mask, fail_start, noise_mult, dt_nom = preprocess_offline(
        t_s, acc, gyr, pr
    )

    t0 = t_s[0]
    t_span = t_s - t0
    end_init_idx = int(np.searchsorted(t_span, pr["T_init"]))
    if end_init_idx < 2:
        end_init_idx = min(n, 200)

    eskf = SparseESKF(pr)
    eskf.initialize(t_s, acc_clean, gyr_clean, dup_mask, invalid_mask, end_init_idx)

    out_p = np.zeros((n, 3), dtype=np.float64)
    out_q = np.zeros((n, 4), dtype=np.float64)

    p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
    q_last = eskf.q

    in_gap = False
    gap_start_idx = -1
    p_gap_start = p_last
    prev_t = t_s[0]
    wx_s = wy_s = wz_s = 0.0
    last_valid_a = (float(acc_clean[0, 0]), float(acc_clean[0, 1]), float(acc_clean[0, 2]))
    last_valid_g = (float(gyr_clean[0, 0]), float(gyr_clean[0, 1]), float(gyr_clean[0, 2]))

    active_limit = min(n, fail_start)
    for k in range(active_limit):
        tk = t_s[k]
        if dup_mask[k]:
            out_p[k] = p_last
            out_q[k] = q_last
            continue

        if invalid_mask[k]:
            if not in_gap:
                in_gap = True
                gap_start_idx = k
                p_gap_start = p_last
            dt_step = max(1e-4, tk - prev_t)
            prev_t = tk
            eskf.propagate_gap(dt_step, dt_nom, wx_s, wy_s, wz_s)
            p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
            q_last = eskf.q
            out_p[k] = p_last
            out_q[k] = q_last
            continue

        # If returning from an invalid_mask segment, apply Hermite bridge
        if in_gap and gap_start_idx >= 0 and k > gap_start_idx and (k - gap_start_idx) < 200:
            p_rec = p_last
            delta_bridge = np.array(p_rec) - np.array(p_gap_start)
            g_len = k - gap_start_idx
            for b_i in range(gap_start_idx, k):
                alpha = float(b_i - gap_start_idx + 1) / float(g_len + 1)
                s_herm = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha
                out_p[b_i] += delta_bridge * s_herm
            in_gap = False
            gap_start_idx = -1

        dt = tk - prev_t
        prev_t = tk

        ax, ay, az = float(acc_clean[k, 0]), float(acc_clean[k, 1]), float(acc_clean[k, 2])
        gx, gy, gz = float(gyr_clean[k, 0]), float(gyr_clean[k, 1]), float(gyr_clean[k, 2])
        nm = float(noise_mult[k])
        t_el = tk - t0

        if dt > 2.5 * dt_nom:
            if dt <= 0.35:
                # degrade_imu.py short missing bursts (25 - 250 ms):
                # Linearly interpolate (acc, gyr) across virtual sub-steps
                n_virt = max(1, int(round(dt / dt_nom)))
                dt_v = dt / n_virt
                a0x, a0y, a0z = last_valid_a
                g0x, g0y, g0z = last_valid_g
                for step_i in range(1, n_virt):
                    alpha = float(step_i) / float(n_virt)
                    av_x = (1.0 - alpha) * a0x + alpha * ax
                    av_y = (1.0 - alpha) * a0y + alpha * ay
                    av_z = (1.0 - alpha) * a0z + alpha * az
                    gv_x = (1.0 - alpha) * g0x + alpha * gx
                    gv_y = (1.0 - alpha) * g0y + alpha * gy
                    gv_z = (1.0 - alpha) * g0z + alpha * gz
                    t_sub = (tk - dt) + step_i * dt_v - t0
                    eskf.step(dt_v, av_x, av_y, av_z, gv_x, gv_y, gv_z, nm, t_sub)
                eskf.step(dt_v, ax, ay, az, gx, gy, gz, nm, t_el)
            else:
                # Long blackout gap (> 0.35 s):
                eskf.propagate_gap(dt - dt_nom, dt_nom, wx_s, wy_s, wz_s)
                eskf.step(dt_nom, ax, ay, az, gx, gy, gz, nm, t_el)
        else:
            eskf.step(min(dt, 0.05), ax, ay, az, gx, gy, gz, nm, t_el)

        last_valid_a = (ax, ay, az)
        last_valid_g = (gx, gy, gz)

        wx_s = 0.95 * wx_s + 0.05 * (gx - eskf.bg[0])
        wy_s = 0.95 * wy_s + 0.05 * (gy - eskf.bg[1])
        wz_s = 0.95 * wz_s + 0.05 * (gz - eskf.bg[2])

        p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
        q_last = eskf.q
        out_p[k] = p_last
        out_q[k] = q_last

    # Process terminal blind segment if sensor failed
    if active_limit < n:
        anchor = make_blind_anchor(eskf, pr)
        t_fail_anchor = t_s[active_limit - 1] if active_limit > 0 else t_s[0]
        for k in range(active_limit, n):
            tau = max(0.0, t_s[k] - t_fail_anchor)
            pb, qb = eval_blind_pose(tau, anchor, pr)
            out_p[k] = pb
            out_q[k] = qb

    # Write output trajectory (TUM format: t tx ty tz qx qy qz qw)
    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for k in range(n):
            ts_str = ts_strs[k]
            if ts_scale == 1e-9:
                try:
                    t_int = int(ts_str)
                    sec = t_int // 1_000_000_000
                    nsec = t_int % 1_000_000_000
                    out_t = f"{sec}.{nsec:09d}"
                except ValueError:
                    out_t = f"{float(ts_str) * 1e-9:.9f}"
            else:
                out_t = f"{float(ts_str):.9f}"
            p = out_p[k]
            q = out_q[k]
            f.write(f"{out_t} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} {q[0]:.9f}\n")

        # Extrapolate 5.0 s if append_blind is requested
        if append_blind and n > 0:
            anchor = make_blind_anchor(eskf, pr)
            dt_b = dt_nom if dt_nom > 0 else 0.005
            dt_b_ns = max(1, int(round(dt_b * 1e9)))
            n_blind = int(round(5.0 / dt_b))
            try:
                last_t_ns = int(ts_strs[-1]) if ts_scale == 1e-9 else int(round(float(ts_strs[-1]) * 1e9))
            except Exception:
                last_t_ns = 0
            for step in range(1, n_blind + 1):
                tau = step * dt_b
                pb, qb = eval_blind_pose(tau, anchor, pr)
                if ts_scale == 1e-9 and last_t_ns > 0:
                    t_step_ns = last_t_ns + step * dt_b_ns
                    sec = t_step_ns // 1_000_000_000
                    nsec = t_step_ns % 1_000_000_000
                    out_t = f"{sec}.{nsec:09d}"
                else:
                    out_t = f"{float(ts_strs[-1]) + tau:.9f}"
                f.write(f"{out_t} {pb[0]:.9f} {pb[1]:.9f} {pb[2]:.9f} {qb[1]:.9f} {qb[2]:.9f} {qb[3]:.9f} {qb[0]:.9f}\n")


class _CustomParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        sys.exit(1)


def main():
    parser = _CustomParser(description="Standalone Single-File 6-DoF Inertial Estimator")
    parser.add_argument("input", nargs="?", default=None, help="Path to input IMU CSV, directory, or ZIP")
    parser.add_argument("output", nargs="?", default="estimated_trajectory.txt",
                        help="Path to output trajectory file (default: estimated_trajectory.txt)")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate 5.0 s blind window after stream")
    parser.add_argument(
        "--ablation",
        type=str,
        default="eskf_full",
        choices=["naive", "mahony", "eskf_basic", "eskf_full"],
        help="Ablation mode",
    )
    parser.add_argument("--params", type=str, default=None, help="Custom params JSON")
    args = parser.parse_args()

    if not args.input:
        parser.print_usage(sys.stderr)
        sys.exit(1)

    if not os.path.exists(args.input):
        sys.stderr.write(f"Input path not found: {args.input}\n")
        sys.exit(1)

    custom_params = None
    if args.params and os.path.exists(args.params):
        import json
        with open(args.params, "r", encoding="utf-8") as f:
            custom_params = json.load(f)

    run_pipeline(args.input, args.output, params=custom_params, ablation=args.ablation, append_blind=args.append_blind)


if __name__ == "__main__":
    main()
