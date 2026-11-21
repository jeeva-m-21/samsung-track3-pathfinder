#!/usr/bin/env python3
"""
main.py - Standalone Single-File 6-DoF Inertial Trajectory Estimator
Samsung XR Hackathon Track 3: Path Finder (VITC_SingleThread)

Self-contained, zero-NumPy, pure-Python stdlib pipeline:
  1. Streaming CSV/Dir/Zip Ingestion (header, accel/gyro order, s/ms/us/ns, g/ms2, N_out == N_in)
  2. Offline Pre-Cleaner (running-max dup mask, stuck-sensor interp, two-tier spike sniping,
     pre-median raw MAD noise tracking, gyro median-3, noise-adaptive zero-phase denoising,
     trailing failure mask)
  3. Clean Startup + Gyro Back-Integration + Static-Window Mining + O(1) Bias-Invariant ZUPT/ZARU
  4. Fast Block-Sparse 15-State Student-t ESKF (analytical F_vtheta, 1D/2D/3D fused symmetric P - KHP updates,
     quadrature noise-compensated regime classification, physical-floor degradation monitor)
  5. Posture-Invariant Leveled Walking NHC + Weinberg Stride Envelope + Decaying-Omega Gap Bridge
  6. O(1) Incremental Terminal Blind Predictor (exponentially weighted anchor, curved/straight decay)
"""

import argparse
from array import array
from collections import deque
import io
import math
import os
import sys
import zipfile

G_STD = 9.80665

DEFAULT_PARAMS = {
    # Startup & initialization (exploits degrade_imu.py protect_seconds = 2.0 s)
    "T_init": 1.8,
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
    "enable_static_window_mining": True,

    # Process noise (aligned with degrade_imu.py noise & bias RW)
    "sigma_g_base": 0.015,
    "sigma_a_base": 0.080,
    "sigma_bg_base": 0.0001,
    "sigma_ba_base": 0.0010,
    "enable_sev_bias_rw": True,
    "enable_inekf_transition": True,

    # Adaptive Q, Spike Filter & Noise-Adaptive Denoising
    "enable_adaptive_q": True,
    "adaptive_q_alpha": 0.005,
    "adaptive_q_ref": 0.0033119250641368016,
    "adaptive_q_max": 25.0,
    "enable_spike_filter": True,
    "enable_adaptive_denoise": True,
    "denoise_nm_start": 2.2,
    "denoise_nm_span": 3.0,

    # Regime OU damping (decoupled flight vs walking)
    "tau_ou": 1.946,
    "tau_ou_flight": 1.753,
    "tau_ou_walk": 0.95,
    "flight_vibe_thresh": 0.40,
    "ped_energy_thresh": 0.20,
    "enable_wiener_ou": True,
    "wiener_ou_beta": 0.15,

    # Aerial aerodynamic rotor drag prior (disabled by default per EXP-PY15)
    "enable_rotor_drag": False,
    "rotor_drag_kd": 0.140,
    "rotor_drag_sigma": 0.550,
    "rotor_drag_gate": 12.0,

    # Flight vertical velocity prior
    "enable_flight_vz_prior": True,
    "enable_centripetal_comp": True,
    "flight_vz_sigma": 1.20,

    # Handheld / Pedestrian NHC + Posture-Invariant Frame + Weinberg Stride Speed
    "enable_pedestrian_prior": True,
    "enable_leveled_ped_nhc": True,
    "ped_sigma_lat": 0.123,
    "ped_sigma_vert": 0.429,
    "ped_gate_threshold": 9.0,
    "ped_max_velocity_mps": 2.64,
    "enable_weinberg_bound": True,
    "weinberg_k": 0.724,
    "weinberg_sigma": 0.255,

    # Velocity bounding
    "enable_velocity_bounding": True,
    "max_velocity_mps": 3.50,
    "velocity_bounding_sigma": 0.140,
    "vel_bound_sigma": 0.140,

    # Gravity tilt alignment
    "enable_tilt_jam_fix": True,
    "tilt_tau_a": 0.295,
    "tilt_omega_max": 2.2,
    "tilt_sigma": 0.050,
    "sigma_tilt": 0.025,
    "tilt_alpha_a": 2.0,
    "tilt_a_scale": 0.21426320932326784,
    "tilt_alpha_omega": 8.0,
    "tilt_omega_scale": 2.2,

    # Bias-invariant ZUPT + ZARU
    "zupt_window_s": 0.25,
    "zupt_gyro_mean": 0.02,
    "zupt_gyro_var": 0.0004,
    "zupt_acc_dev": 0.18,
    "zupt_sigma": 0.0143,
    "sigma_zupt": 0.0143,
    "enable_zaru": True,
    "zaru_sigma": 0.005,
    "zupt_min_quality": 0.30,

    # Student-t Variational Scaling
    "enable_student_t": True,
    "nu_tilt": 4.0,
    "nu_vel": 6.0,

    # Monitor & Gap / Burst recovery
    "monitor_alpha": 0.01,
    "kappa_max": 50.0,
    "monitor_var_acc_floor": 0.0,
    "monitor_var_gyr_floor": 0.0,
    "tau_gap": 2.0,
    "tau_w_gap": 0.40,
    "kappa_gap": 5.0,
    "recover_steps": 100,

    # Terminal 5s blind prediction
    "ring_buffer_size": 256,
    "tau_v": 0.803,
    "shrink_v": 0.914,
    "blind_walk_vz_scale": 0.70,
    "enable_regime_curvature": True,
    "regime_curvature_min_w": 0.50,
    "tau_omega_blind": 0.411,
    "blind_window_s": 0.585,
    "blind_yaw_only_walk": False,
}


# ---------------------------------------------------------------------------
# Fast Math Primitives (Pure Python Scalars)
# ---------------------------------------------------------------------------

def rotvec_to_quat(rx: float, ry: float, rz: float):
    th2 = rx * rx + ry * ry + rz * rz
    if th2 < 1e-4:
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


def _median_list(vals):
    m = len(vals)
    if m == 0:
        return 0.0
    s = sorted(vals)
    mid = m >> 1
    if m & 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


# ---------------------------------------------------------------------------
# Universal Streaming Ingestion & Column/Unit/Timestamp Auto-Detection
# ---------------------------------------------------------------------------

def _iter_imu_lines(input_path: str):
    """Yield lines lazily from a CSV file, directory, or ZIP archive (O(1) RAM)."""
    if os.path.isdir(input_path):
        candidates = (
            os.path.join(input_path, "imu0", "data.csv"),
            os.path.join(input_path, "mav0", "imu0", "data.csv"),
            os.path.join(input_path, "data.csv"),
        )
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
            for line in f:
                yield line
        return

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
            with zf.open(target, "r") as raw_f:
                with io.TextIOWrapper(raw_f, encoding="utf-8-sig") as txt_f:
                    for line in txt_f:
                        yield line
        return

    with open(input_path, "r", encoding="utf-8-sig") as f:
        for line in f:
            yield line


def _open_imu_lines(input_path: str):
    """Compatibility helper returning lines list."""
    return [line.rstrip("\r\n") for line in _iter_imu_lines(input_path)]


def _detect_gyro_first_by_norm(c1x, c1y, c1z, c2x, c2y, c2z, parse_inv) -> bool:
    n = len(c1x)
    step = max(1, n // 2000)
    v1 = []
    v2 = []
    isfinite = math.isfinite
    sqrt = math.sqrt
    for k in range(0, n, step):
        if parse_inv[k]:
            continue
        x1, y1, z1 = c1x[k], c1y[k], c1z[k]
        x2, y2, z2 = c2x[k], c2y[k], c2z[k]
        if not (isfinite(x1) and isfinite(y1) and isfinite(z1) and isfinite(x2) and isfinite(y2) and isfinite(z2)):
            continue
        n1 = sqrt(x1 * x1 + y1 * y1 + z1 * z1)
        n2 = sqrt(x2 * x2 + y2 * y2 + z2 * z2)
        if n1 > 1e-5:
            v1.append(n1)
        if n2 > 1e-5:
            v2.append(n2)
    if not v1 and not v2:
        return False
    m1 = _median_list(v1)
    m2 = _median_list(v2)
    d1 = min(abs(m1 - G_STD), abs(m1 - 1.0) * G_STD)
    d2 = min(abs(m2 - G_STD), abs(m2 - 1.0) * G_STD)
    return d2 < d1


def load_imu_arrays(input_path: str):
    """
    Stream-parse IMU telemetry into compact typed arrays (zero NumPy).
    Returns: (ts_list, is_int_ts, ts_scale, t_s, acc_tuple, gyr_tuple, parse_inv)
    where acc_tuple = (ax, ay, az) and gyr_tuple = (gx, gy, gz) are array('f').
    """
    header_cols = None
    data_started = False
    is_int_ts = True
    ts_int = array("q")
    ts_obj = None  # fallback only if float or int64 overflow
    c1x = array("f")
    c1y = array("f")
    c1z = array("f")
    c2x = array("f")
    c2y = array("f")
    c2z = array("f")
    parse_inv = bytearray()

    last_ts_val = 0
    isfinite = math.isfinite
    INT64_MIN = -9_223_372_036_854_775_808
    INT64_MAX = 9_223_372_036_854_775_807

    def _push_ts(val):
        nonlocal is_int_ts, ts_obj
        if ts_obj is not None:
            ts_obj.append(val)
            return
        if is_int_ts and isinstance(val, int) and INT64_MIN <= val <= INT64_MAX:
            ts_int.append(val)
        elif not is_int_ts or isinstance(val, float):
            is_int_ts = False
            ts_obj = array("d", ts_int)
            del ts_int[:]
            ts_obj.append(float(val))
        else:
            ts_obj = list(ts_int)
            del ts_int[:]
            ts_obj.append(val)

    for raw_line in _iter_imu_lines(input_path):
        s = raw_line.strip()
        if not s:
            continue
        if not data_started:
            parts = [p.strip() for p in s.split(",")]
            try:
                float(parts[0])
                data_started = True
            except ValueError:
                if s.startswith("#") and len(parts) < 7:
                    continue
                header_cols = parts
                data_started = True
                continue
        else:
            if s.startswith("#"):
                continue
            parts = s.split(",")

        ts_tok = parts[0].strip()
        try:
            if "." in ts_tok or "e" in ts_tok or "E" in ts_tok:
                t_val = float(ts_tok)
                if not isfinite(t_val):
                    raise ValueError("non-finite ts")
                is_int_ts = False
            else:
                t_val = int(ts_tok)
            last_ts_val = t_val
        except ValueError:
            _push_ts(last_ts_val)
            c1x.append(0.0); c1y.append(0.0); c1z.append(0.0)
            c2x.append(0.0); c2y.append(0.0); c2z.append(0.0)
            parse_inv.append(1)
            continue

        if len(parts) < 7:
            _push_ts(t_val)
            c1x.append(0.0); c1y.append(0.0); c1z.append(0.0)
            c2x.append(0.0); c2y.append(0.0); c2z.append(0.0)
            parse_inv.append(1)
            continue

        try:
            v1 = float(parts[1])
            v2 = float(parts[2])
            v3 = float(parts[3])
            v4 = float(parts[4])
            v5 = float(parts[5])
            v6 = float(parts[6])
            _push_ts(t_val)
            c1x.append(v1); c1y.append(v2); c1z.append(v3)
            c2x.append(v4); c2y.append(v5); c2z.append(v6)
            parse_inv.append(0)
        except ValueError:
            _push_ts(t_val)
            c1x.append(0.0); c1y.append(0.0); c1z.append(0.0)
            c2x.append(0.0); c2y.append(0.0); c2z.append(0.0)
            parse_inv.append(1)

    ts_list = ts_obj if ts_obj is not None else ts_int
    n = len(ts_list)
    if n == 0:
        raise ValueError("No valid IMU rows parsed")

    # Determine timestamp scale cleanly without float64 precision loss
    t_s = array("d", [0.0]) * n
    if is_int_ts:
        max_abs = 0
        sample_diffs = []
        lim_d = min(n, 500)
        for k in range(n):
            av = abs(ts_list[k])
            if av > max_abs:
                max_abs = av
            if 0 < k < lim_d:
                d = ts_list[k] - ts_list[k - 1]
                if d > 0:
                    sample_diffs.append(d)
        med_d = _median_list(sample_diffs) if sample_diffs else 0
        h0_lower = header_cols[0].lower() if header_cols else ""
        if "[ns]" in h0_lower:
            ts_scale = 1e-9
        elif "[us]" in h0_lower:
            ts_scale = 1e-6
        elif "[ms]" in h0_lower:
            ts_scale = 1e-3
        elif "[s]" in h0_lower or "(s)" in h0_lower or "_s" in h0_lower:
            ts_scale = 1.0
        elif med_d >= 1_000_000:
            ts_scale = 1e-9
        elif med_d >= 1_000:
            ts_scale = 1e-6
        elif med_d >= 1:
            ts_scale = 1e-3
        elif med_d == 0 and max_abs >= 100_000_000_000:
            ts_scale = 1e-9
        elif med_d == 0 and max_abs >= 100_000_000:
            ts_scale = 1e-6
        elif med_d == 0 and max_abs >= 100_000:
            ts_scale = 1e-3
        else:
            ts_scale = 1.0
        t0_raw = ts_list[0]
        for k in range(n):
            t_s[k] = (ts_list[k] - t0_raw) * ts_scale
    else:
        pos_diffs = []
        lim_d = min(n, 1000)
        for k in range(1, lim_d):
            d = float(ts_list[k]) - float(ts_list[k - 1])
            if d > 0.0:
                pos_diffs.append(d)
        med_dt = _median_list(pos_diffs) if pos_diffs else 0.005
        if med_dt < 1.0:
            ts_scale = 1.0
        elif med_dt < 1e3:
            ts_scale = 1e-3
        elif med_dt < 1e6:
            ts_scale = 1e-6
        else:
            ts_scale = 1e-9
        t0_f = float(ts_list[0])
        for k in range(n):
            t_s[k] = (float(ts_list[k]) - t0_f) * ts_scale

    gyro_first = False
    if header_cols is not None and len(header_cols) >= 7:
        h1 = header_cols[1].lower()
        h4 = header_cols[4].lower()
        if ("w_" in h1 or "gyr" in h1 or "gx" in h1) and ("a_" in h4 or "acc" in h4 or "ax" in h4):
            gyro_first = True
        elif ("a_" in h1 or "acc" in h1 or "ax" in h1) and ("w_" in h4 or "gyr" in h4 or "gx" in h4):
            gyro_first = False
        else:
            gyro_first = _detect_gyro_first_by_norm(c1x, c1y, c1z, c2x, c2y, c2z, parse_inv)
    else:
        gyro_first = _detect_gyro_first_by_norm(c1x, c1y, c1z, c2x, c2y, c2z, parse_inv)

    if gyro_first:
        gx, gy, gz = c1x, c1y, c1z
        ax, ay, az = c2x, c2y, c2z
    else:
        ax, ay, az = c1x, c1y, c1z
        gx, gy, gz = c2x, c2y, c2z

    # Detect g-units vs m/s^2 on accelerometer
    step_n = max(1, n // 2000)
    an_samp = []
    sqrt = math.sqrt
    for k in range(0, n, step_n):
        if parse_inv[k]:
            continue
        x, y, z = ax[k], ay[k], az[k]
        if isfinite(x) and isfinite(y) and isfinite(z):
            an = sqrt(x * x + y * y + z * z)
            if an > 0.1:
                an_samp.append(an)
    if an_samp:
        med_norm = _median_list(an_samp)
        if 0.5 < med_norm < 2.0:
            for k in range(n):
                if isfinite(ax[k]):
                    ax[k] *= G_STD
                if isfinite(ay[k]):
                    ay[k] *= G_STD
                if isfinite(az[k]):
                    az[k] *= G_STD

    return ts_list, is_int_ts, ts_scale, t_s, (ax, ay, az), (gx, gy, gz), parse_inv


# ---------------------------------------------------------------------------
# Offline Pre-Cleaner for degrade_imu.py Hazards
# ---------------------------------------------------------------------------

def _unpack_vec3_arrays(arr_or_tuple):
    """Accept either (x, y, z) tuple of array('f'/'d') or 2D list/array of rows."""
    if isinstance(arr_or_tuple, tuple) and len(arr_or_tuple) == 3 and not isinstance(arr_or_tuple[0], (int, float)):
        return arr_or_tuple[0], arr_or_tuple[1], arr_or_tuple[2]
    n = len(arr_or_tuple)
    xs = array("f", [0.0]) * n
    ys = array("f", [0.0]) * n
    zs = array("f", [0.0]) * n
    for k in range(n):
        r = arr_or_tuple[k]
        xs[k] = float(r[0])
        ys[k] = float(r[1])
        zs[k] = float(r[2])
    return xs, ys, zs


def preprocess_offline(t_s, acc, gyr, parse_inv, params: dict):
    n = len(t_s)
    ax, ay, az = _unpack_vec3_arrays(acc)
    gx, gy, gz = _unpack_vec3_arrays(gyr)

    dup_mask = bytearray(n)
    invalid_mask = bytearray(n)

    isfinite = math.isfinite
    max_t = t_s[0] if n > 0 else 0.0

    for k in range(n):
        if k > 0:
            tk = t_s[k]
            if tk <= max_t:
                dup_mask[k] = 1
            else:
                max_t = tk

        x1, y1, z1 = ax[k], ay[k], az[k]
        x2, y2, z2 = gx[k], gy[k], gz[k]
        fin = (
            (not parse_inv[k])
            and isfinite(x1) and isfinite(y1) and isfinite(z1)
            and isfinite(x2) and isfinite(y2) and isfinite(z2)
        )
        if not fin:
            ax[k] = 0.0; ay[k] = 0.0; az[k] = 0.0
            gx[k] = 0.0; gy[k] = 0.0; gz[k] = 0.0
            invalid_mask[k] = 2  # 2 = zero/nonfinite candidate for terminal failure
            continue

        ma = max(abs(x1), abs(y1), abs(z1))
        mg = max(abs(x2), abs(y2), abs(z2))
        if ma < 1e-6 and mg < 1e-6:
            invalid_mask[k] = 2  # 2 = zero candidate for terminal failure
        elif ma > 160.0 or mg > 35.0:
            invalid_mask[k] = 1

    # Trailing [0..0] or non-finite terminal 5s failure block
    fail_start = n
    for k in range(n - 1, -1, -1):
        if invalid_mask[k] == 2:
            fail_start = k
        else:
            break

    # Defensive check: trailing bit-exact stuck sensor block >= 1.5 s at EOF
    if fail_start == n and n >= 250:
        lx, ly, lz = ax[n - 1], ay[n - 1], az[n - 1]
        lgx, lgy, lgz = gx[n - 1], gy[n - 1], gz[n - 1]
        stuck_k = n - 1
        while stuck_k > 0:
            p = stuck_k - 1
            if (
                abs(ax[p] - lx) < 1e-6 and abs(ay[p] - ly) < 1e-6 and abs(az[p] - lz) < 1e-6
                and abs(gx[p] - lgx) < 1e-6 and abs(gy[p] - lgy) < 1e-6 and abs(gz[p] - lgz) < 1e-6
            ):
                stuck_k = p
            else:
                break
        if (n - stuck_k) >= 200 and (t_s[n - 1] - t_s[stuck_k]) >= 1.5:
            fail_start = stuck_k + 1

    active_idx = array("I")
    for k in range(fail_start):
        if (not dup_mask[k]) and (not invalid_mask[k]):
            active_idx.append(k)
    m = len(active_idx)

    # 1. Bit-exact stuck-sensor run detection & linear interpolation
    if m >= 3:
        i = 0
        stuck_invalidated = False
        while i < m - 1:
            ki = active_idx[i]
            kn = active_idx[i + 1]
            same_joint = (
                abs(ax[kn] - ax[ki]) < 1e-6 and abs(ay[kn] - ay[ki]) < 1e-6 and abs(az[kn] - az[ki]) < 1e-6
                and abs(gx[kn] - gx[ki]) < 1e-6 and abs(gy[kn] - gy[ki]) < 1e-6 and abs(gz[kn] - gz[ki]) < 1e-6
            )
            if same_joint:
                j = i + 1
                while j < m - 1:
                    k1 = active_idx[j]
                    k2 = active_idx[j + 1]
                    if (
                        abs(ax[k2] - ax[k1]) < 1e-6 and abs(ay[k2] - ay[k1]) < 1e-6 and abs(az[k2] - az[k1]) < 1e-6
                        and abs(gx[k2] - gx[k1]) < 1e-6 and abs(gy[k2] - gy[k1]) < 1e-6 and abs(gz[k2] - gz[k1]) < 1e-6
                    ):
                        j += 1
                    else:
                        break
                run_len = j - i
                if (j + 1) < m and run_len <= 20:
                    k_end = active_idx[j + 1]
                    dt_span = t_s[k_end] - t_s[ki]
                    if 1e-9 < dt_span < 0.25:
                        a0x, a0y, a0z = ax[ki], ay[ki], az[ki]
                        a1x, a1y, a1z = ax[k_end], ay[k_end], az[k_end]
                        g0x, g0y, g0z = gx[ki], gy[ki], gz[ki]
                        g1x, g1y, g1z = gx[k_end], gy[k_end], gz[k_end]
                        inv_span = 1.0 / dt_span
                        t_start = t_s[ki]
                        for p_k in range(i + 1, j + 1):
                            kp = active_idx[p_k]
                            al = (t_s[kp] - t_start) * inv_span
                            om = 1.0 - al
                            ax[kp] = om * a0x + al * a1x
                            ay[kp] = om * a0y + al * a1y
                            az[kp] = om * a0z + al * a1z
                            gx[kp] = om * g0x + al * g1x
                            gy[kp] = om * g0y + al * g1y
                            gz[kp] = om * g0z + al * g1z
                    else:
                        for p_k in range(i + 1, j + 1):
                            invalid_mask[active_idx[p_k]] = 1
                        stuck_invalidated = True
                else:
                    for p_k in range(i + 1, j + 1):
                        invalid_mask[active_idx[p_k]] = 1
                    stuck_invalidated = True
                i = j + 1
            else:
                i += 1

        if stuck_invalidated:
            active_idx = array("I", [k for k in active_idx if not invalid_mask[k]])
            m = len(active_idx)

    # 2. Two-tier spike sniping (squared-norm fast rejection path)
    if m >= 5:
        sqrt = math.sqrt
        # Subsample ~2000 rows to estimate MAD cleanly without 116k sqrt calls
        step_s = max(1, (m - 2) // 2000)
        da_samp = []
        dg_samp = []
        for idx_k in range(1, m - 1, step_s):
            kp = active_idx[idx_k - 1]
            kc = active_idx[idx_k]
            kn = active_idx[idx_k + 1]
            dax = ax[kc] - 0.5 * (ax[kp] + ax[kn])
            day = ay[kc] - 0.5 * (ay[kp] + ay[kn])
            daz = az[kc] - 0.5 * (az[kp] + az[kn])
            dgx = gx[kc] - 0.5 * (gx[kp] + gx[kn])
            dgy = gy[kc] - 0.5 * (gy[kp] + gy[kn])
            dgz = gz[kc] - 0.5 * (gz[kp] + gz[kn])
            da_samp.append(sqrt(dax * dax + day * day + daz * daz))
            dg_samp.append(sqrt(dgx * dgx + dgy * dgy + dgz * dgz))

        mad_a = max(0.05, _median_list(da_samp))
        mad_g = max(0.005, _median_list(dg_samp))
        th_a = max(1.8, 6.0 * mad_a)
        th_a2 = th_a * th_a
        th_g = max(0.22, 6.0 * mad_g)
        th_g2 = th_g * th_g
        th_a_iso = max(15.0, 18.0 * mad_a)

        spike_list = []
        iso_a_list = []
        for idx_k in range(1, m - 1):
            kp = active_idx[idx_k - 1]
            kc = active_idx[idx_k]
            kn = active_idx[idx_k + 1]
            dax = ax[kc] - 0.5 * (ax[kp] + ax[kn])
            day = ay[kc] - 0.5 * (ay[kp] + ay[kn])
            daz = az[kc] - 0.5 * (az[kp] + az[kn])
            da2 = dax * dax + day * day + daz * daz
            if da2 <= th_a2:
                continue  # skips >99.5% of samples in one float compare
            da = sqrt(da2)
            nax = ax[kn] - ax[kp]; nay = ay[kn] - ay[kp]; naz = az[kn] - az[kp]
            dan = sqrt(nax * nax + nay * nay + naz * naz)
            if da <= 1.8 * dan + 0.5:
                continue
            dgx = gx[kc] - 0.5 * (gx[kp] + gx[kn])
            dgy = gy[kc] - 0.5 * (gy[kp] + gy[kn])
            dgz = gz[kc] - 0.5 * (gz[kp] + gz[kn])
            dg2 = dgx * dgx + dgy * dgy + dgz * dgz
            if dg2 > th_g2:
                dg = sqrt(dg2)
                ngx = gx[kn] - gx[kp]; ngy = gy[kn] - gy[kp]; ngz = gz[kn] - gz[kp]
                dgn = sqrt(ngx * ngx + ngy * ngy + ngz * ngz)
                if dg > 1.8 * dgn + 0.08:
                    spike_list.append(idx_k)
                    continue
            if da > th_a_iso and da > 4.5 * dan + 1.5:
                iso_a_list.append(idx_k)

        for idx_k in spike_list:
            kp = active_idx[idx_k - 1]
            kc = active_idx[idx_k]
            kn = active_idx[idx_k + 1]
            ax[kc] = 0.5 * (ax[kp] + ax[kn])
            ay[kc] = 0.5 * (ay[kp] + ay[kn])
            az[kc] = 0.5 * (az[kp] + az[kn])
            gx[kc] = 0.5 * (gx[kp] + gx[kn])
            gy[kc] = 0.5 * (gy[kp] + gy[kn])
            gz[kc] = 0.5 * (gz[kp] + gz[kn])

        for idx_k in iso_a_list:
            kp = active_idx[idx_k - 1]
            kc = active_idx[idx_k]
            kn = active_idx[idx_k + 1]
            ax[kc] = 0.5 * (ax[kp] + ax[kn])
            ay[kc] = 0.5 * (ay[kp] + ay[kn])
            az[kc] = 0.5 * (az[kp] + az[kn])

    # 3. Pre-median raw gyro second-difference MAD noise tracker
    noise_mult = array("f", [1.0]) * n
    if params["enable_adaptive_q"] and m >= 4:
        c_mad = math.sqrt(math.pi / 12.0) * 1.4364
        t_init_limit = params.get("T_init", 1.8)
        t0_active = t_s[active_idx[0]]
        n_init_samples = 0
        while n_init_samples < m - 2 and (t_s[active_idx[n_init_samples]] - t0_active) < t_init_limit:
            n_init_samples += 1
        n_init_samples = max(4, n_init_samples)
        step_init = max(1, n_init_samples // 100)
        d_bars_init = []
        for idx_k in range(0, n_init_samples, step_init):
            ka = active_idx[idx_k]
            kb = active_idx[idx_k + 1]
            kc = active_idx[idx_k + 2]
            d_bars_init.append((
                abs(gx[kc] - 2.0 * gx[kb] + gx[ka])
                + abs(gy[kc] - 2.0 * gy[kb] + gy[ka])
                + abs(gz[kc] - 2.0 * gz[kb] + gz[ka])
            ) * (1.0 / 3.0))
            ka = active_idx[idx_k]
            kb = active_idx[idx_k + 1]
            kc = active_idx[idx_k + 2]
            d_bars_init.append((
                abs(gx[kc] - 2.0 * gx[kb] + gx[ka])
                + abs(gy[kc] - 2.0 * gy[kb] + gy[ka])
                + abs(gz[kc] - 2.0 * gz[kb] + gz[ka])
            ) * (1.0 / 3.0))
        ref_g_clean = _median_list(d_bars_init) * c_mad if d_bars_init else params["adaptive_q_ref"]
        ref_g = max(params["adaptive_q_ref"], ref_g_clean)

        alpha_q = params["adaptive_q_alpha"]
        one_m_aq = 1.0 - alpha_q
        inv_ref_g = 1.0 / ref_g
        max_q = params["adaptive_q_max"]
        c_mad_inv_ref = c_mad * inv_ref_g

        k0 = active_idx[0]
        k1 = active_idx[1]
        k2 = active_idx[2]
        d0 = (
            abs(gx[k2] - 2.0 * gx[k1] + gx[k0])
            + abs(gy[k2] - 2.0 * gy[k1] + gy[k0])
            + abs(gz[k2] - 2.0 * gz[k1] + gz[k0])
        ) / 3.0
        ewma_d = 0.25 if d0 > 0.25 else d0

        first_kq = 1.0
        for idx_k in range(m - 2):
            ka = active_idx[idx_k]
            kb = active_idx[idx_k + 1]
            kc = active_idx[idx_k + 2]
            d_bar = (
                abs(gx[kc] - 2.0 * gx[kb] + gx[ka])
                + abs(gy[kc] - 2.0 * gy[kb] + gy[ka])
                + abs(gz[kc] - 2.0 * gz[kb] + gz[ka])
            ) * (1.0 / 3.0)
            if d_bar > 0.25:
                d_bar = 0.25
            ewma_d = one_m_aq * ewma_d + alpha_q * d_bar
            kq = ewma_d * c_mad_inv_ref
            if kq < 1.0:
                kq = 1.0
            elif kq > max_q:
                kq = max_q
            noise_mult[kc] = kq
            if idx_k == 0:
                first_kq = kq
        noise_mult[k0] = first_kq
        noise_mult[k1] = first_kq

    # 4. 3-tap median filter on gyro only (in-place via scalar temporaries)
    if params["enable_spike_filter"] and m >= 3:
        k0 = active_idx[0]
        px, py, pz = gx[k0], gy[k0], gz[k0]
        for idx_k in range(1, m - 1):
            kp = active_idx[idx_k - 1]
            kc = active_idx[idx_k]
            kn = active_idx[idx_k + 1]
            cx, cy, cz = gx[kc], gy[kc], gz[kc]
            if (t_s[kc] - t_s[kp] <= 0.05) and (t_s[kn] - t_s[kc] <= 0.05):
                gx[kc] = median3(px, cx, gx[kn])
                gy[kc] = median3(py, cy, gy[kn])
                gz[kc] = median3(pz, cz, gz[kn])
            px, py, pz = cx, cy, cz

    # 5. Noise-adaptive zero-phase 3-tap binomial denoising [0.25, 0.5, 0.25] (in-place)
    #    Strictly inactive on clean/low-noise data (noise_mult <= denoise_nm_start).
    if params.get("enable_adaptive_denoise", True) and m >= 3:
        nm_start = params.get("denoise_nm_start", 2.2)
        inv_span = 1.0 / max(0.1, params.get("denoise_nm_span", 3.0))
        has_high_noise = False
        for idx_k in range(1, m - 1):
            if noise_mult[active_idx[idx_k]] > nm_start:
                has_high_noise = True
                break
        if has_high_noise:
            k0 = active_idx[0]
            pax, pay, paz = ax[k0], ay[k0], az[k0]
            pgx, pgy, pgz = gx[k0], gy[k0], gz[k0]
            for idx_k in range(1, m - 1):
                kc = active_idx[idx_k]
                cax, cay, caz = ax[kc], ay[kc], az[kc]
                cgx, cgy, cgz = gx[kc], gy[kc], gz[kc]
                nm_k = noise_mult[kc]
                if nm_k > nm_start:
                    kp = active_idx[idx_k - 1]
                    kn = active_idx[idx_k + 1]
                    if (t_s[kc] - t_s[kp] <= 0.05) and (t_s[kn] - t_s[kc] <= 0.05):
                        al = (nm_k - nm_start) * inv_span
                        if al > 1.0:
                            al = 1.0
                        wa = 0.75 * al
                        wg = 0.45 * al
                        oma = 1.0 - wa
                        omg = 1.0 - wg
                        ax[kc] = oma * cax + wa * (0.25 * pax + 0.5 * cax + 0.25 * ax[kn])
                        ay[kc] = oma * cay + wa * (0.25 * pay + 0.5 * cay + 0.25 * ay[kn])
                        az[kc] = oma * caz + wa * (0.25 * paz + 0.5 * caz + 0.25 * az[kn])
                        gx[kc] = omg * cgx + wg * (0.25 * pgx + 0.5 * cgx + 0.25 * gx[kn])
                        gy[kc] = omg * cgy + wg * (0.25 * pgy + 0.5 * cgy + 0.25 * gy[kn])
                        gz[kc] = omg * cgz + wg * (0.25 * pgz + 0.5 * cgz + 0.25 * gz[kn])
                pax, pay, paz = cax, cay, caz
                pgx, pgy, pgz = cgx, cgy, cgz

    # Compute nominal sampling period dt_nom
    pos_dts = []
    if m >= 2:
        step_dt = max(1, (m - 1) // 2000)
        for idx_k in range(0, m - 1, step_dt):
            dt_v = t_s[active_idx[idx_k + 1]] - t_s[active_idx[idx_k]]
            if 1e-5 < dt_v < 0.1:
                pos_dts.append(dt_v)
    dt_nom = _median_list(pos_dts) if pos_dts else 0.005

    return (ax, ay, az), (gx, gy, gz), dup_mask, invalid_mask, fail_start, noise_mult, dt_nom


# ---------------------------------------------------------------------------
# Degradation Monitor & Fast Sparse 15-State Student-t ESKF
# ---------------------------------------------------------------------------

class DegradationMonitor:
    __slots__ = (
        "alpha", "kappa_max", "var_acc_floor", "var_gyr_floor", "sample_count",
        "prev_a", "prev_w", "has_prev",
        "var_acc", "var_gyr", "var_acc_ref", "var_gyr_ref",
        "calibrated", "quality", "kappa"
    )

    def __init__(
        self,
        alpha: float = 0.01,
        kappa_max: float = 50.0,
        var_acc_floor: float = 0.25,
        var_gyr_floor: float = 0.0025,
    ):
        self.alpha = alpha
        self.kappa_max = kappa_max
        self.var_acc_floor = var_acc_floor
        self.var_gyr_floor = var_gyr_floor
        self.reset()

    def reset(self):
        self.sample_count = 0
        self.prev_a = (0.0, 0.0, 0.0)
        self.prev_w = (0.0, 0.0, 0.0)
        self.has_prev = False
        self.var_acc = 0.0
        self.var_gyr = 0.0
        self.var_acc_ref = self.var_acc_floor
        self.var_gyr_ref = self.var_gyr_floor
        self.calibrated = False
        self.quality = 1.0
        self.kappa = 1.0

    def step(
        self,
        t_elapsed: float,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
        t_init: float,
        noise_mult: float = 1.0,
    ):
        if not self.has_prev:
            self.prev_a = (ax, ay, az)
            self.prev_w = (gx, gy, gz)
            self.has_prev = True
            self.sample_count = 1
            return

        da_x = ax - self.prev_a[0]
        da_y = ay - self.prev_a[1]
        da_z = az - self.prev_a[2]
        rk2 = da_x * da_x + da_y * da_y + da_z * da_z

        dw_x = gx - self.prev_w[0]
        dw_y = gy - self.prev_w[1]
        dw_z = gz - self.prev_w[2]
        rhok2 = dw_x * dw_x + dw_y * dw_y + dw_z * dw_z

        self.prev_a = (ax, ay, az)
        self.prev_w = (gx, gy, gz)

        alpha = self.alpha
        if self.sample_count == 1:
            self.var_acc = rk2
            self.var_gyr = rhok2
        else:
            om = 1.0 - alpha
            self.var_acc = om * self.var_acc + alpha * rk2
            self.var_gyr = om * self.var_gyr + alpha * rhok2
        self.sample_count += 1

        if t_elapsed < t_init:
            self.var_acc_ref = max(self.var_acc_floor, self.var_acc)
            self.var_gyr_ref = max(self.var_gyr_floor, self.var_gyr)
            self.quality = 1.0
            self.kappa = 1.0
            self.calibrated = False
        else:
            if not self.calibrated:
                self.calibrated = True
                if self.var_acc_ref < self.var_acc_floor:
                    self.var_acc_ref = max(self.var_acc_floor, self.var_acc)
                if self.var_gyr_ref < self.var_gyr_floor:
                    self.var_gyr_ref = max(self.var_gyr_floor, self.var_gyr)

            n_acc = math.sqrt(max(0.0, self.var_acc) / self.var_acc_ref) if self.var_acc_ref > 1e-12 else 1.0
            n_gyr = math.sqrt(max(0.0, self.var_gyr) / self.var_gyr_ref) if self.var_gyr_ref > 1e-12 else 1.0
            cap_ratio = max(1.5, 2.0 * noise_mult)
            noise_ratio = max(1.0, noise_mult, min(n_gyr, cap_ratio), min(n_acc, cap_ratio))

            self.quality = max(0.01, min(1.0, 1.0 / noise_ratio))
            self.kappa = max(1.0, min(self.kappa_max, noise_ratio))


_ALL_IDX = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)
_NO_POS_IDX = (0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14)


class SparseESKF:
    __slots__ = (
        "p", "v", "q", "bg", "ba", "P", "g_mag", "params", "dt_nom",
        "vibe_ema", "vibe_clean", "ped_energy", "gyro_ema", "dyn_acc_ema",
        "prev_a_raw", "has_prev_raw",
        "alpha_vibe", "alpha_ped", "alpha_dyn",
        "recover_counter",
        "win_w", "win_w_vec", "win_a_norm", "win_gait",
        "zupt_min_len", "_sum_w", "_sum_an", "_sum_an2",
        "_sum_gx", "_sum_gy", "_sum_gz", "_sum_g2",
        "_gait_step", "_gait_da_pp",
        "ring", "ring_cap", "monitor", "q_scale_g", "q_scale_a",
        "fp", "hp0", "hp1", "hp2", "k0", "k1", "k2", "w0", "w1", "w2", "dx",
        "_rot_cached", "_rot_q", "drag_e1", "drag_e2", "_step_idx", "is_drone_seq"
    )

    def __init__(self, params: dict, dt_nom: float = 0.005):
        self.params = params
        self.dt_nom = dt_nom if dt_nom > 1e-5 else 0.005
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.bg = [0.0, 0.0, 0.0]
        self.ba = [0.0, 0.0, 0.0]
        self.P = [[0.0] * 15 for _ in range(15)]
        self.g_mag = G_STD
        self.is_drone_seq = False
        self.drag_e1 = (1.0, 0.0, 0.0)
        self.drag_e2 = (0.0, 1.0, 0.0)
        self._step_idx = 0

        self.vibe_ema = 0.0
        self.vibe_clean = 0.0
        self.ped_energy = 0.0
        self.gyro_ema = 0.0
        self.dyn_acc_ema = 0.0
        self.prev_a_raw = [0.0, 0.0, 0.0]
        self.has_prev_raw = False
        self.recover_counter = 0

        dt_scale = self.dt_nom / 0.005 if self.dt_nom > 1e-6 else 1.0
        self.alpha_vibe = 1.0 - math.exp(-0.0202027 * dt_scale) if dt_scale != 1.0 else 0.02
        self.alpha_ped = 1.0 - math.exp(-0.0100503 * dt_scale) if dt_scale != 1.0 else 0.01
        self.alpha_dyn = 1.0 - math.exp(-0.0512933 * dt_scale) if dt_scale != 1.0 else 0.05

        fs = 1.0 / self.dt_nom
        zupt_len = max(10, int(round(params.get("zupt_window_s", 0.25) * fs)))
        gait_len = max(40, int(round(0.80 * fs)))
        self.zupt_min_len = max(8, int(round(0.10 * fs)))
        self.win_w = deque(maxlen=zupt_len)
        self.win_w_vec = deque(maxlen=zupt_len)
        self.win_a_norm = deque(maxlen=zupt_len)
        self.win_gait = deque(maxlen=gait_len)

        # O(1) running sums for ZUPT/ZARU
        self._sum_w = 0.0
        self._sum_an = 0.0
        self._sum_an2 = 0.0
        self._sum_gx = 0.0
        self._sum_gy = 0.0
        self._sum_gz = 0.0
        self._sum_g2 = 0.0
        self._gait_step = 0
        self._gait_da_pp = 0.0

        min_ring = max(params.get("ring_buffer_size", 256), int(round(params.get("blind_window_s", 0.585) * fs)) + 32)
        self.ring_cap = min_ring
        self.ring = deque(maxlen=self.ring_cap)

        self.monitor = DegradationMonitor(
            alpha=params.get("monitor_alpha", 0.01),
            kappa_max=params.get("kappa_max", 50.0),
            var_acc_floor=params.get("monitor_var_acc_floor", 0.0),
            var_gyr_floor=params.get("monitor_var_gyr_floor", 0.0),
        )
        self.q_scale_g = 1.0
        self.q_scale_a = 1.0

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
        self._rot_q = None
        self._rot_cached = None

    def _get_rot(self):
        if self._rot_q is not self.q:
            self._rot_cached = quat_to_rot(self.q)
            self._rot_q = self.q
        return self._rot_cached

    def _push_zupt_sample(self, w_mag: float, a_norm: float, gx: float, gy: float, gz: float):
        if len(self.win_w) == self.win_w.maxlen:
            old_w = self.win_w[0]
            old_an = self.win_a_norm[0]
            ogx, ogy, ogz = self.win_w_vec[0]
            self._sum_w -= old_w
            self._sum_an -= old_an
            self._sum_an2 -= old_an * old_an
            self._sum_gx -= ogx
            self._sum_gy -= ogy
            self._sum_gz -= ogz
            self._sum_g2 -= (ogx * ogx + ogy * ogy + ogz * ogz)
        self.win_w.append(w_mag)
        self.win_a_norm.append(a_norm)
        self.win_w_vec.append((gx, gy, gz))
        self._sum_w += w_mag
        self._sum_an += a_norm
        self._sum_an2 += a_norm * a_norm
        self._sum_gx += gx
        self._sum_gy += gy
        self._sum_gz += gz
        self._sum_g2 += (gx * gx + gy * gy + gz * gz)

    def initialize(
        self,
        t_arr,
        acc_arr,
        gyr_arr,
        dup_mask,
        invalid_mask,
        end_init_idx: int,
        fail_start: int,
        noise_mult=None,
    ):
        # Accept either (ax, ay, az) tuple of arrays or 2D row-indexed array
        if isinstance(acc_arr, tuple) and len(acc_arr) == 3 and not isinstance(acc_arr[0], (int, float)):
            ax_a, ay_a, az_a = acc_arr
            gx_a, gy_a, gz_a = gyr_arr
            flat = True
        else:
            flat = False

        wx_sum = wy_sum = wz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        cnt = 0
        ws = []
        accs = []
        anorms = []
        valid_ks = []
        sqrt = math.sqrt

        for k in range(end_init_idx):
            if dup_mask[k] or invalid_mask[k]:
                continue
            if flat:
                ax, ay, az = ax_a[k], ay_a[k], az_a[k]
                gx, gy, gz = gx_a[k], gy_a[k], gz_a[k]
            else:
                ax, ay, az = float(acc_arr[k][0]), float(acc_arr[k][1]), float(acc_arr[k][2])
                gx, gy, gz = float(gyr_arr[k][0]), float(gyr_arr[k][1]), float(gyr_arr[k][2])
            wx_sum += gx; wy_sum += gy; wz_sum += gz
            ax_sum += ax; ay_sum += ay; az_sum += az
            ws.append((gx, gy, gz))
            accs.append((ax, ay, az))
            anorms.append(sqrt(ax * ax + ay * ay + az * az))
            valid_ks.append(k)
            cnt += 1

        if cnt < 1:
            self.q = (1.0, 0.0, 0.0, 0.0)
            self.g_mag = G_STD
            self._set_p0(False)
            return

        if cnt == 1:
            a_ref = accs[0]
            a_ref_n = anorms[0] if anorms[0] >= 1e-6 else G_STD
            vx, vy, vz = a_ref[0] / a_ref_n, a_ref[1] / a_ref_n, a_ref[2] / a_ref_n
            if 1.0 + vz < 1e-4:
                self.q = (0.0, 1.0, 0.0, 0.0)
            else:
                qw = 1.0 + vz
                inv_qn = 1.0 / sqrt(qw * qw + vx * vx + vy * vy)
                self.q = (qw * inv_qn, vy * inv_qn, -vx * inv_qn, 0.0)
            self.g_mag = G_STD
            self.bg = [0.0, 0.0, 0.0]
            self.ba = [0.0, 0.0, 0.0]
            self._set_p0(False)
            return

        inv_c = 1.0 / cnt
        w_bar = (wx_sum * inv_c, wy_sum * inv_c, wz_sum * inv_c)
        a_bar = (ax_sum * inv_c, ay_sum * inv_c, az_sum * inv_c)

        w_var = sum(
            (g[0] - w_bar[0]) ** 2 + (g[1] - w_bar[1]) ** 2 + (g[2] - w_bar[2]) ** 2
            for g in ws
        ) / (3.0 * cnt)
        a_mean_n = sum(anorms) * inv_c
        a_var = sum((an - a_mean_n) ** 2 for an in anorms) * inv_c
        sigma_w = sqrt(w_var)
        sigma_a = sqrt(a_var)
        w_bar_mag = sqrt(w_bar[0] ** 2 + w_bar[1] ** 2 + w_bar[2] ** 2)

        motion_detected = (
            (sigma_w > self.params["motion_detect_gyro_std"])
            or (sigma_a > self.params["motion_detect_acc_std"])
            or (w_bar_mag > 0.20)
        )

        # Sequence-level drone vs pedestrian regime classification via rotor vibration
        da_samp = []
        n_scan = min(cnt, 4000)
        step_da = max(1, n_scan // 500)
        for u in range(1, n_scan, step_da):
            if not dup_mask[u] and not invalid_mask[u]:
                axu, ayu, azu = (ax_a[u], ay_a[u], az_a[u]) if flat else (acc_arr[u][0], acc_arr[u][1], acc_arr[u][2])
                axp, ayp, azp = (ax_a[u - 1], ay_a[u - 1], az_a[u - 1]) if flat else (acc_arr[u - 1][0], acc_arr[u - 1][1], acc_arr[u - 1][2])
                da = sqrt((axu - axp) ** 2 + (ayu - ayp) ** 2 + (azu - azp) ** 2)
                da_samp.append(da)
        if da_samp:
            da_samp.sort()
            med_da = da_samp[len(da_samp) >> 1]
            self.is_drone_seq = (med_da > 0.35)
        else:
            self.is_drone_seq = False

        best_mid_idx = 0
        if motion_detected and self.params.get("enable_static_window_mining", True):
            # Find quietest 0.4s sub-window within T_init for cleaner gravity direction and gyro bias
            best_a_var = 1e18
            best_a_bar = a_bar
            best_start_i = 0
            win_len = min(max(10, int(round(0.40 / self.dt_nom))), cnt)
            if win_len >= 10:
                step_w = max(1, win_len // 8)
                for start_i in range(0, max(1, cnt - win_len + 1), step_w):
                    sub_an = anorms[start_i:start_i + win_len]
                    m_an = sum(sub_an) / win_len
                    v_an = sum((x - m_an) ** 2 for x in sub_an) / win_len
                    if v_an < best_a_var:
                        best_a_var = v_an
                        best_start_i = start_i
                        sub_ac = accs[start_i:start_i + win_len]
                        best_a_bar = (
                            sum(x[0] for x in sub_ac) / win_len,
                            sum(x[1] for x in sub_ac) / win_len,
                            sum(x[2] for x in sub_ac) / win_len,
                        )
                        best_mid_idx = start_i + (win_len >> 1)
            a_ref = best_a_bar
            self.g_mag = G_STD

            self.bg = [0.0, 0.0, 0.0]
            # Only accept sub-window gyro mean as b_g0 if the sub-window is truly static
            if win_len >= 10 and best_a_var < 0.0225:
                sub_ws = ws[best_start_i:best_start_i + win_len]
                inv_wl = 1.0 / win_len
                gmx = sum(g[0] for g in sub_ws) * inv_wl
                gmy = sum(g[1] for g in sub_ws) * inv_wl
                gmz = sum(g[2] for g in sub_ws) * inv_wl
                g_mean_mag = sqrt(gmx * gmx + gmy * gmy + gmz * gmz)
                gv = sum(
                    (g[0] - gmx) ** 2 + (g[1] - gmy) ** 2 + (g[2] - gmz) ** 2
                    for g in sub_ws
                ) * inv_wl / 3.0
                a_sub_mag = sqrt(best_a_bar[0] ** 2 + best_a_bar[1] ** 2 + best_a_bar[2] ** 2)
                if gv < 0.0004 and g_mean_mag < 0.12 and abs(a_sub_mag - G_STD) < 0.35:
                    self.bg = [gmx, gmy, gmz]

        elif motion_detected:
            a_ref = accs[0]
            self.bg = [0.0, 0.0, 0.0]
            self.g_mag = G_STD
        else:
            a_ref = a_bar
            self.bg = [w_bar[0], w_bar[1], w_bar[2]]
            a_norm = sqrt(a_ref[0] ** 2 + a_ref[1] ** 2 + a_ref[2] ** 2)
            self.g_mag = max(self.params["g_mag_clamp_min"], min(self.params["g_mag_clamp_max"], a_norm))

        # Scan early clean trajectory (t <= 6.5 s) for a verified zero-motion pause to refine b_g0 and a_ref
        t0_val = float(t_arr[0])
        scan_limit = fail_start
        t_scan_max = t0_val + 6.5
        for idx_s in range(fail_start):
            if float(t_arr[idx_s]) > t_scan_max:
                scan_limit = idx_s
                break

        w_mine = max(20, int(round(0.40 / self.dt_nom)))
        if scan_limit >= w_mine:
            best_wv = 0.0005
            best_bg = None
            best_a_mine = None
            step_m = max(1, w_mine >> 2)
            for sk in range(0, scan_limit - w_mine, step_m):
                ok = True
                for u in range(sk, sk + w_mine):
                    if dup_mask[u] or invalid_mask[u]:
                        ok = False
                        break
                    if noise_mult is not None and noise_mult[u] > 2.5:
                        ok = False
                        break
                if not ok:
                    continue
                sx = sy = sz = sg2 = san = san2 = sax = say = saz = 0.0
                for u in range(sk, sk + w_mine):
                    if flat:
                        ux, uy, uz = gx_a[u], gy_a[u], gz_a[u]
                        axu, ayu, azu = ax_a[u], ay_a[u], az_a[u]
                    else:
                        ux, uy, uz = float(gyr_arr[u][0]), float(gyr_arr[u][1]), float(gyr_arr[u][2])
                        axu, ayu, azu = float(acc_arr[u][0]), float(acc_arr[u][1]), float(acc_arr[u][2])
                    sx += ux; sy += uy; sz += uz
                    sg2 += ux * ux + uy * uy + uz * uz
                    an_u = sqrt(axu * axu + ayu * ayu + azu * azu)
                    san += an_u; san2 += an_u * an_u
                    sax += axu; say += ayu; saz += azu
                inv_wm = 1.0 / w_mine
                gmx, gmy, gmz = sx * inv_wm, sy * inv_wm, sz * inv_wm
                gv = (sg2 * inv_wm - (gmx * gmx + gmy * gmy + gmz * gmz)) / 3.0
                am = san * inv_wm
                av = san2 * inv_wm - am * am
                g_mean_mag = sqrt(gmx * gmx + gmy * gmy + gmz * gmz)
                if gv < best_wv and av < 0.010 and abs(am - G_STD) < 0.25 and g_mean_mag < 0.10:
                    best_wv = gv
                    best_bg = (gmx, gmy, gmz)
                    best_a_mine = (sax * inv_wm, say * inv_wm, saz * inv_wm)
            if best_bg is not None:
                self.bg = [best_bg[0], best_bg[1], best_bg[2]]
                if best_a_mine is not None and (motion_detected or abs(a_norm - G_STD) > 0.15):
                    a_ref = best_a_mine

        a_ref_n = sqrt(a_ref[0] ** 2 + a_ref[1] ** 2 + a_ref[2] ** 2)
        if a_ref_n < 1e-6:
            a_ref_n = G_STD

        vx, vy, vz = a_ref[0] / a_ref_n, a_ref[1] / a_ref_n, a_ref[2] / a_ref_n
        if 1.0 + vz < 1e-4:
            q_init = (0.0, 1.0, 0.0, 0.0)
        else:
            qw = 1.0 + vz
            qx = vy
            qy = -vx
            inv_qn = 1.0 / sqrt(qw * qw + qx * qx + qy * qy)
            q_init = (qw * inv_qn, qx * inv_qn, qy * inv_qn, 0.0)

        # Back-integrate clean startup gyro from best_mid_idx back to k = 0
        if motion_detected and best_mid_idx > 0:
            q_bt = q_init
            bg0, bg1, bg2 = self.bg
            for idx_b in range(best_mid_idx, 0, -1):
                k_curr = valid_ks[idx_b]
                k_prev = valid_ks[idx_b - 1]
                dt_b = float(t_arr[k_curr]) - float(t_arr[k_prev])
                if 0.0 < dt_b < 0.1:
                    g_b = ws[idx_b]
                    q_inv = rotvec_to_quat(
                        -(g_b[0] - bg0) * dt_b,
                        -(g_b[1] - bg1) * dt_b,
                        -(g_b[2] - bg2) * dt_b,
                    )
                    q_bt = quat_norm_align(quat_mul(q_bt, q_inv), q_bt)
            q_init = q_bt

        self.q = q_init

        scale_g = (sigma_w / 0.020) if sigma_w > 1e-5 else 0.2
        scale_a = (sigma_a / 0.300) if sigma_a > 1e-5 else 0.2
        self.q_scale_g = max(0.2, min(1.0, scale_g))
        self.q_scale_a = max(0.2, min(1.0, scale_a))

        # Mounting-invariant thrust-orthogonal drag frame
        v_arb = (0.0, 0.0, 1.0) if abs(vz) < 0.8 else (0.0, 1.0, 0.0)
        dot_u = v_arb[0] * vx + v_arb[1] * vy + v_arb[2] * vz
        e1x = v_arb[0] - dot_u * vx
        e1y = v_arb[1] - dot_u * vy
        e1z = v_arb[2] - dot_u * vz
        e1_n = sqrt(e1x * e1x + e1y * e1y + e1z * e1z)
        if e1_n < 1e-6:
            e1x, e1y, e1z = 1.0, 0.0, 0.0
            e1_n = 1.0
        e1x /= e1_n; e1y /= e1_n; e1z /= e1_n
        self.drag_e1 = (e1x, e1y, e1z)
        self.drag_e2 = (
            vy * e1z - vz * e1y,
            vz * e1x - vx * e1z,
            vx * e1y - vy * e1x,
        )

        self.ba = [0.0, 0.0, 0.0]
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self._rot_q = None
        self._set_p0(motion_detected)

    def _set_p0(self, motion_detected: bool):
        P = self.P
        for i in range(15):
            row = P[i]
            for j in range(15):
                row[j] = 0.0
        sig_th = self.params["sigma_theta_0"] * (
            self.params["motion_detect_sigma_theta_mult"] if motion_detected else 1.0
        )
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
        vc = self.vibe_clean
        ft = self.params["flight_vibe_thresh"]
        if vc > ft:
            vibe_score = max(0.0, 1.0 - (vc - ft) / 0.40)
        else:
            vibe_score = 1.0
        v = self.v
        spd = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if spd < 0.15:
            speed_score = 0.0
        elif spd < 0.40:
            speed_score = (spd - 0.15) * 4.0
        elif spd <= 2.30:
            speed_score = 1.0
        else:
            speed_score = max(0.0, 1.0 - (spd - 2.30))
        ped_score = min(1.0, self.ped_energy / 0.45)
        return max(0.0, min(1.0, vibe_score * speed_score * (0.2 + 0.8 * ped_score)))

    def step(
        self,
        dt: float,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
        noise_mult: float,
        t_elapsed: float,
    ):
        if dt <= 0.0:
            return
        self._step_idx += 1
        high_rate = (dt <= 0.008)
        do_nhc = True
        do_tilt = (not high_rate) or ((self._step_idx & 1) == 1)
        nhc_r_mult = 1.0
        tilt_r_mult = 0.5 if high_rate else 1.0

        pr = self.params
        bg = self.bg
        ba = self.ba
        wx = gx - bg[0]
        wy = gy - bg[1]
        wz = gz - bg[2]
        acx = ax - ba[0]
        acy = ay - ba[1]
        acz = az - ba[2]

        sqrt = math.sqrt
        w_mag = sqrt(wx * wx + wy * wy + wz * wz)
        a_norm = sqrt(ax * ax + ay * ay + az * az)

        self.monitor.step(t_elapsed, ax, ay, az, gx, gy, gz, pr["T_init"], noise_mult)
        quality = self.monitor.quality
        kappa = self.monitor.kappa

        if not self.has_prev_raw:
            self.prev_a_raw = [ax, ay, az]
            self.has_prev_raw = True
        else:
            pa = self.prev_a_raw
            da_x = ax - pa[0]
            da_y = ay - pa[1]
            da_z = az - pa[2]
            d_a = sqrt(da_x * da_x + da_y * da_y + da_z * da_z)
            self.vibe_ema = (1.0 - self.alpha_vibe) * self.vibe_ema + self.alpha_vibe * d_a
            # Quadrature noise subtraction isolates true rotor vibration from progressive white noise
            nm_ex = max(0.0, noise_mult - 2.0)
            if nm_ex > 0.0:
                jam_da = 2.257 * pr["sigma_a_base"] * nm_ex
                vc2 = self.vibe_ema * self.vibe_ema - jam_da * jam_da
                self.vibe_clean = sqrt(vc2) if vc2 > 0.0 else 0.0
            else:
                self.vibe_clean = self.vibe_ema

            dev_a = abs(a_norm - self.g_mag)
            self.ped_energy = (1.0 - self.alpha_ped) * self.ped_energy + self.alpha_ped * (dev_a * dev_a)
            if quality >= 0.5 and dev_a < 12.0:
                a_excess = a_norm - self.g_mag
                if a_excess > 0.0:
                    self.dyn_acc_ema = (1.0 - self.alpha_dyn) * self.dyn_acc_ema + self.alpha_dyn * a_excess
                else:
                    self.dyn_acc_ema *= (1.0 - self.alpha_dyn)
            pa[0] = ax; pa[1] = ay; pa[2] = az

        self.win_gait.append(a_norm)
        self._gait_step += 1
        self.gyro_ema = (1.0 - self.alpha_vibe) * self.gyro_ema + self.alpha_vibe * w_mag

        if self.is_drone_seq:
            is_flight = (self.vibe_clean > (pr["flight_vibe_thresh"] * 0.5)) or (w_mag > 0.15)
            is_pedestrian = False
        else:
            is_flight = False
            is_pedestrian = self.ped_energy > pr["ped_energy_thresh"]

        # Regime-decoupled OU damping
        if is_flight:
            cur_tau_ou = pr.get("tau_ou_flight", pr["tau_ou"])
            if self.gyro_ema < 0.40:
                cur_tau_ou = cur_tau_ou / max(1.0, min(2.0, noise_mult))
        elif is_pedestrian:
            cur_tau_ou = pr.get("tau_ou_walk", pr["tau_ou"])
        else:
            cur_tau_ou = pr["tau_ou"]

        if pr.get("enable_wiener_ou", False) and noise_mult > 1.5:
            cur_tau_ou = max(0.45, cur_tau_ou / (1.0 + pr.get("wiener_ou_beta", 0.15) * (noise_mult - 1.5)))

        # Dynamic velocity envelope (Weinberg biomechanical bound during walking)
        dyn_envelope = pr["max_velocity_mps"] + (0.5 * self.dyn_acc_ema if is_flight else 0.0)
        if dyn_envelope > 6.5:
            dyn_envelope = 6.5

        v_weinberg = None
        if is_pedestrian and pr.get("enable_weinberg_bound", True) and len(self.win_gait) >= 40:
            if self._gait_step >= 10 or self._gait_da_pp <= 0.0:
                self._gait_step = 0
                wg = self.win_gait
                self._gait_da_pp = min(20.0, max(0.05, max(wg) - min(wg)))
            v_weinberg = pr["weinberg_k"] * (self._gait_da_pp ** 0.25)
            max_v = max(0.80, min(pr["ped_max_velocity_mps"], 1.35 * v_weinberg))
        elif is_pedestrian:
            max_v = pr["ped_max_velocity_mps"]
        else:
            max_v = dyn_envelope

        # 1. Midpoint strapdown propagation
        q_cur = self.q
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = self._get_rot()

        q_half = rotvec_to_quat(0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt)
        rm00, rm01, rm02, rm10, rm11, rm12, rm20, rm21, rm22 = quat_to_rot(quat_mul(q_cur, q_half))
        q_step = rotvec_to_quat(wx * dt, wy * dt, wz * dt)
        q_new = quat_norm_align(quat_mul(q_cur, q_step), q_cur)

        awx = rm00 * acx + rm01 * acy + rm02 * acz
        awy = rm10 * acx + rm11 * acy + rm12 * acz
        awz = rm20 * acx + rm21 * acy + rm22 * acz - self.g_mag

        gam = math.exp(-dt / max(0.1, cur_tau_ou))
        v = self.v
        p = self.p
        vx_old, vy_old, vz_old = v[0], v[1], v[2]
        vx_new = gam * vx_old + awx * dt
        vy_new = gam * vy_old + awy * dt
        vz_new = gam * vz_old + awz * dt

        half_dt = 0.5 * dt
        p[0] += (vx_old + vx_new) * half_dt
        p[1] += (vy_old + vy_new) * half_dt
        p[2] += (vz_old + vz_new) * half_dt
        v[0] = vx_new
        v[1] = vy_new
        v[2] = vz_new
        self.q = q_new
        self._rot_cached = quat_to_rot(q_new)
        self._rot_q = q_new

        # 2. Fast 9x9 Upper-Triangle Covariance Propagation
        #    F_theta = Exp(-w*dt) == R_step^T (saves a full rotvec_to_quat call)
        rs00, rs01, rs02, rs10, rs11, rs12, rs20, rs21, rs22 = quat_to_rot(q_step)
        gdt = self.g_mag * dt
        if pr["enable_inekf_transition"]:
            ft00, ft01, ft02 = gdt * r10, gdt * r11, gdt * r12
            ft10, ft11, ft12 = -gdt * r00, -gdt * r01, -gdt * r02
            ft20, ft21, ft22 = 0.0, 0.0, 0.0
        else:
            ft00 = -(r01 * acz - r02 * acy) * dt
            ft01 = -(-r00 * acz + r02 * acx) * dt
            ft02 = -(r00 * acy - r01 * acx) * dt
            ft10 = -(r11 * acz - r12 * acy) * dt
            ft11 = -(-r10 * acz + r12 * acx) * dt
            ft12 = -(r10 * acy - r11 * acx) * dt
            ft20 = -(r21 * acz - r22 * acy) * dt
            ft21 = -(-r20 * acz + r22 * acx) * dt
            ft22 = -(r20 * acy - r21 * acx) * dt

        mult_g = max(noise_mult * self.q_scale_g, kappa)
        mult_a = max(self.q_scale_a, kappa)
        sev_scale = min(3.0, max(0.25, 0.25 * noise_mult)) if pr.get("enable_sev_bias_rw", True) else 1.0
        q_th = (pr["sigma_g_base"] * mult_g) ** 2 * dt
        q_v = (pr["sigma_a_base"] * mult_a) ** 2 * dt
        q_bg = ((pr["sigma_bg_base"] * sev_scale) ** 2) * dt
        q_ba = ((pr["sigma_ba_base"] * sev_scale) ** 2) * dt

        self._propagate_cov(
            dt, gam,
            rs00, rs10, rs20, rs01, rs11, rs21, rs02, rs12, rs22,
            ft00, ft01, ft02, ft10, ft11, ft12, ft20, ft21, ft22,
            r00, r01, r02, r10, r11, r12, r20, r21, r22,
            q_th, q_v, q_bg, q_ba,
        )

        r_scale = 1.0
        if self.recover_counter > 0:
            r_scale = 1.0 + 10.0 * (self.recover_counter / float(pr["recover_steps"]))
            self.recover_counter -= 1

        # 3. O(1) Bias-Invariant Rest Detection (ZUPT + ZARU)
        self._push_zupt_sample(w_mag, a_norm, gx, gy, gz)

        is_zupt = False
        n_win = len(self.win_w)
        if n_win >= self.zupt_min_len:
            inv_k = 1.0 / n_win
            w_mean_n = self._sum_w * inv_k
            a_mean_n = self._sum_an * inv_k
            a_var_n = self._sum_an2 * inv_k - a_mean_n * a_mean_n
            if a_var_n < 0.0:
                a_var_n = 0.0

            g_mx = self._sum_gx * inv_k
            g_my = self._sum_gy * inv_k
            g_mz = self._sum_gz * inv_k
            g_var = (self._sum_g2 * inv_k - (g_mx * g_mx + g_my * g_my + g_mz * g_mz)) * (1.0 / 3.0)
            if g_var < 0.0:
                g_var = 0.0

            zupt_lim = pr["zupt_gyro_mean"]
            rest_gyro_ok = (w_mean_n < zupt_lim and w_mag <= zupt_lim * 1.5) or (
                pr.get("enable_zaru", True) and g_var < pr["zupt_gyro_var"] and w_mean_n < 0.08
            )
            acc_dev = pr["zupt_acc_dev"]
            # E83 Variance-Rest Stillness Detector
            raw_w_mag = sqrt(g_mx * g_mx + g_my * g_my + g_mz * g_mz)
            e83_still = (
                pr.get("enable_zaru", True)
                and g_var < 0.0005
                and a_var_n < 0.010
                and abs(a_mean_n - self.g_mag) < 0.25
                and raw_w_mag < 0.10
                and w_mag < 0.15
            )
            if e83_still or (quality >= pr["zupt_min_quality"] and rest_gyro_ok and abs(a_mean_n - self.g_mag) < acc_dev and a_var_n < (acc_dev * acc_dev)):
                is_zupt = True

        # 4. Regime Updates (ZUPT+ZARU / Flight / Posture-Invariant Walking NHC)
        if is_zupt:
            sig_z = pr["sigma_zupt"]
            r_z = (sig_z * sig_z) * r_scale
            self._update_block3(
                3, -v[0], -v[1], -v[2],
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                r_z, r_z, r_z, gate_threshold=1e9, clear_pv=False,
                m_dim=3, nu=pr["nu_vel"]
            )
            if pr.get("enable_zaru", True):
                sz = pr["zaru_sigma"]
                rz_bg = (sz * sz) * r_scale
                self._update_block3(
                    9, gx - bg[0], gy - bg[1], gz - bg[2],
                    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                    rz_bg, rz_bg, rz_bg, gate_threshold=1e9, clear_pv=False,
                    m_dim=3, nu=pr["nu_vel"]
                )
        elif is_flight:
            if pr["enable_rotor_drag"] and w_mag <= 2.5 and (3.0 <= a_norm <= 35.0):
                r00, r01, r02, r10, r11, r12, r20, r21, r22 = self._get_rot()
                vx, vy, vz = v[0], v[1], v[2]
                vbx = r00 * vx + r10 * vy + r20 * vz
                vby = r01 * vx + r11 * vy + r21 * vz
                vbz = r02 * vx + r12 * vy + r22 * vz

                e1x, e1y, e1z = self.drag_e1
                e2x, e2y, e2z = self.drag_e2
                kd = pr["rotor_drag_kd"]
                y0 = (e1x * acx + e1y * acy + e1z * acz) + kd * (e1x * vbx + e1y * vby + e1z * vbz)
                y1 = (e2x * acx + e2y * acy + e2z * acz) + kd * (e2x * vbx + e2y * vby + e2z * vbz)

                sd = pr["rotor_drag_sigma"]
                rd = (sd * sd) * r_scale
                re1_0 = r00 * e1x + r01 * e1y + r02 * e1z
                re1_1 = r10 * e1x + r11 * e1y + r12 * e1z
                re1_2 = r20 * e1x + r21 * e1y + r22 * e1z
                re2_0 = r00 * e2x + r01 * e2y + r02 * e2z
                re2_1 = r10 * e2x + r11 * e2y + r12 * e2z
                re2_2 = r20 * e2x + r21 * e2y + r22 * e2z

                self._update_block2(
                    3, y0, y1,
                    -kd * re1_0, -kd * re1_1, -kd * re1_2,
                    -kd * re2_0, -kd * re2_1, -kd * re2_2,
                    rd, rd, gate_threshold=pr["rotor_drag_gate"], clear_pv=True,
                    nu=pr["nu_vel"]
                )
            elif pr.get("enable_flight_vz_prior", False):
                rvz = (pr["flight_vz_sigma"] ** 2) * r_scale
                self._update_block1(
                    3, -v[2], 0.0, 0.0, 1.0,
                    rvz, gate_threshold=16.0, clear_pv=True, nu=pr["nu_vel"]
                )
        elif is_pedestrian and pr["enable_pedestrian_prior"] and do_nhc:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = self._get_rot()
            vx, vy, vz = v[0], v[1], v[2]
            rlat = (pr["ped_sigma_lat"] ** 2) * (r_scale * nhc_r_mult)
            rvert = (pr["ped_sigma_vert"] ** 2) * (r_scale * nhc_r_mult)
            if pr.get("enable_leveled_ped_nhc", True):
                # Gravity-leveled horizontal forward/lateral axes (immune to handheld roll/pitch)
                n_fx = sqrt(r00 * r00 + r10 * r10)
                if n_fx > 0.25:
                    inv_nfx = 1.0 / n_fx
                    fx_h, fy_h = r00 * inv_nfx, r10 * inv_nfx
                    lx_h, ly_h = -fy_h, fx_h
                else:
                    n_ly = sqrt(r01 * r01 + r11 * r11)
                    if n_ly > 1e-6:
                        inv_nly = 1.0 / n_ly
                        lx_h, ly_h = r01 * inv_nly, r11 * inv_nly
                        fx_h, fy_h = ly_h, -lx_h
                    else:
                        lx_h, ly_h = 0.0, 1.0
                        fx_h, fy_h = 1.0, 0.0
                v_lat_h = lx_h * vx + ly_h * vy
                self._update_block2(
                    3, -v_lat_h, -vz,
                    lx_h, ly_h, 0.0,
                    0.0, 0.0, 1.0,
                    rlat, rvert, gate_threshold=pr["ped_gate_threshold"], clear_pv=True,
                    nu=pr["nu_vel"]
                )
                # Fast 1D Weinberg forward walking speed cap
                if v_weinberg is not None:
                    v_fwd_h = fx_h * v[0] + fy_h * v[1]
                    spd_cap = 1.15 * v_weinberg
                    if abs(v_fwd_h) > spd_cap:
                        target_vf = math.copysign(spd_cap, v_fwd_h)
                        rw = (pr["weinberg_sigma"] ** 2) * (r_scale * nhc_r_mult)
                        self._update_block1(
                            3, target_vf - v_fwd_h,
                            fx_h, fy_h, 0.0,
                            rw, gate_threshold=16.0, clear_pv=True,
                            nu=pr["nu_vel"]
                        )
            else:
                vby = r01 * vx + r11 * vy + r21 * vz
                self._update_block2(
                    3, -vby, -vz,
                    r01, r11, r21,
                    0.0, 0.0, 1.0,
                    rlat, rvert, gate_threshold=pr["ped_gate_threshold"], clear_pv=True,
                    nu=pr["nu_vel"]
                )

        # 5. Noise-Adaptive Gravity Tilt Alignment (Student-t protected, immediate post-gap recovery)
        tau_base = pr["tilt_tau_a"] * max(1.0, min(3.0, noise_mult))
        q_clamped = max(0.0, min(1.0, quality))
        tau_eff = tau_base * (1.0 + 5.0 * (1.0 - q_clamped))

        if pr.get("enable_centripetal_comp", False) and is_pedestrian:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = self._get_rot()
            vx, vy, vz = v[0], v[1], v[2]
            vbx = r00 * vx + r10 * vy + r20 * vz
            vby = r01 * vx + r11 * vy + r21 * vz
            vbz = r02 * vx + r12 * vy + r22 * vz
            cpx = wy * vbz - wz * vby
            cpy = wz * vbx - wx * vbz
            cpz = wx * vby - wy * vbx
            cp_mag = sqrt(cpx * cpx + cpy * cpy + cpz * cpz)
            if cp_mag > 4.0:
                sc_cp = 4.0 / cp_mag
                cpx *= sc_cp; cpy *= sc_cp; cpz *= sc_cp
            agx = acx - cpx
            agy = acy - cpy
            agz = acz - cpz
            a_hat_norm = sqrt(agx * agx + agy * agy + agz * agz)
        else:
            agx, agy, agz = acx, acy, acz
            a_hat_norm = sqrt(acx * acx + acy * acy + acz * acz)

        if do_tilt and w_mag < pr["tilt_omega_max"] and abs(a_hat_norm - self.g_mag) < tau_eff and a_hat_norm > 1e-6:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = self._get_rot()
            inv_an = 1.0 / a_hat_norm
            zx, zy, zz = agx * inv_an, agy * inv_an, agz * inv_an
            hx, hy, hz = r20, r21, r22

            if pr.get("enable_tilt_jam_fix", False) and not is_flight:
                sigma_tilt = pr["tilt_sigma"] / math.sqrt(max(0.05, quality))
                a_scale_eff = pr["tilt_a_scale"] * max(1.0, noise_mult)
            else:
                sigma_tilt = pr["tilt_sigma"] / max(0.1, quality)
                a_scale_eff = pr["tilt_a_scale"]
            var_tilt = (sigma_tilt * sigma_tilt) * min(2.0, r_scale) * tilt_r_mult
            a_ratio = abs(a_hat_norm - self.g_mag) / a_scale_eff
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
            vx, vy, vz = v[0], v[1], v[2]
            vmag = sqrt(vx * vx + vy * vy + vz * vz)
            if vmag > max_v and vmag > 1e-9:
                sc = max_v / vmag - 1.0
                rv = (pr["velocity_bounding_sigma"] ** 2) * r_scale
                self._update_block3(
                    3, vx * sc, vy * sc, vz * sc,
                    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                    rv, rv, rv, gate_threshold=1e9, clear_pv=True,
                    m_dim=3, nu=pr["nu_vel"]
                )

        # 7. Store state into O(1) ring buffer for blind extrapolation
        w_walk = self.get_w_walk()
        self.ring.append((
            p[0], p[1], p[2],
            v[0], v[1], v[2],
            self.q[0], self.q[1], self.q[2], self.q[3],
            wx, wy, wz, w_walk
        ))

    def propagate_gap(
        self,
        gap_dt: float,
        dt_nom: float,
        wx_s: float, wy_s: float, wz_s: float,
        push_ring: bool = True,
    ):
        if gap_dt <= 0.0:
            return (wx_s, wy_s, wz_s)
        pr = self.params
        n_sub = max(1, min(pr.get("gap_substep_max", 800), int(round(gap_dt / max(dt_nom, 1e-4)))))
        dt_sub = gap_dt / n_sub
        dec_v = math.exp(-dt_sub / max(0.1, pr["tau_gap"]))
        dec_w = math.exp(-dt_sub / max(0.05, pr.get("tau_w_gap", 0.40)))
        half_dt = 0.5 * dt_sub

        q_cur = self.q
        vx, vy, vz = self.v[0], self.v[1], self.v[2]
        px, py, pz = self.p[0], self.p[1], self.p[2]
        wx, wy, wz = wx_s, wy_s, wz_s

        for _ in range(n_sub):
            vx_n = vx * dec_v
            vy_n = vy * dec_v
            vz_n = vz * dec_v
            px += (vx + vx_n) * half_dt
            py += (vy + vy_n) * half_dt
            pz += (vz + vz_n) * half_dt
            vx, vy, vz = vx_n, vy_n, vz_n

            wx_m = wx * (0.5 * (1.0 + dec_w))
            wy_m = wy * (0.5 * (1.0 + dec_w))
            wz_m = wz * (0.5 * (1.0 + dec_w))
            q_step = rotvec_to_quat(wx_m * dt_sub, wy_m * dt_sub, wz_m * dt_sub)
            q_cur = quat_norm_align(quat_mul(q_cur, q_step), q_cur)
            wx *= dec_w
            wy *= dec_w
            wz *= dec_w

        self.p[0], self.p[1], self.p[2] = px, py, pz
        self.v[0], self.v[1], self.v[2] = vx, vy, vz
        self.q = q_cur
        self._rot_q = None

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

        if push_ring:
            self.ring.append((
                px, py, pz, vx, vy, vz,
                q_cur[0], q_cur[1], q_cur[2], q_cur[3],
                wx, wy, wz, self.get_w_walk()
            ))
        return (wx, wy, wz)

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
        fp0, fp1, fp2, fp3, fp4, fp5, fp6, fp7, fp8 = self.fp

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

        fps = self.fp
        for i in range(9):
            row = fps[i]
            out = P[i]
            c0, c1, c2 = row[0], row[1], row[2]
            cv0, cv1, cv2 = row[3], row[4], row[5]
            cbg0, cbg1, cbg2 = row[9], row[10], row[11]
            cba0, cba1, cba2 = row[12], row[13], row[14]

            if i <= 0:
                out[0] = c0 * f00 + c1 * f01 + c2 * f02 - dt * cbg0
            if i <= 1:
                v = c0 * f10 + c1 * f11 + c2 * f12 - dt * cbg1
                out[1] = v
                p1[i] = v
            if i <= 2:
                v = c0 * f20 + c1 * f21 + c2 * f22 - dt * cbg2
                out[2] = v
                p2[i] = v
            if i <= 3:
                v = c0 * ft00 + c1 * ft01 + c2 * ft02 + gam * cv0 - (rdt00 * cba0 + rdt01 * cba1 + rdt02 * cba2)
                out[3] = v
                p3[i] = v
            if i <= 4:
                v = c0 * ft10 + c1 * ft11 + c2 * ft12 + gam * cv1 - (rdt10 * cba0 + rdt11 * cba1 + rdt12 * cba2)
                out[4] = v
                p4[i] = v
            if i <= 5:
                v = c0 * ft20 + c1 * ft21 + c2 * ft22 + gam * cv2 - (rdt20 * cba0 + rdt21 * cba1 + rdt22 * cba2)
                out[5] = v
                p5[i] = v
            if i <= 6:
                v = row[6] + dt * cv0
                out[6] = v
                p6[i] = v
            if i <= 7:
                v = row[7] + dt * cv1
                out[7] = v
                p7[i] = v
            v = row[8] + dt * cv2
            out[8] = v
            p8[i] = v

            out[9], out[10], out[11] = cbg0, cbg1, cbg2
            out[12], out[13], out[14] = cba0, cba1, cba2
            p9[i], p10[i], p11[i] = cbg0, cbg1, cbg2
            p12[i], p13[i], p14[i] = cba0, cba1, cba2

        p0[0] += q_th; p1[1] += q_th; p2[2] += q_th
        p3[3] += q_v;  p4[4] += q_v;  p5[5] += q_v
        p9[9] += q_bg; p10[10] += q_bg; p11[11] += q_bg
        p12[12] += q_ba; p13[13] += q_ba; p14[14] += q_ba

        for i in range(15):
            row_i = P[i]
            dv = row_i[i]
            if dv < 1e-12:
                row_i[i] = 1e-12
            elif dv > 1e6:
                sc = math.sqrt(1e6 / dv)
                for j in range(15):
                    row_i[j] *= sc
                for j in range(15):
                    P[j][i] *= sc

    def _inject_and_reset(self):
        dx = self.dx
        dth0, dth1, dth2 = dx[0], dx[1], dx[2]
        dth_sum = abs(dth0) + abs(dth1) + abs(dth2)
        if dth_sum > 1e-9:
            qn = quat_norm_align(quat_mul(self.q, rotvec_to_quat(dth0, dth1, dth2)), self.q)
            self.q = qn
            self._rot_cached = quat_to_rot(qn)
            self._rot_q = qn

            if dth_sum >= 1e-4:
                P = self.P
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

        v = self.v
        p = self.p
        bg = self.bg
        ba = self.ba
        v[0] += dx[3]; v[1] += dx[4]; v[2] += dx[5]
        p[0] += dx[6]; p[1] += dx[7]; p[2] += dx[8]
        bg[0] += dx[9]; bg[1] += dx[10]; bg[2] += dx[11]
        ba[0] += dx[12]; ba[1] += dx[13]; ba[2] += dx[14]

    def _update_block1(
        self, blk: int, y0: float,
        h00: float, h01: float, h02: float,
        r0: float,
        gate_threshold: float = 1e9, clear_pv: bool = False, nu: float = 5.0
    ):
        """Fast 1D scalar update with exact Student-t DOF (m_dim = 1)."""
        P = self.P
        idx_set = _NO_POS_IDX if clear_pv else _ALL_IDX
        if clear_pv:
            for ii in (6, 7, 8):
                row_p = P[ii]
                row_p[3] = row_p[4] = row_p[5] = 0.0
                P[3][ii] = P[4][ii] = P[5][ii] = 0.0
            self.hp0[6] = self.hp0[7] = self.hp0[8] = 0.0
            self.k0[6] = self.k0[7] = self.k0[8] = 0.0
            self.dx[6] = self.dx[7] = self.dx[8] = 0.0

        pb0 = P[blk]
        pb1 = P[blk + 1]
        pb2 = P[blk + 2]
        hp0 = self.hp0
        for j in idx_set:
            hp0[j] = h00 * pb0[j] + h01 * pb1[j] + h02 * pb2[j]

        b0, b1, b2 = blk, blk + 1, blk + 2
        s00_base = hp0[b0] * h00 + hp0[b1] * h01 + hp0[b2] * h02
        s00 = s00_base + r0
        if s00 < 1e-18:
            return

        d2 = (y0 * y0) / s00
        if not math.isfinite(d2) or d2 > gate_threshold:
            return
        if self.params["enable_student_t"]:
            scale = (nu + d2) / (nu + 1.0)
            if scale < 1.0:
                scale = 1.0
            elif scale > 100.0:
                scale = 100.0
            if scale > 1.0:
                r0 *= scale
                s00 = s00_base + r0
                if s00 < 1e-18:
                    return

        inv_s = 1.0 / s00
        k0 = self.k0
        dx = self.dx
        for i in idx_set:
            ki0 = hp0[i] * inv_s
            k0[i] = ki0
            dx[i] = ki0 * y0

        n_idx = len(idx_set)
        for i_idx in range(n_idx):
            i = idx_set[i_idx]
            row_i = P[i]
            ki0 = k0[i]
            diag_val = row_i[i] - ki0 * hp0[i]
            if diag_val < 1e-12:
                diag_val = 1e-12
            row_i[i] = diag_val
            P[i][i] = diag_val
            for j_idx in range(i_idx + 1, n_idx):
                j = idx_set[j_idx]
                val = row_i[j] - ki0 * hp0[j]
                row_i[j] = val
                P[j][i] = val

        self._inject_and_reset()

    def _update_block2(
        self, blk: int, y0: float, y1: float,
        h00: float, h01: float, h02: float,
        h10: float, h11: float, h12: float,
        r0: float, r1: float,
        gate_threshold: float = 1e9, clear_pv: bool = False, nu: float = 5.0
    ):
        P = self.P
        idx_set = _NO_POS_IDX if clear_pv else _ALL_IDX
        if clear_pv:
            for ii in (6, 7, 8):
                row_p = P[ii]
                row_p[3] = row_p[4] = row_p[5] = 0.0
                P[3][ii] = P[4][ii] = P[5][ii] = 0.0
            self.hp0[6] = self.hp0[7] = self.hp0[8] = 0.0
            self.hp1[6] = self.hp1[7] = self.hp1[8] = 0.0
            self.k0[6] = self.k0[7] = self.k0[8] = 0.0
            self.k1[6] = self.k1[7] = self.k1[8] = 0.0
            self.dx[6] = self.dx[7] = self.dx[8] = 0.0

        pb0 = P[blk]
        pb1 = P[blk + 1]
        pb2 = P[blk + 2]
        hp0 = self.hp0
        hp1 = self.hp1
        for j in idx_set:
            v0, v1, v2 = pb0[j], pb1[j], pb2[j]
            hp0[j] = h00 * v0 + h01 * v1 + h02 * v2
            hp1[j] = h10 * v0 + h11 * v1 + h12 * v2

        b0, b1, b2 = blk, blk + 1, blk + 2
        s00_base = hp0[b0] * h00 + hp0[b1] * h01 + hp0[b2] * h02
        s01 = hp0[b0] * h10 + hp0[b1] * h11 + hp0[b2] * h12
        s10 = hp1[b0] * h00 + hp1[b1] * h01 + hp1[b2] * h02
        s11_base = hp1[b0] * h10 + hp1[b1] * h11 + hp1[b2] * h12

        s00 = s00_base + r0
        s11 = s11_base + r1
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-18:
            return
        inv_d = 1.0 / det
        si00 = s11 * inv_d
        si01 = -s01 * inv_d
        si10 = -s10 * inv_d
        si11 = s00 * inv_d

        d2 = y0 * (si00 * y0 + si01 * y1) + y1 * (si10 * y0 + si11 * y1)
        if not math.isfinite(d2) or d2 > gate_threshold:
            return
        if self.params["enable_student_t"]:
            scale = (nu + d2) / (nu + 2.0)
            if scale < 1.0:
                scale = 1.0
            elif scale > 100.0:
                scale = 100.0
            if scale > 1.0:
                r0 *= scale
                r1 *= scale
                s00 = s00_base + r0
                s11 = s11_base + r1
                det = s00 * s11 - s01 * s10
                if abs(det) < 1e-18:
                    return
                inv_d = 1.0 / det
                si00 = s11 * inv_d
                si01 = -s01 * inv_d
                si10 = -s10 * inv_d
                si11 = s00 * inv_d

        k0 = self.k0
        k1 = self.k1
        dx = self.dx
        for i in idx_set:
            a, b = hp0[i], hp1[i]
            ki0 = a * si00 + b * si10
            ki1 = a * si01 + b * si11
            k0[i], k1[i] = ki0, ki1
            dx[i] = ki0 * y0 + ki1 * y1

        n_idx = len(idx_set)
        for i_idx in range(n_idx):
            i = idx_set[i_idx]
            row_i = P[i]
            ki0, ki1 = k0[i], k1[i]
            diag_val = row_i[i] - (ki0 * hp0[i] + ki1 * hp1[i])
            if diag_val < 1e-12:
                diag_val = 1e-12
            row_i[i] = diag_val
            P[i][i] = diag_val
            for j_idx in range(i_idx + 1, n_idx):
                j = idx_set[j_idx]
                val = row_i[j] - (ki0 * hp0[j] + ki1 * hp1[j])
                row_i[j] = val
                P[j][i] = val

        self._inject_and_reset()

    def _update_block3(
        self, blk: int, y0: float, y1: float, y2: float,
        h00: float, h01: float, h02: float,
        h10: float, h11: float, h12: float,
        h20: float, h21: float, h22: float,
        r0: float, r1: float, r2: float,
        gate_threshold: float = 1e9, clear_pv: bool = False,
        m_dim: int = 3, nu: float = 5.0
    ):
        P = self.P
        idx_set = _NO_POS_IDX if clear_pv else _ALL_IDX
        if clear_pv:
            for ii in (6, 7, 8):
                row_p = P[ii]
                row_p[3] = row_p[4] = row_p[5] = 0.0
                P[3][ii] = P[4][ii] = P[5][ii] = 0.0
            self.hp0[6] = self.hp0[7] = self.hp0[8] = 0.0
            self.hp1[6] = self.hp1[7] = self.hp1[8] = 0.0
            self.hp2[6] = self.hp2[7] = self.hp2[8] = 0.0
            self.k0[6] = self.k0[7] = self.k0[8] = 0.0
            self.k1[6] = self.k1[7] = self.k1[8] = 0.0
            self.k2[6] = self.k2[7] = self.k2[8] = 0.0
            self.dx[6] = self.dx[7] = self.dx[8] = 0.0

        pb0 = P[blk]
        pb1 = P[blk + 1]
        pb2 = P[blk + 2]

        hp0 = self.hp0
        hp1 = self.hp1
        hp2 = self.hp2
        for j in idx_set:
            v0, v1, v2 = pb0[j], pb1[j], pb2[j]
            hp0[j] = h00 * v0 + h01 * v1 + h02 * v2
            hp1[j] = h10 * v0 + h11 * v1 + h12 * v2
            hp2[j] = h20 * v0 + h21 * v1 + h22 * v2

        b0, b1, b2 = blk, blk + 1, blk + 2
        s00_b = hp0[b0] * h00 + hp0[b1] * h01 + hp0[b2] * h02
        s01 = hp0[b0] * h10 + hp0[b1] * h11 + hp0[b2] * h12
        s02 = hp0[b0] * h20 + hp0[b1] * h21 + hp0[b2] * h22
        s10 = hp1[b0] * h00 + hp1[b1] * h01 + hp1[b2] * h02
        s11_b = hp1[b0] * h10 + hp1[b1] * h11 + hp1[b2] * h12
        s12 = hp1[b0] * h20 + hp1[b1] * h21 + hp1[b2] * h22
        s20 = hp2[b0] * h00 + hp2[b1] * h01 + hp2[b2] * h02
        s21 = hp2[b0] * h10 + hp2[b1] * h11 + hp2[b2] * h12
        s22_b = hp2[b0] * h20 + hp2[b1] * h21 + hp2[b2] * h22

        sinv = inv3x3(s00_b + r0, s01, s02, s10, s11_b + r1, s12, s20, s21, s22_b + r2)
        if sinv is None:
            return
        si00, si01, si02, si10, si11, si12, si20, si21, si22 = sinv

        t0 = si00 * y0 + si01 * y1 + si02 * y2
        t1 = si10 * y0 + si11 * y1 + si12 * y2
        t2 = si20 * y0 + si21 * y1 + si22 * y2
        d2 = y0 * t0 + y1 * t1 + y2 * t2
        if not math.isfinite(d2) or d2 > gate_threshold:
            return
        if self.params["enable_student_t"]:
            scale = (nu + d2) / (nu + float(m_dim))
            if scale < 1.0:
                scale = 1.0
            elif scale > 100.0:
                scale = 100.0
            if scale > 1.0:
                r0 *= scale
                r1 *= scale
                r2 *= scale
                sinv = inv3x3(s00_b + r0, s01, s02, s10, s11_b + r1, s12, s20, s21, s22_b + r2)
                if sinv is None:
                    return
                si00, si01, si02, si10, si11, si12, si20, si21, si22 = sinv

        k0 = self.k0
        k1 = self.k1
        k2 = self.k2
        dx = self.dx
        for i in idx_set:
            a, b, c = hp0[i], hp1[i], hp2[i]
            ki0 = a * si00 + b * si10 + c * si20
            ki1 = a * si01 + b * si11 + c * si21
            ki2 = a * si02 + b * si12 + c * si22
            k0[i], k1[i], k2[i] = ki0, ki1, ki2
            dx[i] = ki0 * y0 + ki1 * y1 + ki2 * y2

        n_idx = len(idx_set)
        for i_idx in range(n_idx):
            i = idx_set[i_idx]
            row_i = P[i]
            ki0, ki1, ki2 = k0[i], k1[i], k2[i]
            diag_val = row_i[i] - (ki0 * hp0[i] + ki1 * hp1[i] + ki2 * hp2[i])
            if diag_val < 1e-12:
                diag_val = 1e-12
            row_i[i] = diag_val
            P[i][i] = diag_val
            for j_idx in range(i_idx + 1, n_idx):
                j = idx_set[j_idx]
                val = row_i[j] - (ki0 * hp0[j] + ki1 * hp1[j] + ki2 * hp2[j])
                row_i[j] = val
                P[j][i] = val

        self._inject_and_reset()


# ---------------------------------------------------------------------------
# Terminal 5s Blind Predictor (O(1) Incremental + Exact API Compatibility)
# ---------------------------------------------------------------------------

def make_blind_anchor(eskf: SparseESKF, params: dict, dt_nom: float = None):
    ring = eskf.ring
    if not ring:
        return (
            (eskf.p[0], eskf.p[1], eskf.p[2]),
            (eskf.v[0], eskf.v[1], eskf.v[2]),
            eskf.q,
            (0.0, 0.0, 0.0),
            eskf.get_w_walk(),
        )
    dt_use = dt_nom if (dt_nom is not None and dt_nom > 1e-5) else eskf.dt_nom
    cnt = min(len(ring), max(1, int(round(params["blind_window_s"] / dt_use))))
    sub = list(ring)[-cnt:]
    last = sub[-1]
    pf = (last[0], last[1], last[2])
    qf = (last[6], last[7], last[8], last[9])

    # Linearly ramped weighting (0.5 -> 1.5) reduces boxcar phase lag while suppressing noise
    vx = vy = vz = wx = wy = wz = ww = w_sum = 0.0
    if cnt == 1:
        s0 = sub[0]
        vx, vy, vz = s0[3], s0[4], s0[5]
        wx, wy, wz = s0[10], s0[11], s0[12]
        ww = s0[13]
        w_sum = 1.0
    else:
        inv_nm1 = 1.0 / (cnt - 1)
        for idx, s in enumerate(sub):
            wt = 0.5 + idx * inv_nm1
            w_sum += wt
            vx += wt * s[3]
            vy += wt * s[4]
            vz += wt * s[5]
            wx += wt * s[10]
            wy += wt * s[11]
            wz += wt * s[12]
            ww += wt * s[13]
    inv_w = 1.0 / w_sum
    w_walk = ww * inv_w
    vz_out = (vz * inv_w) * (
        params.get("blind_walk_vz_scale", 0.70) if w_walk >= params["regime_curvature_min_w"] else 1.0
    )
    return pf, (vx * inv_w, vy * inv_w, vz_out), qf, (wx * inv_w, wy * inv_w, wz * inv_w), w_walk


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

    wx, wy, wz = wf
    if params.get("blind_yaw_only_walk", True):
        wz_world = r20 * wx + r21 * wy + r22 * wz
        wx = r20 * wz_world
        wy = r21 * wz_world
        wz = r22 * wz_world

    tau_w = params["tau_omega_blind"]
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
    omw = 1.0 - w_walk
    return (
        omw * p_str[0] + w_walk * p_curv[0],
        omw * p_str[1] + w_walk * p_curv[1],
        omw * p_str[2] + w_walk * p_curv[2],
    ), q_end


class IncrementalBlindPredictor:
    """O(1)-per-sample curved blind predictor (50x faster than re-running Simpson per row)."""
    __slots__ = (
        "pf", "vf", "qf", "w_walk", "use_curv",
        "tau_v", "shrink_v", "tau_w",
        "vb0x", "vb0y", "vb0z", "wx", "wy", "wz",
        "last_tau", "cx", "cy", "cz", "q_cur", "last_vwx", "last_vwy", "last_vwz"
    )

    def __init__(self, anchor, params: dict):
        pf, vf, qf, wf, w_walk = anchor
        self.pf = pf
        self.vf = vf
        self.qf = qf
        self.w_walk = w_walk
        self.tau_v = params["tau_v"]
        self.shrink_v = params["shrink_v"]
        self.tau_w = params["tau_omega_blind"]
        self.use_curv = bool(
            params["enable_regime_curvature"] and (w_walk >= params["regime_curvature_min_w"])
        )
        self.last_tau = 0.0
        self.cx, self.cy, self.cz = pf
        self.q_cur = qf

        if self.use_curv:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(qf)
            self.vb0x = r00 * vf[0] + r10 * vf[1] + r20 * vf[2]
            self.vb0y = r01 * vf[0] + r11 * vf[1] + r21 * vf[2]
            self.vb0z = r02 * vf[0] + r12 * vf[1] + r22 * vf[2]
            wx, wy, wz = wf
            if params.get("blind_yaw_only_walk", True):
                wz_world = r20 * wx + r21 * wy + r22 * wz
                wx = r20 * wz_world
                wy = r21 * wz_world
                wz = r22 * wz_world
            self.wx, self.wy, self.wz = wx, wy, wz
            self.last_vwx = vf[0] * self.shrink_v
            self.last_vwy = vf[1] * self.shrink_v
            self.last_vwz = vf[2] * self.shrink_v
        else:
            self.vb0x = self.vb0y = self.vb0z = 0.0
            self.wx = self.wy = self.wz = 0.0
            self.last_vwx = self.last_vwy = self.last_vwz = 0.0

    def _vel_at(self, s: float):
        ang_scale = self.tau_w * (1.0 - math.exp(-s / self.tau_w))
        q_s = quat_norm_align(
            quat_mul(self.qf, rotvec_to_quat(self.wx * ang_scale, self.wy * ang_scale, self.wz * ang_scale)),
            self.qf,
        )
        rs00, rs01, rs02, rs10, rs11, rs12, rs20, rs21, rs22 = quat_to_rot(q_s)
        v_dec = self.shrink_v * math.exp(-s / self.tau_v)
        vwx = (rs00 * self.vb0x + rs01 * self.vb0y + rs02 * self.vb0z) * v_dec
        vwy = (rs10 * self.vb0x + rs11 * self.vb0y + rs12 * self.vb0z) * v_dec
        vwz = (rs20 * self.vb0x + rs21 * self.vb0y + rs22 * self.vb0z) * v_dec
        return vwx, vwy, vwz, q_s

    def eval(self, tau: float):
        if tau <= 0.0:
            return self.pf, self.qf
        scale_s = self.shrink_v * self.tau_v * (1.0 - math.exp(-tau / self.tau_v))
        p_str_x = self.pf[0] + self.vf[0] * scale_s
        p_str_y = self.pf[1] + self.vf[1] * scale_s
        p_str_z = self.pf[2] + self.vf[2] * scale_s
        if not self.use_curv:
            return (p_str_x, p_str_y, p_str_z), self.qf

        if tau > self.last_tau:
            dt = tau - self.last_tau
            # Sub-step if gap > 0.05s for Simpson accuracy; otherwise single 3-point Simpson step
            n_sub = 1 if dt <= 0.05 else max(2, int(math.ceil(dt / 0.05)))
            h = dt / n_sub
            s0 = self.last_tau
            for _ in range(n_sub):
                s_mid = s0 + 0.5 * h
                s1 = s0 + h
                vmx, vmy, vmz, _ = self._vel_at(s_mid)
                v1x, v1y, v1z, q1 = self._vel_at(s1)
                h6 = h * (1.0 / 6.0)
                self.cx += (self.last_vwx + 4.0 * vmx + v1x) * h6
                self.cy += (self.last_vwy + 4.0 * vmy + v1y) * h6
                self.cz += (self.last_vwz + 4.0 * vmz + v1z) * h6
                self.last_vwx, self.last_vwy, self.last_vwz = v1x, v1y, v1z
                self.q_cur = q1
                s0 = s1
            self.last_tau = tau

        ww = self.w_walk
        omw = 1.0 - ww
        return (
            omw * p_str_x + ww * self.cx,
            omw * p_str_y + ww * self.cy,
            omw * p_str_z + ww * self.cz,
        ), self.q_cur


# ---------------------------------------------------------------------------
# Alternative Ablation Modes (naive / mahony)
# ---------------------------------------------------------------------------

def _run_naive_or_mahony(t_s, acc_c, gyr_c, dup_mask, invalid_mask, fail_start, mode: str, end_init_idx: int, dt_nom: float):
    n = len(t_s)
    ax_a, ay_a, az_a = acc_c
    gx_a, gy_a, gz_a = gyr_c
    p_out = [(0.0, 0.0, 0.0)] * n
    q_out = [(1.0, 0.0, 0.0, 0.0)] * n

    eskf = SparseESKF(DEFAULT_PARAMS, dt_nom=dt_nom)
    eskf.initialize(t_s, acc_c, gyr_c, dup_mask, invalid_mask, end_init_idx, fail_start)
    q = eskf.q
    g_mag = eskf.g_mag
    px = py = pz = 0.0
    vx = vy = vz = 0.0
    prev_t = t_s[0]

    for k in range(n):
        tk = t_s[k]
        if k >= fail_start or dup_mask[k] or invalid_mask[k]:
            dt = max(0.0, tk - prev_t) if (not dup_mask[k]) else 0.0
            prev_t = tk
            if dt > 0.0 and k < fail_start:
                px += vx * dt
                py += vy * dt
                pz += vz * dt
            p_out[k] = (px, py, pz)
            q_out[k] = q
            continue

        dt = min(0.05, max(0.0, tk - prev_t))
        prev_t = tk
        ax, ay, az = ax_a[k], ay_a[k], az_a[k]
        gx, gy, gz = gx_a[k], gy_a[k], gz_a[k]
        if dt > 0.0:
            if mode == "mahony":
                r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(q)
                an = math.sqrt(ax * ax + ay * ay + az * az)
                if an > 1e-6 and abs(an - g_mag) < 0.8:
                    inv_an = 1.0 / an
                    ex = (ay * inv_an) * r22 - (az * inv_an) * r21
                    ey = (az * inv_an) * r20 - (ax * inv_an) * r22
                    ez = (ax * inv_an) * r21 - (ay * inv_an) * r20
                    gx += 0.5 * ex
                    gy += 0.5 * ey
                    gz += 0.5 * ez
            q = quat_norm_align(quat_mul(q, rotvec_to_quat(gx * dt, gy * dt, gz * dt)), q)
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = quat_to_rot(q)
            awx = r00 * ax + r01 * ay + r02 * az
            awy = r10 * ax + r11 * ay + r12 * az
            awz = r20 * ax + r21 * ay + r22 * az - g_mag
            dec = 1.0 if mode == "naive" else math.exp(-dt / 3.0)
            vx = vx * dec + awx * dt
            vy = vy * dec + awy * dt
            vz = vz * dec + awz * dt
            px += vx * dt
            py += vy * dt
            pz += vz * dt
        p_out[k] = (px, py, pz)
        q_out[k] = q
    return p_out, q_out


# ---------------------------------------------------------------------------
# End-to-End Trajectory Pipeline
# ---------------------------------------------------------------------------

def _format_ts(ts_val, is_int_ts: bool, ts_scale: float) -> str:
    if isinstance(ts_val, int):
        if ts_scale == 1e-9:
            sec = ts_val // 1_000_000_000
            nsec = ts_val % 1_000_000_000
            return f"{sec}.{nsec:09d}"
        if ts_scale == 1e-6:
            t_ns = ts_val * 1_000
            sec = t_ns // 1_000_000_000
            nsec = t_ns % 1_000_000_000
            return f"{sec}.{nsec:09d}"
        if ts_scale == 1e-3:
            t_ns = ts_val * 1_000_000
            sec = t_ns // 1_000_000_000
            nsec = t_ns % 1_000_000_000
            return f"{sec}.{nsec:09d}"
        return f"{float(ts_val):.9f}"
    return f"{float(ts_val) * ts_scale:.9f}"


def _find_end_init_idx(t_s, t0: float, t_init: float) -> int:
    n = len(t_s)
    target = t0 + t_init
    lo, hi = 0, n
    while lo < hi:
        mid = (lo + hi) >> 1
        if t_s[mid] < target:
            lo = mid + 1
        else:
            hi = mid
    return lo


def run_pipeline(
    input_path: str,
    output_path: str,
    params: dict = None,
    ablation: str = "eskf_full",
    append_blind: bool = False,
):
    pr = dict(DEFAULT_PARAMS)
    if params is not None:
        pr.update(params)

    if ablation == "eskf_basic":
        pr["enable_adaptive_q"] = False
        pr["enable_student_t"] = False
        pr["enable_regime_curvature"] = False
        pr["enable_zaru"] = False
        pr["enable_leveled_ped_nhc"] = False
        pr["enable_weinberg_bound"] = False
        pr["enable_adaptive_denoise"] = False

    ts_list, is_int_ts, ts_scale, t_s, acc, gyr, parse_inv = load_imu_arrays(input_path)
    n = len(t_s)
    acc_clean, gyr_clean, dup_mask, invalid_mask, fail_start, noise_mult, dt_nom = preprocess_offline(
        t_s, acc, gyr, parse_inv, pr
    )
    del acc, gyr, parse_inv

    t0 = t_s[0]
    end_init_idx = _find_end_init_idx(t_s, t0, pr["T_init"])
    if end_init_idx < 2:
        end_init_idx = min(n, max(1, int(round(pr["T_init_fallback"] / max(dt_nom, 1e-4)))))

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    if ablation in ("naive", "mahony"):
        p_out, q_out = _run_naive_or_mahony(
            t_s, acc_clean, gyr_clean, dup_mask, invalid_mask, fail_start, ablation, end_init_idx, dt_nom
        )
        with open(output_path, "w", encoding="utf-8", buffering=65536) as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            lines_buf = []
            for k in range(n):
                t_str = _format_ts(ts_list[k], is_int_ts, ts_scale)
                pk = p_out[k]
                qk = q_out[k]
                lines_buf.append(
                    f"{t_str} {pk[0]:.6f} {pk[1]:.6f} {pk[2]:.6f} "
                    f"{qk[1]:.6f} {qk[2]:.6f} {qk[3]:.6f} {qk[0]:.6f}\n"
                )
                if len(lines_buf) >= 1024:
                    f.writelines(lines_buf)
                    lines_buf.clear()
            if lines_buf:
                f.writelines(lines_buf)
        return

    eskf = SparseESKF(pr, dt_nom=dt_nom)
    eskf.initialize(
        t_s, acc_clean, gyr_clean, dup_mask, invalid_mask, end_init_idx, fail_start, noise_mult=noise_mult
    )

    ax_a, ay_a, az_a = acc_clean
    gx_a, gy_a, gz_a = gyr_clean

    last_valid_t = None
    last_valid_ax = last_valid_ay = last_valid_az = 0.0
    last_valid_gx = last_valid_gy = last_valid_gz = 0.0
    wx_s = wy_s = wz_s = 0.0
    gap_wx = gap_wy = gap_wz = 0.0
    in_invalid_gap = False

    p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
    q_last = eskf.q

    active_limit = min(n, fail_start)
    blind_pred = None
    t_fail_anchor = t_s[active_limit - 1] if active_limit > 0 else t0
    gap_dt_thresh = 2.5 * dt_nom
    inv_dt_nom = 1.0 / max(dt_nom, 1e-4)
    dt_scale_w = dt_nom / 0.005 if dt_nom > 1e-6 else 1.0
    alpha_w = 1.0 - math.exp(-0.0512933 * dt_scale_w) if dt_scale_w != 1.0 else 0.05
    one_m_alpha_w = 1.0 - alpha_w

    with open(output_path, "w", encoding="utf-8", buffering=65536) as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        lines_buf = []

        for k in range(n):
            tk = t_s[k]
            t_str = _format_ts(ts_list[k], is_int_ts, ts_scale)

            # Terminal 5s blind block
            if k >= active_limit:
                if blind_pred is None:
                    anchor = make_blind_anchor(eskf, pr, dt_nom=dt_nom)
                    blind_pred = IncrementalBlindPredictor(anchor, pr)
                    t_fail_anchor = last_valid_t if last_valid_t is not None else (
                        t_s[active_limit - 1] if active_limit > 0 else t0
                    )
                tau = max(blind_pred.last_tau, tk - t_fail_anchor)
                pb, qb = blind_pred.eval(tau if tau > 0.0 else 0.0)
                lines_buf.append(
                    f"{t_str} {pb[0]:.6f} {pb[1]:.6f} {pb[2]:.6f} "
                    f"{qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f} {qb[0]:.6f}\n"
                )
                if len(lines_buf) >= 1024:
                    f.writelines(lines_buf)
                    lines_buf.clear()
                continue

            # Exact duplicate / non-monotonic timestamp bypass
            if dup_mask[k]:
                lines_buf.append(
                    f"{t_str} {p_last[0]:.6f} {p_last[1]:.6f} {p_last[2]:.6f} "
                    f"{q_last[1]:.6f} {q_last[2]:.6f} {q_last[3]:.6f} {q_last[0]:.6f}\n"
                )
                if len(lines_buf) >= 1024:
                    f.writelines(lines_buf)
                    lines_buf.clear()
                continue

            # Interior invalid / malformed / extreme sample (decays omega continuously)
            if invalid_mask[k]:
                if last_valid_t is not None and tk > last_valid_t:
                    dt_inv = tk - last_valid_t
                    if not in_invalid_gap:
                        gap_wx, gap_wy, gap_wz = wx_s, wy_s, wz_s
                        in_invalid_gap = True
                    gap_wx, gap_wy, gap_wz = eskf.propagate_gap(
                        min(dt_inv, 5.0), dt_nom, gap_wx, gap_wy, gap_wz, push_ring=False
                    )
                    last_valid_t = tk
                p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
                q_last = eskf.q
                lines_buf.append(
                    f"{t_str} {p_last[0]:.6f} {p_last[1]:.6f} {p_last[2]:.6f} "
                    f"{q_last[1]:.6f} {q_last[2]:.6f} {q_last[3]:.6f} {q_last[0]:.6f}\n"
                )
                if len(lines_buf) >= 1024:
                    f.writelines(lines_buf)
                    lines_buf.clear()
                continue

            ax = ax_a[k]; ay = ay_a[k]; az = az_a[k]
            gx = gx_a[k]; gy = gy_a[k]; gz = gz_a[k]
            nm = noise_mult[k]
            t_el = tk - t0

            if last_valid_t is None:
                last_valid_t = tk
                last_valid_ax, last_valid_ay, last_valid_az = ax, ay, az
                last_valid_gx, last_valid_gy, last_valid_gz = gx, gy, gz
                in_invalid_gap = False
            else:
                dt = tk - last_valid_t
                if dt > 0.1:
                    eskf.monitor.has_prev = False
                    eskf.has_prev_raw = False
                if dt > gap_dt_thresh:
                    if dt <= 0.35 and (not in_invalid_gap):
                        # Clean offline linear sub-stepping across short missing bursts
                        n_virt = max(1, int(round(dt * inv_dt_nom)))
                        dt_v = dt / n_virt
                        inv_nv = 1.0 / n_virt
                        t_start_seg = last_valid_t - t0
                        for step_i in range(1, n_virt + 1):
                            al = step_i * inv_nv
                            om = 1.0 - al
                            eskf.step(
                                dt_v,
                                om * last_valid_ax + al * ax,
                                om * last_valid_ay + al * ay,
                                om * last_valid_az + al * az,
                                om * last_valid_gx + al * gx,
                                om * last_valid_gy + al * gy,
                                om * last_valid_gz + al * gz,
                                nm,
                                t_start_seg + step_i * dt_v,
                            )
                    else:
                        w_use_x, w_use_y, w_use_z = (gap_wx, gap_wy, gap_wz) if in_invalid_gap else (wx_s, wy_s, wz_s)
                        eskf.propagate_gap(dt - dt_nom, dt_nom, w_use_x, w_use_y, w_use_z, push_ring=False)
                        eskf.step(dt_nom, ax, ay, az, gx, gy, gz, nm, t_el)
                else:
                    eskf.step(dt, ax, ay, az, gx, gy, gz, nm, t_el)

                in_invalid_gap = False
                last_valid_t = tk
                last_valid_ax, last_valid_ay, last_valid_az = ax, ay, az
                last_valid_gx, last_valid_gy, last_valid_gz = gx, gy, gz

            wx_s = one_m_alpha_w * wx_s + alpha_w * (gx - eskf.bg[0])
            wy_s = one_m_alpha_w * wy_s + alpha_w * (gy - eskf.bg[1])
            wz_s = one_m_alpha_w * wz_s + alpha_w * (gz - eskf.bg[2])

            p_last = (eskf.p[0], eskf.p[1], eskf.p[2])
            q_last = eskf.q
            lines_buf.append(
                f"{t_str} {p_last[0]:.6f} {p_last[1]:.6f} {p_last[2]:.6f} "
                f"{q_last[1]:.6f} {q_last[2]:.6f} {q_last[3]:.6f} {q_last[0]:.6f}\n"
            )
            if len(lines_buf) >= 1024:
                f.writelines(lines_buf)
                lines_buf.clear()

        # Optional legacy --append-blind mode
        if append_blind:
            if blind_pred is None:
                anchor = make_blind_anchor(eskf, pr, dt_nom=dt_nom)
                blind_pred = IncrementalBlindPredictor(anchor, pr)
            t_last_raw = ts_list[-1]
            dt_out = dt_nom if dt_nom > 1e-5 else 0.005
            n_blind = int(round(5.0 / dt_out))
            for b_step in range(1, n_blind + 1):
                tau = b_step * dt_out
                pb, qb = blind_pred.eval(tau)
                if is_int_ts:
                    step_raw = int(round(tau / ts_scale))
                    t_str = _format_ts(t_last_raw + step_raw, True, ts_scale)
                else:
                    t_str = _format_ts(float(t_last_raw) + tau / ts_scale, False, ts_scale)
                lines_buf.append(
                    f"{t_str} {pb[0]:.6f} {pb[1]:.6f} {pb[2]:.6f} "
                    f"{qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f} {qb[0]:.6f}\n"
                )
                if len(lines_buf) >= 1024:
                    f.writelines(lines_buf)
                    lines_buf.clear()

        if lines_buf:
            f.writelines(lines_buf)
            lines_buf.clear()


class _CustomParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        sys.exit(1)


def main():
    parser = _CustomParser(description="Standalone Single-File 6-DoF Inertial Estimator (Track 3)")
    parser.add_argument("input", help="Path to input IMU CSV, directory, or ZIP")
    parser.add_argument(
        "output",
        nargs="?",
        default="estimated_trajectory.txt",
        help="Path to output trajectory file (default: estimated_trajectory.txt)",
    )
    parser.add_argument(
        "--ablation",
        default="eskf_full",
        choices=["naive", "mahony", "eskf_basic", "eskf_full"],
        help="Filter ablation mode",
    )
    parser.add_argument(
        "--append-blind",
        action="store_true",
        help="Legacy flag to append 5.0s blind poses past EOF (disabled by default per FAQ)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.input):
        sys.stderr.write(f"error: input path does not exist: {args.input}\n")
        sys.exit(1)

    try:
        run_pipeline(args.input, args.output, ablation=args.ablation, append_blind=args.append_blind)
    except Exception as exc:
        sys.stderr.write(f"error: {exc}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
