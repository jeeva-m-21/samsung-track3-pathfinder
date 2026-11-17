#!/usr/bin/env python3
"""
degrade_imu.py - generate a degraded copy of an IMU recording.

Input
    CSV file with 7 columns in this order:
        timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    A header row is optional. Timestamp may be in s, ms, us or ns
    (auto-detected, or forced with --ts-unit). Acceleration may be in
    m/s^2 or g (auto-detected, or forced with --accel-unit). Gyro is
    assumed rad/s unless --gyro-deg is given.

Output
    CSV file with the same columns, same header (if present) and same
    column order as the input. Row count may differ from the input.

Degradation
    Degradation is minimal at the first timestamp and increases with time,
    reaching the configured maximum at the last timestamp of the file.
    --max-level sets the maximum, --gamma controls how quickly the ramp
    rises. The random seed is fixed inside the script (SEED), so the same
    input always gives the same output.

Usage
    python degrade_imu.py <input.csv> <output.csv>
                          [--max-level X] [--gamma Y]
                          [--ts-unit auto|s|ms|us|ns] [--accel-unit auto|ms2|g]
                          [--gyro-deg] [--meta <stats.json>]
"""

import argparse
import json
import sys

import numpy as np
import pandas as pd

G = 9.80665

SEED = 42

CONFIG = {
    "max_level": 1.0,
    "gamma": 1.7,
    "floor": 0.02,
    "protect_seconds": 2.0,
    "failure_seconds": 5.0,

    "accel_white_max": 0.20,
    "gyro_white_max": 0.015,

    "accel_bias_rw_max": 0.010,
    "gyro_bias_rw_max": 0.0006,

    "scale_rw_max": 0.0015,

    "spike_prob_max": 0.003,
    "accel_spike_mag": 6.0,
    "gyro_spike_mag": 1.0,

    "stuck_prob_max": 0.008,
    "stuck_len": (2, 10),

    "drop_prob_max": 0.06,
    "burst_prob_max": 0.0015,
    "burst_len": (5, 50),

    "dup_prob_max": 0.003,
}


def detect_header(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        first = f.readline().strip().split(",")[0]
    try:
        float(first)
        return None
    except ValueError:
        return 0


def detect_ts_scale(ts, override):
    table = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
    if override != "auto":
        return table[override], override
    dt = np.median(np.diff(ts))
    if dt < 1.0:
        return 1.0, "s"
    if dt < 1e3:
        return 1e-3, "ms"
    if dt < 1e6:
        return 1e-6, "us"
    return 1e-9, "ns"


def detect_accel_scale(acc, override):
    if override == "ms2":
        return 1.0, "m/s^2"
    if override == "g":
        return 1.0 / G, "g"
    norm = np.median(np.linalg.norm(acc, axis=1))
    if 0.5 < norm < 2.0:
        return 1.0 / G, "g"
    return 1.0, "m/s^2"


def severity(t_norm, cfg):
    return cfg["floor"] + (cfg["max_level"] - cfg["floor"]) * np.power(t_norm, cfg["gamma"])


def run_lengths(rng, n, lo_hi, sev):
    lo, hi = lo_hi
    hi_eff = np.maximum(lo, np.round(lo + (hi - lo) * sev)).astype(int)
    return np.array([rng.integers(lo, h + 1) for h in hi_eff]) if n else np.array([], int)


def degrade(df, cfg, seed, ts_unit="auto", accel_unit="auto", gyro_deg=False):
    rng = np.random.default_rng(seed)
    cols = list(df.columns)
    ts_col, acc_cols, gyr_cols = cols[0], cols[1:4], cols[4:7]

    ts = df[ts_col].to_numpy(dtype=float)
    acc = df[acc_cols].to_numpy(dtype=float)
    gyr = df[gyr_cols].to_numpy(dtype=float)
    n = len(df)

    ts_scale, ts_name = detect_ts_scale(ts, ts_unit)
    acc_scale, acc_name = detect_accel_scale(acc, accel_unit)
    gyr_scale = np.degrees(1.0) if gyro_deg else 1.0
    gyr_name = "deg/s" if gyro_deg else "rad/s"

    t_sec = (ts - ts[0]) * ts_scale
    T = t_sec[-1] if t_sec[-1] > 0 else 1.0
    dt = np.diff(t_sec, prepend=np.nan)
    dt[0] = np.nanmedian(dt[1:])
    dt = np.clip(dt, 1e-4, None)
    sqrt_dt = np.sqrt(dt)[:, None]

    sev = severity(t_sec / T, cfg)
    sev_c = sev[:, None]
    
    if T > cfg.get("failure_seconds", 0.0) + cfg["protect_seconds"]:
        failed = t_sec >= (T - cfg["failure_seconds"])
    else:
        failed = np.zeros(n, dtype=bool)
    active = ~(t_sec < cfg["protect_seconds"]) & ~failed

    acc_n = acc + rng.standard_normal((n, 3)) * cfg["accel_white_max"] * acc_scale * sev_c
    gyr_n = gyr + rng.standard_normal((n, 3)) * cfg["gyro_white_max"] * gyr_scale * sev_c

    acc_bias = np.cumsum(rng.standard_normal((n, 3)) * cfg["accel_bias_rw_max"] * acc_scale * sqrt_dt * sev_c, axis=0)
    gyr_bias = np.cumsum(rng.standard_normal((n, 3)) * cfg["gyro_bias_rw_max"] * gyr_scale * sqrt_dt * sev_c, axis=0)

    acc_sf = np.cumsum(rng.standard_normal((n, 3)) * cfg["scale_rw_max"] * sqrt_dt * sev_c, axis=0)
    gyr_sf = np.cumsum(rng.standard_normal((n, 3)) * cfg["scale_rw_max"] * sqrt_dt * sev_c, axis=0)

    acc_n = acc_n * (1.0 + acc_sf) + acc_bias
    gyr_n = gyr_n * (1.0 + gyr_sf) + gyr_bias

    spike = (rng.random(n) < cfg["spike_prob_max"] * sev) & active
    k = int(spike.sum())
    if k:
        sgn = rng.choice([-1.0, 1.0], size=(k, 3))
        mag = rng.uniform(0.5, 1.5, size=(k, 3))
        acc_n[spike] += sgn * mag * cfg["accel_spike_mag"] * acc_scale * sev[spike, None]
        gyr_n[spike] += sgn * mag * cfg["gyro_spike_mag"] * gyr_scale * sev[spike, None]

    stuck_start = (rng.random(n) < cfg["stuck_prob_max"] * sev) & active
    stuck_idx = np.flatnonzero(stuck_start)
    stuck_lens = run_lengths(rng, len(stuck_idx), cfg["stuck_len"], sev[stuck_idx])
    n_stuck = 0
    for i, L in zip(stuck_idx, stuck_lens):
        j = min(n, i + 1 + L)
        acc_n[i + 1:j] = acc_n[i]
        gyr_n[i + 1:j] = gyr_n[i]
        n_stuck += j - (i + 1)

    keep = np.ones(n, dtype=bool)
    single_drop = (rng.random(n) < cfg["drop_prob_max"] * sev) & active
    keep &= ~single_drop

    burst_start = (rng.random(n) < cfg["burst_prob_max"] * sev) & active
    burst_idx = np.flatnonzero(burst_start)
    burst_lens = run_lengths(rng, len(burst_idx), cfg["burst_len"], sev[burst_idx])
    for i, L in zip(burst_idx, burst_lens):
        keep[i:min(n, i + L)] = False
        
    keep[~active] = True
    keep[0] = keep[-1] = True

    if cfg.get("failure_seconds", 0.0) > 0:
        acc_n[failed] = 0.0
        gyr_n[failed] = 0.0

    out = pd.DataFrame({ts_col: df[ts_col].to_numpy()})
    for c, j in zip(acc_cols, range(3)):
        out[c] = acc_n[:, j]
    for c, j in zip(gyr_cols, range(3)):
        out[c] = gyr_n[:, j]
    out = out[keep].reset_index(drop=True)

    sev_kept = sev[keep]
    dup = rng.random(len(out)) < cfg["dup_prob_max"] * sev_kept
    counts = 1 + dup.astype(int)
    out = out.iloc[np.repeat(np.arange(len(out)), counts)].reset_index(drop=True)

    out[acc_cols + gyr_cols] = out[acc_cols + gyr_cols].round(6)

    stats = {
        "seed": seed,
        "rows_in": int(n),
        "rows_out": int(len(out)),
        "duration_s": float(T),
        "median_dt_s": float(np.nanmedian(dt[1:])),
        "timestamp_unit": ts_name,
        "accel_unit": acc_name,
        "gyro_unit": gyr_name,
        "severity_start": float(sev[0]),
        "severity_end": float(sev[-1]),
        "spikes": int(k),
        "stuck_samples": int(n_stuck),
        "dropped_single": int(single_drop.sum()),
        "dropped_burst_gaps": int(len(burst_idx)),
        "dropped_total": int(n - keep.sum()),
        "duplicated_rows": int(dup.sum()),
        "failure_frames": int(failed.sum()),
        "final_gyro_bias_rad_s": (gyr_bias[-1] / gyr_scale).round(5).tolist(),
        "final_accel_bias_m_s2": (acc_bias[-1] / acc_scale).round(4).tolist(),
        "config": {k2: (list(v) if isinstance(v, tuple) else v) for k2, v in cfg.items()},
    }
    return out, stats


def main():
    ap = argparse.ArgumentParser(description="Generate a degraded copy of an IMU CSV.")
    ap.add_argument("input", help="path to the clean input CSV")
    ap.add_argument("output", help="path where the degraded CSV is written")
    ap.add_argument("--max-level", type=float, default=None, help="degradation level at the last timestamp (0..1)")
    ap.add_argument("--gamma", type=float, default=None, help="ramp curvature; larger keeps the early part cleaner")
    ap.add_argument("--ts-unit", choices=["auto", "s", "ms", "us", "ns"], default="auto")
    ap.add_argument("--accel-unit", choices=["auto", "ms2", "g"], default="auto")
    ap.add_argument("--gyro-deg", action="store_true", help="gyro columns are in deg/s (default rad/s)")
    ap.add_argument("--meta", default=None, help="optional path to write run statistics as JSON (organiser use)")
    args = ap.parse_args()

    cfg = dict(CONFIG)
    if args.max_level is not None:
        cfg["max_level"] = args.max_level
    if args.gamma is not None:
        cfg["gamma"] = args.gamma

    header = detect_header(args.input)
    df = pd.read_csv(args.input, header=header)
    if df.shape[1] != 7:
        sys.exit(f"expected 7 columns, got {df.shape[1]}")
    if header is None:
        df.columns = ["timestamp", "ax", "ay", "az", "gx", "gy", "gz"]

    out, stats = degrade(df, cfg, SEED, args.ts_unit, args.accel_unit, args.gyro_deg)
    out.to_csv(args.output, index=False, header=header is not None)

    if args.meta:
        with open(args.meta, "w") as f:
            json.dump(stats, f, indent=2)

    print(f"{args.input} -> {args.output}")
    print(f"  rows_in={stats['rows_in']}  rows_out={stats['rows_out']}  duration_s={stats['duration_s']:.2f}")
    print(f"  units: timestamp={stats['timestamp_unit']}  accel={stats['accel_unit']}  gyro={stats['gyro_unit']}")
    print(f"  severity: start={stats['severity_start']:.3f}  end={stats['severity_end']:.3f}")


if __name__ == "__main__":
    main()
