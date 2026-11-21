#!/usr/bin/env python3
"""
degrade_hard.py - hard, deterministic IMU degradation generator for Track 3
(Path Finder: Trajectory Estimation Under Sensor Degradation).

The organisers' degradation script is not available yet, so this is OUR OWN
independent surrogate. It is deliberately harsher than what the deck describes
(progressive noise, bias drift, dropouts up to 2 s, unknown gravity alignment,
complete IMU failure followed by a 5 s blind segment). Label every result that
uses it "independent surrogate", never "official".

Fault model (applied in this physical order)
  1. optional random sensor mounting rotation (unknown gravity alignment)
  2. scale-factor and cross-axis errors (static + drifting)
  3. turn-on bias, linear bias ramp, bias random walk
  4. narrow-band vibration tones (motor-like, grows over time)
  5. progressive white / heavy-tailed noise, with jamming bursts (noise x mult + bias step)
  6. impulsive spikes
  7. saturation and quantisation
  8. per-axis channel faults (stuck / zero / garbage windows)
  9. row-level faults: dropout windows (missing, zeros, hold, nan, empty, garbage),
     random single-frame drops, optional timing faults
 10. complete IMU failure for the last N seconds (the blind segment), in any encoding

Severity presets: mild < medium < severe < extreme < hell.
  mild / medium  ~ what an organiser script most plausibly does
  severe         ~ a hard but realistic test (recommended main benchmark)
  extreme / hell ~ stress tests. Do NOT tune only on hell; it will over-regularise.

Outputs (per run), in <out>/
  imu0/data.csv              EuRoC format, same header as the deck
  degradation_manifest.json  every parameter, every event, hashes, mount rotation
  row_flags.csv              out_row, orig_row, orig_timestamp_ns, flags (bit mask)
  <out>.zip (with --zip)     zipped folder containing imu0/data.csv only

Examples
  python degrade_hard.py degrade --input MH_01_easy.zip --out d/mh01_hell_s1 --preset hell --seed 1 --zip
  python degrade_hard.py batch --input data/MH_01/mav0/imu0/data.csv --out d \
         --presets mild,medium,severe,extreme,hell --seeds 1-5 --failure-encodings zeros,missing
  python degrade_hard.py adjust-gt --gt gt.tum --manifest d/x/degradation_manifest.json --out gt_adj.tum

Only numpy is required.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import io
import json
import math
import os
import sys
import zipfile

import numpy as np

VERSION = "1.0"
HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
          "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")

# row flag bits (row_flags.csv)
F_DROPOUT, F_FAILURE, F_CHANNEL, F_JAM, F_SPIKE, F_GAP_BEFORE, F_TIMING = 1, 2, 4, 8, 16, 32, 64
FLAG_LEGEND = {"dropout": 1, "failure": 2, "channel_fault": 4, "jamming": 8, "spike": 16,
               "gap_before(row follows missing rows)": 32, "timing_fault": 64}

# row encodings
M_OK, M_MISSING, M_ZEROS, M_HOLD, M_NAN, M_EMPTY, M_GARBAGE = range(7)
MODE_BY_NAME = {"missing": M_MISSING, "zeros": M_ZEROS, "hold": M_HOLD,
                "nan": M_NAN, "empty": M_EMPTY, "garbage": M_GARBAGE}
ALL_TYPES = {"missing": 3, "zeros": 2, "hold": 2, "nan": 1, "empty": 1, "garbage": 1}

BASE = dict(
    gyro_noise=(0.0, 0.0), accel_noise=(0.0, 0.0), ramp_power=1.5, heavy_tail_df=0,
    gyro_bias0=0.0, accel_bias0=0.0, gyro_bias_ramp=0.0, accel_bias_ramp=0.0,
    gyro_bias_rw=0.0, accel_bias_rw=0.0,
    scale_std=0.0, cross_std=0.0, scale_drift=0.0,
    spike_rate=0.0, spike_gyro=0.0, spike_accel=0.0,
    vib_tones=0, vib_gyro=0.0, vib_accel=0.0, vib_growth=0.0,
    jam_per_min=0.0, jam_len=(1.0, 3.0), jam_noise_mult=1.0, jam_bias_gyro=0.0, jam_bias_accel=0.0,
    gyro_clip=0.0, accel_clip=0.0, gyro_lsb=0.0, accel_lsb=0.0,
    dropouts=(0, 0), dropout_len=(0.3, 1.0), dropout_types={"missing": 1, "zeros": 1},
    dropout_in_init=False, frame_drop_rate=0.0,
    channel_faults=(0, 0), channel_len=(0.5, 1.5),
    mount="none", protect_start_s=2.0, min_gap_s=1.0,
)

PRESETS = {
    "mild": dict(
        gyro_noise=(0.001, 0.01), accel_noise=(0.01, 0.10),
        gyro_bias0=0.002, accel_bias0=0.02, gyro_bias_ramp=0.002, accel_bias_ramp=0.05,
        gyro_bias_rw=1e-4, accel_bias_rw=1e-3, scale_std=0.002, cross_std=0.001, scale_drift=0.002,
        dropouts=(1, 2), dropout_len=(0.3, 1.0), dropout_types={"missing": 1, "zeros": 1}),
    "medium": dict(
        gyro_noise=(0.002, 0.03), accel_noise=(0.02, 0.4),
        gyro_bias0=0.004, accel_bias0=0.04, gyro_bias_ramp=0.005, accel_bias_ramp=0.15,
        gyro_bias_rw=3e-4, accel_bias_rw=3e-3, scale_std=0.005, cross_std=0.003, scale_drift=0.005,
        spike_rate=0.2, spike_gyro=0.5, spike_accel=5.0,
        vib_tones=2, vib_gyro=0.02, vib_accel=0.3, vib_growth=1.0,
        jam_per_min=1.0, jam_len=(1.0, 3.0), jam_noise_mult=5.0, jam_bias_gyro=0.02, jam_bias_accel=0.3,
        dropouts=(2, 4), dropout_len=(0.3, 1.5), dropout_types={"missing": 2, "zeros": 2, "hold": 1},
        frame_drop_rate=0.003, channel_faults=(0, 1)),
    "severe": dict(
        gyro_noise=(0.004, 0.08), accel_noise=(0.05, 1.0), heavy_tail_df=4,
        gyro_bias0=0.008, accel_bias0=0.08, gyro_bias_ramp=0.01, accel_bias_ramp=0.4,
        gyro_bias_rw=1e-3, accel_bias_rw=8e-3, scale_std=0.01, cross_std=0.006, scale_drift=0.01,
        spike_rate=0.5, spike_gyro=1.0, spike_accel=10.0,
        vib_tones=3, vib_gyro=0.05, vib_accel=0.8, vib_growth=2.0,
        jam_per_min=2.0, jam_len=(1.0, 4.0), jam_noise_mult=10.0, jam_bias_gyro=0.05, jam_bias_accel=0.8,
        gyro_lsb=1e-3, accel_lsb=1e-2,
        dropouts=(3, 6), dropout_len=(0.3, 2.0),
        dropout_types={"missing": 2, "zeros": 2, "hold": 1, "nan": 1},
        frame_drop_rate=0.01, channel_faults=(1, 2)),
    "extreme": dict(
        gyro_noise=(0.008, 0.15), accel_noise=(0.1, 2.0), heavy_tail_df=3,
        gyro_bias0=0.015, accel_bias0=0.15, gyro_bias_ramp=0.02, accel_bias_ramp=0.8,
        gyro_bias_rw=2e-3, accel_bias_rw=1.5e-2, scale_std=0.02, cross_std=0.01, scale_drift=0.02,
        spike_rate=1.0, spike_gyro=2.0, spike_accel=20.0,
        vib_tones=4, vib_gyro=0.1, vib_accel=1.5, vib_growth=3.0,
        jam_per_min=3.0, jam_len=(1.0, 4.0), jam_noise_mult=20.0, jam_bias_gyro=0.1, jam_bias_accel=1.5,
        gyro_clip=6.0, accel_clip=40.0, gyro_lsb=2e-3, accel_lsb=2e-2,
        dropouts=(4, 8), dropout_len=(0.3, 2.5), dropout_types=dict(ALL_TYPES),
        frame_drop_rate=0.02, channel_faults=(2, 3)),
    "hell": dict(
        gyro_noise=(0.01, 0.30), accel_noise=(0.2, 4.0), heavy_tail_df=3,
        gyro_bias0=0.03, accel_bias0=0.3, gyro_bias_ramp=0.04, accel_bias_ramp=1.5,
        gyro_bias_rw=4e-3, accel_bias_rw=3e-2, scale_std=0.03, cross_std=0.02, scale_drift=0.04,
        spike_rate=2.0, spike_gyro=4.0, spike_accel=40.0,
        vib_tones=5, vib_gyro=0.2, vib_accel=3.0, vib_growth=4.0,
        jam_per_min=4.0, jam_len=(1.0, 5.0), jam_noise_mult=30.0, jam_bias_gyro=0.2, jam_bias_accel=3.0,
        gyro_clip=4.0, accel_clip=25.0, gyro_lsb=4e-3, accel_lsb=4e-2,
        dropouts=(6, 12), dropout_len=(0.3, 3.0), dropout_types=dict(ALL_TYPES), dropout_in_init=True,
        frame_drop_rate=0.05, channel_faults=(3, 5), mount="random"),
}


def get_params(preset):
    p = copy.deepcopy(BASE)
    p.update(copy.deepcopy(PRESETS[preset]))
    return p


# ----------------------------------------------------------------------------- math helpers
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_mat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_quat(r, p, y):
    r, p, y = math.radians(r), math.radians(p), math.radians(y)
    qx = np.array([math.cos(r / 2), math.sin(r / 2), 0, 0])
    qy = np.array([math.cos(p / 2), 0, math.sin(p / 2), 0])
    qz = np.array([math.cos(y / 2), 0, 0, math.sin(y / 2)])
    return qmul(qz, qmul(qy, qx))


def mount_quat(spec, rng):
    if spec in (None, "none"):
        return None
    if spec == "random":
        q = rng.standard_normal(4)
        return q / np.linalg.norm(q)
    if spec == "flip":
        return rpy_to_quat(180, 0, 0)
    if spec.startswith("rpy:"):
        r, p, y = [float(v) for v in spec[4:].split(",")]
        return rpy_to_quat(r, p, y)
    raise ValueError("mount must be none|random|flip|rpy:R,P,Y")


def unit_noise(rng, shape, df):
    if df and df > 2:
        return rng.standard_t(df, size=shape) / math.sqrt(df / (df - 2.0))
    return rng.standard_normal(shape)


def rand_mis(rng, s, c):
    m = np.diag(rng.normal(0.0, s, 3))
    for i in range(3):
        for j in range(3):
            if i != j:
                m[i, j] = rng.normal(0.0, c)
    return m


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def pick_weighted(rng, weights):
    names = sorted(weights)
    w = np.array([weights[k] for k in names], dtype=float)
    return names[int(rng.choice(len(names), p=w / w.sum()))]


# ----------------------------------------------------------------------------- I/O
def load_imu(path):
    """Accepts a csv, a folder (imu0/data.csv or mav0/imu0/data.csv) or a zip."""
    raw = None
    src = path
    if os.path.isdir(path):
        for cand in ("imu0/data.csv", "mav0/imu0/data.csv"):
            f = os.path.join(path, cand)
            if os.path.exists(f):
                src = f
                break
        else:
            raise FileNotFoundError("no imu0/data.csv under %s" % path)
        raw = open(src, "rb").read()
    elif path.lower().endswith(".zip"):
        with zipfile.ZipFile(path) as z:
            names = [n for n in z.namelist() if n.endswith("imu0/data.csv")]
            if not names:
                raise FileNotFoundError("no imu0/data.csv in zip")
            raw = z.read(sorted(names, key=len)[0])
    else:
        raw = open(path, "rb").read()
    text = raw.decode("utf-8-sig")
    ts, rows = [], []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) < 7:
            continue
        ts.append(int(parts[0]))
        rows.append([float(v) for v in parts[1:7]])
    return np.array(ts, dtype=np.int64), np.array(rows, dtype=np.float64), sha256_bytes(raw)


def fmt6(v):
    return ",".join("%.9f" % x for x in v)


# ----------------------------------------------------------------------------- window placement
def place_windows(rng, count, len_rng, t, lo, hi, min_gap, force_init=False, existing=None):
    wins = list(existing or [])
    placed = []
    for k in range(count):
        for _ in range(300):
            ln = rng.uniform(*len_rng)
            if force_init and k == 0 and not placed:
                s = rng.uniform(0.1, 1.0)
            else:
                if hi - ln <= lo:
                    continue
                s = rng.uniform(lo, hi - ln)
            e = s + ln
            if all(e + min_gap <= a or s >= b + min_gap for a, b in wins):
                wins.append((s, e))
                placed.append((s, e))
                break
    return placed


# ----------------------------------------------------------------------------- core
def degrade(ts, vals, P, seed, opts):
    n = len(ts)
    r = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(14)]
    (r_mount, r_mis, r_bias, r_vib, r_noise, r_spk, r_jam, r_chan, r_drop, r_fd, r_enc, r_time, r_sc, r_x) = r

    t = (ts - ts[0]) * 1e-9
    pos = np.diff(t)
    med_dt = float(np.median(pos[pos > 0])) if (pos > 0).any() else 0.005
    dts = np.diff(t, prepend=t[0] - med_dt)
    dts = np.where(dts > 0, dts, med_dt)

    # ---- failure index
    if opts.get("no_failure"):
        fail_idx = n
    elif opts.get("fail_at") is not None:
        fail_idx = int(np.searchsorted(t, opts["fail_at"]))
    else:
        fail_idx = int(np.searchsorted(t, t[-1] - opts.get("fail_from_end", 5.0)))
    fail_idx = min(max(fail_idx, 1), n)
    Tf = max(float(t[min(fail_idx, n - 1)]), 1.0)
    p_ramp = opts.get("p_ramp") or P["ramp_power"]
    ramp = np.clip(t / Tf, 0.0, 1.0) ** p_ramp

    events = []
    flags = np.zeros(n, dtype=np.int32)
    g = vals[:, 0:3].copy()
    a = vals[:, 3:6].copy()

    # 1 mounting rotation
    mq = mount_quat(opts.get("mount") or P["mount"], r_mount)
    mount_R = None
    if mq is not None:
        mount_R = quat_to_mat(mq)
        g = g @ mount_R.T
        a = a @ mount_R.T
        events.append({"type": "mount_rotation", "quat_wxyz": mq.tolist()})

    # 2 scale / cross-axis
    for name, arr, s_std, c_std, s_dr in (("gyro", g, P["scale_std"], P["cross_std"], P["scale_drift"]),
                                         ("accel", a, P["scale_std"], P["cross_std"], P["scale_drift"])):
        if s_std or c_std or s_dr:
            e0 = rand_mis(r_mis, s_std, c_std)
            e1 = rand_mis(r_mis, s_dr, c_std * 0.5)
            arr += arr @ e0.T + ramp[:, None] * (arr @ e1.T)

    # 3 bias
    for arr, b0s, rampamp, rw in ((g, P["gyro_bias0"], P["gyro_bias_ramp"], P["gyro_bias_rw"]),
                                   (a, P["accel_bias0"], P["accel_bias_ramp"], P["accel_bias_rw"])):
        b0 = r_bias.normal(0.0, b0s, 3) if b0s else np.zeros(3)
        rv = r_bias.choice([-1.0, 1.0], 3) * r_bias.uniform(0.5, 1.0, 3) * rampamp
        walk = np.cumsum(r_bias.standard_normal((n, 3)) * (rw * np.sqrt(dts))[:, None], axis=0) if rw else 0.0
        arr += b0 + ramp[:, None] * rv + walk

    # 4 vibration
    if P["vib_tones"]:
        grow = 1.0 + P["vib_growth"] * ramp[:, None]
        for _ in range(P["vib_tones"]):
            f = r_vib.uniform(25.0, 95.0)
            for arr, amp in ((g, P["vib_gyro"]), (a, P["vib_accel"])):
                ph = r_vib.uniform(0, 2 * math.pi, 3)
                am = r_vib.uniform(0.5, 1.0, 3)
                arr += amp / math.sqrt(P["vib_tones"]) * am * np.sin(2 * math.pi * f * t[:, None] + ph) * grow

    # 5 jamming bursts + progressive noise
    jam_scale = np.ones(n)
    nj = r_jam.poisson(P["jam_per_min"] * Tf / 60.0) if P["jam_per_min"] else 0
    jam_wins = place_windows(r_jam, nj, P["jam_len"], t, 0.5, Tf - 0.5, 0.5)
    for (s0, e0) in jam_wins:
        i0, i1 = np.searchsorted(t, s0), np.searchsorted(t, e0)
        jam_scale[i0:i1] = P["jam_noise_mult"]
        g[i0:i1] += r_jam.choice([-1.0, 1.0], 3) * P["jam_bias_gyro"]
        a[i0:i1] += r_jam.choice([-1.0, 1.0], 3) * P["jam_bias_accel"]
        flags[i0:i1] |= F_JAM
        events.append({"type": "jamming", "start_s": float(s0), "end_s": float(e0),
                       "noise_mult": P["jam_noise_mult"]})
    sg = P["gyro_noise"][0] + (P["gyro_noise"][1] - P["gyro_noise"][0]) * ramp
    sa = P["accel_noise"][0] + (P["accel_noise"][1] - P["accel_noise"][0]) * ramp
    g += unit_noise(r_noise, (n, 3), P["heavy_tail_df"]) * (sg * jam_scale)[:, None]
    a += unit_noise(r_noise, (n, 3), P["heavy_tail_df"]) * (sa * jam_scale)[:, None]

    # 6 spikes
    if P["spike_rate"]:
        ns = r_spk.poisson(P["spike_rate"] * Tf)
        idx = r_spk.integers(0, max(1, min(fail_idx, n)), ns)
        for i in idx:
            mask = r_spk.random(3) < 0.6
            g[i] += mask * r_spk.choice([-1.0, 1.0], 3) * P["spike_gyro"] * r_spk.uniform(0.5, 1.5)
            a[i] += mask * r_spk.choice([-1.0, 1.0], 3) * P["spike_accel"] * r_spk.uniform(0.5, 1.5)
            flags[i] |= F_SPIKE
        events.append({"type": "spikes", "count": int(ns)})

    # 7 saturation + quantisation
    if P["gyro_clip"]:
        g = np.clip(g, -P["gyro_clip"], P["gyro_clip"])
    if P["accel_clip"]:
        a = np.clip(a, -P["accel_clip"], P["accel_clip"])
    if P["gyro_lsb"]:
        g = np.round(g / P["gyro_lsb"]) * P["gyro_lsb"]
    if P["accel_lsb"]:
        a = np.round(a / P["accel_lsb"]) * P["accel_lsb"]
    X = np.hstack([g, a])

    # 8 channel faults
    nch = int(r_chan.integers(P["channel_faults"][0], P["channel_faults"][1] + 1))
    ch_wins = place_windows(r_chan, nch, P["channel_len"], t, P["protect_start_s"], max(Tf - 0.5, 1.0), 0.5)
    for (s0, e0) in ch_wins:
        i0, i1 = np.searchsorted(t, s0), np.searchsorted(t, e0)
        ax = int(r_chan.integers(0, 6))
        mode = r_chan.choice(["stuck", "zero", "garbage"])
        if mode == "stuck":
            X[i0:i1, ax] = X[max(i0 - 1, 0), ax]
        elif mode == "zero":
            X[i0:i1, ax] = 0.0
        else:
            X[i0:i1, ax] = r_chan.uniform(-1, 1, i1 - i0) * (20.0 if ax < 3 else 200.0)
        flags[i0:i1] |= F_CHANNEL
        events.append({"type": "channel_fault", "axis": ["gx", "gy", "gz", "ax", "ay", "az"][ax],
                       "mode": str(mode), "start_s": float(s0), "end_s": float(e0)})

    # 9 row-level faults
    mode = np.full(n, M_OK, dtype=np.int8)
    types = opts.get("dropout_types") or P["dropout_types"]
    nd = int(r_drop.integers(P["dropouts"][0], P["dropouts"][1] + 1))
    lo = P["protect_start_s"]
    hi = max(t[min(fail_idx, n - 1)] - 1.0, lo + 1.0)
    drop_wins = place_windows(r_drop, nd, P["dropout_len"], t, lo, hi, P["min_gap_s"],
                              force_init=P["dropout_in_init"])
    for (s0, e0) in sorted(drop_wins):
        i0, i1 = int(np.searchsorted(t, s0)), int(np.searchsorted(t, e0))
        i1 = min(i1, fail_idx)
        enc = pick_weighted(r_enc, types)
        mode[i0:i1] = MODE_BY_NAME[enc]
        flags[i0:i1] |= F_DROPOUT
        events.append({"type": "dropout", "encoding": enc, "start_s": float(t[i0]), "end_s": float(t[min(i1, n - 1)]),
                       "start_row": i0, "end_row": i1})
    if P["frame_drop_rate"]:
        fd = (r_fd.random(n) < P["frame_drop_rate"]) & (t >= lo)
        fd[fail_idx:] = False
        fd &= (mode == M_OK)
        mode[fd] = M_MISSING
        events.append({"type": "frame_drops", "count": int(fd.sum())})

    # 10 complete failure
    fail_enc = opts.get("failure_encoding", "zeros")
    if fail_idx < n:
        fm = M_MISSING if fail_enc == "truncate" else MODE_BY_NAME[fail_enc]
        mode[fail_idx:] = fm
        flags[fail_idx:] |= F_FAILURE
        events.append({"type": "failure", "encoding": fail_enc, "start_row": int(fail_idx),
                       "start_s": float(t[fail_idx]), "blind_seconds": float(t[-1] - t[fail_idx])})

    # ---- emit rows
    out = []  # dict(ts, fields, orig, flags)
    last_ok = X[0].copy()
    prev_emitted = -1
    for i in range(n):
        m = mode[i]
        if m == M_MISSING:
            continue
        if m == M_OK:
            v = X[i]
            last_ok = v
            fields = fmt6(v)
        elif m == M_ZEROS:
            fields = fmt6(np.zeros(6))
        elif m == M_HOLD:
            fields = fmt6(last_ok)
        elif m == M_NAN:
            fields = ",".join(["nan"] * 6)
        elif m == M_EMPTY:
            fields = ",".join([""] * 6)
        else:  # garbage
            v = np.concatenate([r_x.uniform(-20, 20, 3), r_x.uniform(-200, 200, 3)])
            fields = fmt6(v)
        fl = int(flags[i])
        if i - prev_emitted > 1:
            fl |= F_GAP_BEFORE
        prev_emitted = i
        out.append({"ts": int(ts[i]), "fields": fields, "orig": i, "flags": fl})

    # optional timing faults (fuzz; breaks one-to-one timestamp association, opt in only)
    if opts.get("timing_faults"):
        jit = int(opts.get("jitter_us", 500) * 1000)
        for row in out:
            row["ts"] += int(round(r_time.normal(0, jit)))
            row["flags"] |= F_TIMING
        k = 0
        while k < len(out) - 1:
            u = r_time.random()
            if u < 0.002:
                out[k], out[k + 1] = out[k + 1], out[k]
                k += 2
            elif u < 0.004:
                out.insert(k + 1, dict(out[k]))
                k += 2
            else:
                k += 1
        events.append({"type": "timing_faults", "jitter_us": opts.get("jitter_us", 500)})

    meta = {
        "rows_in": int(n), "rows_out": len(out), "failure_row": int(fail_idx) if fail_idx < n else None,
        "failure_time_s": float(t[fail_idx]) if fail_idx < n else None,
        "duration_s": float(t[-1]), "median_dt_s": med_dt,
        "mount_rotation_matrix": mount_R.tolist() if mount_R is not None else None,
        "mount_quat_wxyz": mq.tolist() if mq is not None else None,
        "end_noise_std": {"gyro": float(sg[min(fail_idx, n) - 1]), "accel": float(sa[min(fail_idx, n) - 1])},
    }
    return out, events, meta


# ----------------------------------------------------------------------------- commands
def write_run(out_dir, ts, vals, in_hash, input_path, preset, seed, opts, make_zip):
    P = get_params(preset)
    rows, events, meta = degrade(ts, vals, P, seed, opts)
    os.makedirs(os.path.join(out_dir, "imu0"), exist_ok=True)
    csv_text = HEADER + "\n" + "\n".join("%d,%s" % (r["ts"], r["fields"]) for r in rows) + "\n"
    csv_bytes = csv_text.encode()
    with open(os.path.join(out_dir, "imu0", "data.csv"), "wb") as f:
        f.write(csv_bytes)
    with open(os.path.join(out_dir, "row_flags.csv"), "w") as f:
        f.write("out_row,orig_row,orig_timestamp_ns,flags\n")
        for k, r in enumerate(rows):
            f.write("%d,%d,%d,%d\n" % (k, r["orig"], int(ts[r["orig"]]), r["flags"]))
    manifest = {
        "script": "degrade_hard.py", "version": VERSION, "preset": preset, "seed": seed,
        "options": {k: v for k, v in opts.items()}, "params": P, "flag_legend": FLAG_LEGEND,
        "input": {"path": os.path.abspath(input_path), "sha256": in_hash},
        "output": {"imu_csv_sha256": sha256_bytes(csv_bytes)},
        "meta": meta, "events": events,
        "note": "Independent surrogate degradation. NOT the organisers' script.",
    }
    with open(os.path.join(out_dir, "degradation_manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    if make_zip:
        zp = out_dir.rstrip("/\\") + ".zip"
        with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
            z.writestr("imu0/data.csv", csv_bytes)
    return manifest


def common_opts(args):
    types = None
    if getattr(args, "dropout_types", None):
        types = {k: 1 for k in args.dropout_types.split(",")}
    return dict(fail_from_end=args.fail_from_end, fail_at=args.fail_at, no_failure=args.no_failure,
                mount=args.mount, timing_faults=args.timing_faults, p_ramp=args.p_ramp,
                dropout_types=types)


def crop(ts, vals, start_s, max_s):
    t = (ts - ts[0]) * 1e-9
    m = t >= start_s
    if max_s:
        m &= t <= start_s + max_s
    return ts[m], vals[m]


def parse_seeds(s):
    if "-" in s and "," not in s:
        a, b = s.split("-")
        return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def cmd_degrade(args):
    ts, vals, h = load_imu(args.input)
    ts, vals = crop(ts, vals, args.crop_start_s, args.max_duration_s)
    opts = common_opts(args)
    opts["failure_encoding"] = args.failure_encoding
    m = write_run(args.out, ts, vals, h, args.input, args.preset, args.seed, opts, args.zip)
    print("wrote %s | rows %d -> %d | events %d | failure row %s" % (
        args.out, m["meta"]["rows_in"], m["meta"]["rows_out"], len(m["events"]), m["meta"]["failure_row"]))


def cmd_batch(args):
    ts, vals, h = load_imu(args.input)
    ts, vals = crop(ts, vals, args.crop_start_s, args.max_duration_s)
    for preset in args.presets.split(","):
        for seed in parse_seeds(args.seeds):
            for enc in args.failure_encodings.split(","):
                opts = common_opts(args)
                opts["failure_encoding"] = enc
                out = os.path.join(args.out, "%s_s%d_%s" % (preset, seed, enc))
                m = write_run(out, ts, vals, h, args.input, preset, seed, opts, args.zip)
                print("%-28s rows %d -> %d" % (os.path.basename(out), m["meta"]["rows_in"], m["meta"]["rows_out"]))


def cmd_adjust_gt(args):
    """Ground-truth orientation for a run that used a mount rotation.
    Measurements were rotated x_S' = R x_S, so R_WS' = R_WS R^T  (q' = q (x) conj(q_R))."""
    man = json.load(open(args.manifest))
    q = man["meta"].get("mount_quat_wxyz")
    lines_out = []
    for line in open(args.gt):
        s = line.strip()
        if not s or s.startswith("#"):
            lines_out.append(line.rstrip("\n"))
            continue
        v = s.split()
        if q is None:
            lines_out.append(line.rstrip("\n"))
            continue
        qx, qy, qz, qw = [float(x) for x in v[4:8]]
        qn = qmul(np.array([qw, qx, qy, qz]), qconj(np.array(q)))
        qn /= np.linalg.norm(qn)
        lines_out.append(" ".join(v[:4] + ["%.9f" % qn[1], "%.9f" % qn[2], "%.9f" % qn[3], "%.9f" % qn[0]]))
    open(args.out, "w").write("\n".join(lines_out) + "\n")
    print("wrote", args.out, "(no mount rotation in manifest, copied unchanged)" if q is None else "")


def cmd_presets(args):
    print(json.dumps({k: get_params(k) for k in PRESETS}, indent=2, default=str))


def add_common(p):
    p.add_argument("--input", required=True, help="csv, folder (imu0/data.csv or mav0/imu0/data.csv) or zip")
    p.add_argument("--out", required=True)
    p.add_argument("--fail-from-end", type=float, default=5.0, help="blind seconds at the end (default 5)")
    p.add_argument("--fail-at", type=float, default=None, help="failure time in seconds from start (overrides --fail-from-end)")
    p.add_argument("--no-failure", action="store_true")
    p.add_argument("--mount", default=None, help="none|random|flip|rpy:R,P,Y (default: preset)")
    p.add_argument("--dropout-types", default=None, help="comma list of missing,zeros,hold,nan,empty,garbage")
    p.add_argument("--timing-faults", action="store_true", help="opt-in fuzz: jitter, duplicate and swapped rows")
    p.add_argument("--p-ramp", type=float, default=None, help="progression exponent (default 1.5)")
    p.add_argument("--crop-start-s", type=float, default=0.0)
    p.add_argument("--max-duration-s", type=float, default=0.0)
    p.add_argument("--zip", action="store_true", help="also write <out>.zip containing imu0/data.csv")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("degrade", help="one degraded run")
    add_common(d)
    d.add_argument("--preset", default="severe", choices=list(PRESETS))
    d.add_argument("--seed", type=int, default=1)
    d.add_argument("--failure-encoding", default="zeros",
                   choices=["zeros", "missing", "truncate", "hold", "nan", "empty", "garbage"])
    d.set_defaults(fn=cmd_degrade)
    b = sub.add_parser("batch", help="presets x seeds x failure encodings")
    add_common(b)
    b.add_argument("--presets", default="mild,medium,severe,extreme,hell")
    b.add_argument("--seeds", default="1-5")
    b.add_argument("--failure-encodings", default="zeros,missing")
    b.set_defaults(fn=cmd_batch)
    a = sub.add_parser("adjust-gt", help="rotate GT orientation for runs with a mount rotation")
    a.add_argument("--gt", required=True, help="TUM format ground truth")
    a.add_argument("--manifest", required=True)
    a.add_argument("--out", required=True)
    a.set_defaults(fn=cmd_adjust_gt)
    pr = sub.add_parser("presets", help="print all preset parameters")
    pr.set_defaults(fn=cmd_presets)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
