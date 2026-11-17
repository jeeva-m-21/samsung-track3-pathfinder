#!/usr/bin/env python3
"""Track 03 (Path Finder) Standalone Single-File Submission.

Self-contained pure-Python 15-State Error-State Kalman Filter (ESKF)
with heavy-tailed Student-t covariance updates, adaptive dynamic noise scaling,
regime-aware velocity bounds, zero-velocity updates, dropout gap bridge,
and blind dead-reckoning predictor.

Zero external dependencies beyond Python standard library and NumPy.
Memory profile: <= 4.0 MB heap via chunked stream processing.

Usage:
    python main.py <input.csv|input.zip|dir> [output.txt]
    python main.py <input.csv> [output.txt] [--ablation naive|mahony|eskf_basic|eskf_full] [--append-blind]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import tempfile
import time
import zipfile
from dataclasses import dataclass, field
from enum import Enum, IntFlag
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, NamedTuple, Optional, TextIO, Tuple, Union

import numpy as np

# Embedded default parameters (no external params.json file required)
DEFAULT_PARAMS: Dict[str, Any] = {'frame_mode': 'gravity_aligned', 'append_blind': False, 'T_init': 1.0, 'T_init_fallback': 0.5, 'g_mag_clamp_min': 9.0, 'g_mag_clamp_max': 10.5, 'sigma_theta_0': 0.02, 'sigma_v_0': 0.05, 'sigma_p_0': 0.01, 'sigma_bg_0': 0.02, 'sigma_ba_0': 0.2, 'motion_detect_gyro_std': 0.2, 'motion_detect_acc_std': 0.5, 'motion_detect_sigma_theta_mult': 5.0, 'noise_mult_gyro': 1.0, 'noise_mult_acc': 1.0, 'walk_mult_bg': 1.0, 'walk_mult_ba': 1.0, 'enable_centripetal_comp': False, 'sigma_g_base': 0.015, 'sigma_a_base': 0.08, 'sigma_bg_base': 0.0001, 'sigma_ba_base': 0.001, 'cov_diag_floor': 1e-12, 'cov_diag_cap': 1000000.0, 'tilt_tau_a': 0.1888081177663604, 'tilt_omega_max': 2.2, 'tilt_sigma': 0.02431134926953681, 'tilt_alpha_a': 2.0, 'tilt_a_scale': 0.21426320932326784, 'tilt_alpha_omega': 8.0, 'tilt_omega_scale': 2.2, 'zupt_window_s': 0.25, 'zupt_gyro_mean': 0.02, 'zupt_acc_dev': 0.15, 'zupt_sigma': 0.01427568922533429, 'zupt_min_quality': 0.3, 'monitor_alpha': 0.01, 'kappa_max': 50.0, 'sigma_bgm': 0.005, 'invalid_gyro_max': 35.0, 'invalid_acc_max': 160.0, 'gap_substep_max': 800, 'tau_gap': 2.0, 'kappa_gap': 5.0, 'recover_steps': 100, 'T_fail': 1.0, 'smooth_omega_tau': 0.25, 'ring_buffer_size': 256, 'tau_v': 3.367753801188697, 'tau_omega': 1.5, 'shrink_v': 0.8573318952578088, 'stationary_v_thresh': 0.026318142128718358, 'enable_regime_curvature': True, 'regime_curvature_min_w': 0.5, 'tau_omega_blind': 1.8265880306839262, 'enable_iir_filter': False, 'iir_cutoff_hz': 14.0, 'enable_velocity_bounding': True, 'max_velocity_mps': 2.7961217917434604, 'velocity_bounding_sigma': 0.12749291902103269, 'delayed_output_frames': 512, 'enable_two_pass_filter': False, 'two_pass_cutoff_period': 45.0, 'tau_ou': 1.9457419085342305, 'enable_inekf_transition': True, 'enable_rotor_drag': True, 'rotor_drag_kd': 0.09669368905032731, 'rotor_drag_sigma': 0.6226596252563928, 'rotor_drag_gate': 12.0, 'flight_vibe_thresh': 0.4, 'ped_energy_thresh': 0.2, 'enable_pedestrian_prior': True, 'ped_sigma_lat': 0.21472410214586019, 'ped_sigma_vert': 0.6478380742938604, 'ped_gate_threshold': 9.0, 'ped_max_velocity_mps': 3.5, 'enable_rts_smoother': False, 'rts_min_span': 1.5, 'rts_sigma_a': 0.5, 'rts_r_p': 0.001, 'zupt_omega_thresh': 0.02, 'enable_spike_filter': True, 'spike_filter_acc': False, 'spike_filter_hampel': False, 'spike_thresh_gyro': 0.3, 'spike_thresh_acc': 4.0, 'enable_adaptive_q': True, 'adaptive_q_scale_acc': False, 'adaptive_q_alpha': 0.005, 'adaptive_q_min': 1.0, 'adaptive_q_max': 25.0, 'enable_unbiased_adaptive_q': True, 'enable_adaptive_tilt_gate': True, 'adaptive_q_ref': 0.0033119250641368016, 'enable_init_hardening': True, 'enable_student_t_updates': True, 'student_t_nu': 8.0, 'student_t_clamp_min': 1.0, 'student_t_clamp_max': 100.0}


def get_default_params_path() -> Path:
    """Return path to configs/params.json if present, else fallback."""
    cand = Path(__file__).resolve().parent / "configs" / "params.json"
    return cand


def load_params(params: Optional[Union[Dict[str, Any], str, Path]] = None) -> Dict[str, Any]:
    """Load parameter dictionary or return default parameters."""
    if params is None:
        return dict(DEFAULT_PARAMS)
    if isinstance(params, (str, Path)):
        p = Path(params)
        if p.is_file():
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        return dict(DEFAULT_PARAMS)
    return dict(params)


# ==============================================================================
# Module: mathlib.py
# ==============================================================================
"""Quaternion and SO(3) Kinematics Math Library.

SPEC §12.1 Implementation:
- Quaternion q = [w, x, y, z], Hamilton product q1 (x) q2, body -> world.
- Skew-symmetric cross product matrix: skew(v).
- Exponential map: quat_exp(phi) with angle a = |phi| (Taylor expansion for a < 1e-8).
- Logarithmic map: quat_log(q).
- Rotation matrix conversions: quat_to_rot_matrix(q) and rot_matrix_to_quat(R).
- Quaternion normalization with floor 1e-12.
- Sign continuity: quat_sign_continuity(q_new, q_old).
"""



def skew(v: Union[np.ndarray, list]) -> np.ndarray:
    """Return 3x3 skew-symmetric cross-product matrix of a 3-vector.

    skew(v) @ u == v x u.
    skew(v) = [[ 0,   -vz,  vy],
               [ vz,   0,  -vx],
               [-vy,  vx,   0 ]]
    """
    vx = float(v[0])
    vy = float(v[1])
    vz = float(v[2])
    return np.array([
        [0.0, -vz, vy],
        [vz, 0.0, -vx],
        [-vy, vx, 0.0]
    ], dtype=np.float64)


def quat_mult(q1: Union[np.ndarray, list], q2: Union[np.ndarray, list]) -> np.ndarray:
    """Hamilton quaternion product q1 (x) q2 where q = [w, x, y, z]."""
    w1, x1, y1, z1 = float(q1[0]), float(q1[1]), float(q1[2]), float(q1[3])
    w2, x2, y2, z2 = float(q2[0]), float(q2[1]), float(q2[2]), float(q2[3])

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z], dtype=np.float64)


def quat_conj(q: Union[np.ndarray, list]) -> np.ndarray:
    """Quaternion conjugate q* = [w, -x, -y, -z] where q = [w, x, y, z]."""
    return np.array([float(q[0]), -float(q[1]), -float(q[2]), -float(q[3])], dtype=np.float64)


def quat_normalize(q: Union[np.ndarray, list]) -> np.ndarray:
    """Normalize quaternion q = [w, x, y, z] to unit norm with floor 1e-12."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    norm = float(np.sqrt(w * w + x * x + y * y + z * z))
    if norm < 1e-12:
        norm = 1e-12
    inv_norm = 1.0 / norm
    return np.array([w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm], dtype=np.float64)


def quat_exp(phi: Union[np.ndarray, list]) -> np.ndarray:
    """Exponential map of rotation vector phi to unit Hamilton quaternion [w, x, y, z].

    Angle a = |phi|: exp(phi) = [cos(a/2), sin(a/2)*phi/a].
    For a < 1e-8, uses [1, phi/2] then normalizes.
    """
    phi_x = float(phi[0])
    phi_y = float(phi[1])
    phi_z = float(phi[2])
    a = float(np.sqrt(phi_x * phi_x + phi_y * phi_y + phi_z * phi_z))

    if a < 1e-8:
        raw = np.array([1.0, 0.5 * phi_x, 0.5 * phi_y, 0.5 * phi_z], dtype=np.float64)
        return quat_normalize(raw)

    half_a = 0.5 * a
    factor = float(np.sin(half_a) / a)
    return np.array([
        float(np.cos(half_a)),
        factor * phi_x,
        factor * phi_y,
        factor * phi_z
    ], dtype=np.float64)


def quat_log(q: Union[np.ndarray, list]) -> np.ndarray:
    """Logarithmic map of unit Hamilton quaternion q = [w, x, y, z] to rotation vector phi."""
    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])

    # Ensure w >= 0 so that angle is in [0, pi]
    if w < 0.0:
        w = -w
        x = -x
        y = -y
        z = -z

    v_norm = float(np.sqrt(x * x + y * y + z * z))
    if v_norm < 1e-8:
        scale = 2.0 / (w if w > 1e-12 else 1e-12)
        return np.array([scale * x, scale * y, scale * z], dtype=np.float64)

    angle = 2.0 * float(np.arctan2(v_norm, w))
    scale = angle / v_norm
    return np.array([scale * x, scale * y, scale * z], dtype=np.float64)


def quat_to_rot_matrix(q: Union[np.ndarray, list]) -> np.ndarray:
    """Convert Hamilton quaternion [w, x, y, z] to 3x3 rotation matrix R (body to world)."""
    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - w * z)
    r02 = 2.0 * (x * z + w * y)

    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - w * x)

    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit Hamilton quaternion [w, x, y, z] using Shepperd's algorithm."""
    r00, r01, r02 = float(R[0, 0]), float(R[0, 1]), float(R[0, 2])
    r10, r11, r12 = float(R[1, 0]), float(R[1, 1]), float(R[1, 2])
    r20, r21, r22 = float(R[2, 0]), float(R[2, 1]), float(R[2, 2])

    trace = r00 + r11 + r22
    if trace > 0.0:
        s = 0.5 / float(np.sqrt(trace + 1.0))
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    elif (r00 > r11) and (r00 > r22):
        s = 2.0 * float(np.sqrt(1.0 + r00 - r11 - r22))
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = 2.0 * float(np.sqrt(1.0 + r11 - r00 - r22))
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = 2.0 * float(np.sqrt(1.0 + r22 - r00 - r11))
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    return quat_normalize(q)


def quat_sign_continuity(q_new: Union[np.ndarray, list], q_old: Union[np.ndarray, list]) -> np.ndarray:
    """Enforce quaternion sign continuity dot(q_new, q_old) >= 0."""
    w1, x1, y1, z1 = float(q_new[0]), float(q_new[1]), float(q_new[2]), float(q_new[3])
    w2, x2, y2, z2 = float(q_old[0]), float(q_old[1]), float(q_old[2]), float(q_old[3])
    dot = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
    q_arr = np.array([w1, x1, y1, z1], dtype=np.float64)
    if dot < 0.0:
        return -q_arr
    return q_arr

# ==============================================================================
# Module: parser.py
# ==============================================================================
"""Tolerant streaming IMU CSV parser (SPEC §12.8, §12.7).

Handles:
- Stripping spaces and carriage returns (\\r).
- Skipping comment lines (#...) and blank lines.
- Skipping and counting rows with missing/unparseable timestamps (int64).
- Flagging invalid rows:
  - Malformed floats or NaN/Inf values (converted to NaN)
  - Truncated lines (missing columns filled with NaN)
  - Out of range checks: |omega_i| > invalid_gyro_max (default 35.0 rad/s)
                         |acc_i| > invalid_acc_max (default 160.0 m/s^2)
  - All-zero readings (all six values exactly 0.0)
- Streaming line-by-line iterator yielding (t_ns, omega, acc, is_valid, flags).
"""

from typing import Any, Dict, Iterable, Iterator, List, Optional, TextIO, Tuple, Union


class ParserFlags(IntFlag):
    """Bitmask flags indicating parsing status and anomalies."""
    OK = 0
    MISSING_TIMESTAMP = 1 << 0
    NAN_OR_INF = 1 << 1
    TRUNCATED = 1 << 2
    OUT_OF_RANGE_GYRO = 1 << 3
    OUT_OF_RANGE_ACC = 1 << 4
    ALL_ZEROS = 1 << 5
    COMMENT_OR_BLANK = 1 << 6


FLAG_OK = ParserFlags.OK
FLAG_MISSING_TIMESTAMP = ParserFlags.MISSING_TIMESTAMP
FLAG_NAN_OR_INF = ParserFlags.NAN_OR_INF
FLAG_TRUNCATED = ParserFlags.TRUNCATED
FLAG_OUT_OF_RANGE_GYRO = ParserFlags.OUT_OF_RANGE_GYRO
FLAG_OUT_OF_RANGE_ACC = ParserFlags.OUT_OF_RANGE_ACC
FLAG_ALL_ZEROS = ParserFlags.ALL_ZEROS
FLAG_COMMENT_OR_BLANK = ParserFlags.COMMENT_OR_BLANK




def is_comment_or_blank(line: str) -> bool:
    """Check if line is a comment (#...) or empty/blank whitespace."""
    s = line.strip()
    return len(s) == 0 or s.startswith("#")


def parse_imu_line(
    line: str,
    params: Optional[Dict[str, Any]] = None,
    official_format: bool = False,
) -> Tuple[Optional[int], np.ndarray, np.ndarray, bool, int]:
    """Pure function to parse a single IMU line in EuRoC or official format.

    EuRoC format (official_format=False):
        timestamp [ns], w_RS_S_x, w_RS_S_y, w_RS_S_z, a_RS_S_x, a_RS_S_y, a_RS_S_z
    Official format (official_format=True):
        timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    Args:
        line: Raw text line.
        params: Configuration dictionary containing invalid_gyro_max and invalid_acc_max.
        official_format: When True, columns 1:4 are accel and 4:7 are gyro.

    Returns:
        t_ns: int timestamp in nanoseconds, or None if unparseable/missing/comment.
        omega: np.ndarray shape (3,), float64 gyro reading [wx, wy, wz].
        acc: np.ndarray shape (3,), float64 accelerometer reading [ax, ay, az].
        is_valid: bool indicating whether sample is completely valid.
        flags: bitmask of ParserFlags.
    """
    s = line.strip()
    if len(s) == 0 or s.startswith("#"):
        return (
            None,
            np.full(3, np.nan, dtype=np.float64),
            np.full(3, np.nan, dtype=np.float64),
            False,
            int(ParserFlags.COMMENT_OR_BLANK),
        )

    # Thresholds from params
    gyro_max = 35.0
    acc_max = 160.0
    if params is not None:
        gyro_max = float(params.get("invalid_gyro_max", 35.0))
        acc_max = float(params.get("invalid_acc_max", 160.0))

    # Split fields
    if "," in s:
        parts = [p.strip() for p in s.split(",")]
    else:
        parts = [p.strip() for p in s.split()]

    flags = 0
    t_ns: Optional[int] = None

    # 1. Parse timestamp
    if len(parts) == 0 or not parts[0]:
        flags |= ParserFlags.MISSING_TIMESTAMP
    else:
        try:
            t_ns = int(parts[0])
        except ValueError:
            try:
                f_val = float(parts[0])
                if math.isfinite(f_val) and f_val.is_integer():
                    t_ns = int(f_val)
                else:
                    flags |= ParserFlags.MISSING_TIMESTAMP
            except ValueError:
                flags |= ParserFlags.MISSING_TIMESTAMP

    # 2. Parse 6 measurement fields
    vals = [np.nan] * 6
    if len(parts) < 7:
        flags |= ParserFlags.TRUNCATED

    for k in range(6):
        col_idx = k + 1
        if col_idx < len(parts):
            raw_field = parts[col_idx]
            try:
                v = float(raw_field)
                if math.isfinite(v):
                    vals[k] = v
                else:
                    vals[k] = np.nan
                    flags |= ParserFlags.NAN_OR_INF
            except ValueError:
                vals[k] = np.nan
                flags |= ParserFlags.NAN_OR_INF
        else:
            vals[k] = np.nan
            flags |= ParserFlags.NAN_OR_INF

    if official_format:
        acc = np.array(vals[0:3], dtype=np.float64)
        omega = np.array(vals[3:6], dtype=np.float64)
    else:
        omega = np.array(vals[0:3], dtype=np.float64)
        acc = np.array(vals[3:6], dtype=np.float64)

    # 3. Check bounds and all zeros if all finite
    if not (flags & ParserFlags.NAN_OR_INF):
        # Component-wise gyro check: |omega_i| > invalid_gyro_max
        if np.any(np.abs(omega) > gyro_max):
            flags |= ParserFlags.OUT_OF_RANGE_GYRO

        # Component-wise acc check: |acc_i| > invalid_acc_max
        if np.any(np.abs(acc) > acc_max):
            flags |= ParserFlags.OUT_OF_RANGE_ACC

        # All zeros check
        if np.all(omega == 0.0) and np.all(acc == 0.0):
            flags |= ParserFlags.ALL_ZEROS

    is_valid = (flags == 0) and (t_ns is not None)
    return (t_ns, omega, acc, is_valid, int(flags))


class IMUStream(Iterator[Tuple[int, np.ndarray, np.ndarray, bool, int]]):
    """Streaming iterator reading IMU CSV data line-by-line according to SPEC §12.8."""

    def __init__(
        self,
        lines_or_file: Union[str, Path, Iterable[str], TextIO],
        params: Optional[Dict[str, Any]] = None,
        stats: Optional[Dict[str, int]] = None,
        official_format: Optional[bool] = None,
    ) -> None:
        self.params = load_params(params)
        self.stats = stats
        self.skipped_timestamp_count: int = 0
        self.valid_count: int = 0
        self.invalid_count: int = 0
        self.total_lines_read: int = 0

        self._file_handle: Optional[TextIO] = None
        self._line_iterator: Iterator[str]

        if isinstance(lines_or_file, Path):
            self._file_handle = open(lines_or_file, "r", encoding="utf-8")
            self._line_iterator = iter(self._file_handle)
        elif isinstance(lines_or_file, str):
            if "\n" not in lines_or_file and Path(lines_or_file).is_file():
                self._file_handle = open(lines_or_file, "r", encoding="utf-8")
                self._line_iterator = iter(self._file_handle)
            else:
                self._line_iterator = iter(lines_or_file.splitlines())
        elif hasattr(lines_or_file, "__iter__"):
            self._line_iterator = iter(lines_or_file)
        else:
            raise TypeError(f"Unsupported input type for parse_imu_stream: {type(lines_or_file)}")

        self.official_format: bool = False
        self._init_column_format(official_format)

    def _init_column_format(self, official_format: Optional[bool]) -> None:
        peeked: List[str] = []
        detected = False
        for raw_line in self._line_iterator:
            s = raw_line.strip()
            if not s:
                peeked.append(raw_line)
                continue

            clean = s.lstrip("#").strip()
            parts = [p.strip() for p in (clean.split(",") if "," in clean else clean.split())]
            first_field = parts[0] if parts else ""
            is_header = False
            try:
                float(first_field)
                is_header = False
            except ValueError:
                is_header = True

            if is_header:
                cols_1_3 = [p.lower() for p in parts[1:4]] if len(parts) >= 4 else [p.lower() for p in parts[1:]]
                targets = ("ax", "accel_x", "a_x", "accel")
                if any(any(t == c or c.startswith(t) or t in c for t in targets) for c in cols_1_3):
                    detected = True
                    # Consume official header
                    break
                else:
                    if not s.startswith("#"):
                        # Non-comment header (e.g. text header) -> consume it
                        detected = False
                        break
                    else:
                        # Comment line starting with '#' -> keep in peeked
                        peeked.append(raw_line)
                        if any("w_" in c or "gyro" in c or "gx" in c for c in cols_1_3):
                            detected = False
                            break
                        continue
            else:
                # Numeric first token -> data row
                peeked.append(raw_line)
                detected = False
                break

        if official_format is not None:
            self.official_format = bool(official_format)
        else:
            self.official_format = detected

        if peeked:
            self._line_iterator = itertools.chain(peeked, self._line_iterator)

    @property
    def skipped_count(self) -> int:
        """Alias for skipped_timestamp_count."""
        return self.skipped_timestamp_count

    def __iter__(self) -> "IMUStream":
        return self

    def __next__(self) -> Tuple[int, np.ndarray, np.ndarray, bool, int]:
        while True:
            raw_line = next(self._line_iterator)
            self.total_lines_read += 1

            # Skip comment and blank lines
            if is_comment_or_blank(raw_line):
                continue

            t_ns, omega, acc, is_valid, flags = parse_imu_line(
                raw_line, self.params, official_format=self.official_format
            )

            # Skip rows with missing or unparseable timestamps (SPEC §12.8)
            if t_ns is None:
                self.skipped_timestamp_count += 1
                if self.stats is not None:
                    self.stats["skipped_timestamps"] = self.stats.get("skipped_timestamps", 0) + 1
                continue

            if is_valid:
                self.valid_count += 1
                if self.stats is not None:
                    self.stats["valid_samples"] = self.stats.get("valid_samples", 0) + 1
            else:
                self.invalid_count += 1
                if self.stats is not None:
                    self.stats["invalid_samples"] = self.stats.get("invalid_samples", 0) + 1

            return (t_ns, omega, acc, is_valid, flags)

    def close(self) -> None:
        """Close opened file handle if one was created."""
        if self._file_handle is not None and not self._file_handle.closed:
            self._file_handle.close()
            self._file_handle = None

    def __enter__(self) -> "IMUStream":
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()


def parse_imu_stream(
    lines_or_file: Union[str, Path, Iterable[str], TextIO],
    params: Optional[Dict[str, Any]] = None,
    stats: Optional[Dict[str, int]] = None,
    official_format: Optional[bool] = None,
) -> IMUStream:
    """Create streaming IMU CSV iterator yielding (t_ns, omega, acc, is_valid, flags)."""
    return IMUStream(lines_or_file, params=params, stats=stats, official_format=official_format)

# ==============================================================================
# Module: init.py
# ==============================================================================
"""Filter Initialisation (SPEC §12.2).

Performs:
- Static window tilt and bias estimation over T_init = 1.0 s (fallback 0.5 s).
- Gravity magnitude estimation clamped to [g_mag_clamp_min, g_mag_clamp_max] ([9.0, 10.5]).
- World gravity vector construction: g_w = [0, 0, -g_mag].
- Initial gyro bias: b_g = mean(gyro) when static; b_a = [0, 0, 0].
- Initial orientation: unit quaternion q aligning mean body specific force to world +z with zero yaw.
- Initial position p = 0 and velocity v = 0.
- Initial error covariance P = diag(sigma_theta_0^2, sigma_v_0^2, sigma_p_0^2, sigma_bg_0^2, sigma_ba_0^2).
- Motion fallback: if gyro std > motion_detect_gyro_std or accel-norm std > motion_detect_acc_std,
  use first sample's tilt, b_g = 0, and multiply sigma_theta_0 by motion_detect_sigma_theta_mult (5x).
"""





@dataclass
class NominalState:
    """16-parameter nominal kinematic state (SPEC §12.2, §12.3).

    Attributes:
        p: Position in world frame, shape (3,), float64.
        v: Velocity in world frame, shape (3,), float64.
        q: Orientation quaternion [w, x, y, z] (body to world), shape (4,), float64.
        b_g: Gyroscope bias in body frame, shape (3,), float64.
        b_a: Accelerometer bias in body frame, shape (3,), float64.
    """

    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    b_g: np.ndarray
    b_a: np.ndarray

    def __post_init__(self) -> None:
        self.p = np.asarray(self.p, dtype=np.float64).reshape(3)
        self.v = np.asarray(self.v, dtype=np.float64).reshape(3)
        self.q = np.asarray(self.q, dtype=np.float64).reshape(4)
        self.b_g = np.asarray(self.b_g, dtype=np.float64).reshape(3)
        self.b_a = np.asarray(self.b_a, dtype=np.float64).reshape(3)

    def copy(self) -> "NominalState":
        """Return a deep copy with cloned numpy arrays."""
        return NominalState(
            p=self.p.copy(),
            v=self.v.copy(),
            q=self.q.copy(),
            b_g=self.b_g.copy(),
            b_a=self.b_a.copy(),
        )

    def to_array(self) -> np.ndarray:
        """Serialize state to a flat 16-element float64 array [p(3), v(3), q(4), b_g(3), b_a(3)]."""
        arr = np.empty(16, dtype=np.float64)
        arr[0:3] = self.p
        arr[3:6] = self.v
        arr[6:10] = self.q
        arr[10:13] = self.b_g
        arr[13:16] = self.b_a
        return arr

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "NominalState":
        """Construct NominalState from a 16-element array."""
        arr = np.asarray(arr, dtype=np.float64).reshape(16)
        return cls(
            p=arr[0:3].copy(),
            v=arr[3:6].copy(),
            q=arr[6:10].copy(),
            b_g=arr[10:13].copy(),
            b_a=arr[13:16].copy(),
        )


class InitResult(NamedTuple):
    """Result of filter initialisation."""

    state: NominalState
    P: np.ndarray  # (15, 15) float64
    g_w: np.ndarray  # (3,) float64
    g_mag: float
    motion_detected: bool
    samples_used: int
    t0: Union[int, float]




def compute_initial_tilt(a_mean: np.ndarray) -> np.ndarray:
    """Compute initial unit quaternion [w, x, y, z] aligning specific force to world +z (SPEC §12.2).

    R(q) * (a_mean / |a_mean|) = [0, 0, 1]^T with zero yaw.
    Body specific force maps to world +z.

    Args:
        a_mean: (3,) specific force vector in body frame.

    Returns:
        q: (4,) unit Hamilton quaternion [w, x, y, z] with q_w >= 0.
    """
    a_vec = np.asarray(a_mean, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(a_vec)):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    norm_a = float(np.linalg.norm(a_vec))
    if not np.isfinite(norm_a) or norm_a < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    v = a_vec / norm_a  # unit body vector
    u = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # target world +z

    cos_theta = float(v[2])  # dot(v, [0, 0, 1]) == v[2]

    # Threshold for parallel vectors
    if not np.isfinite(cos_theta) or cos_theta > 0.999999999:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # Threshold for anti-parallel vectors (180 deg rotation about x)
    if cos_theta < -0.999999999:
        return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)

    # Cross product v x u = [v_y * 1 - v_z * 0, v_z * 0 - v_x * 1, v_x * 0 - v_y * 0]
    # v x u = [v[1], -v[0], 0.0]
    # Notice the z-component is identically zero, ensuring zero yaw.
    # Half-angle quaternion: q_raw = [1 + cos_theta, (v x u)_x, (v x u)_y, (v x u)_z]
    q_raw = np.array([1.0 + cos_theta, v[1], -v[0], 0.0], dtype=np.float64)
    q = quat_normalize(q_raw)

    # Ensure canonical positive scalar part
    if q[0] < 0.0:
        q = -q
    return q


def detect_motion(
    omega_window: np.ndarray,
    acc_window: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Detect dynamic motion in the initialisation window (SPEC §12.2).

    Condition: gyro std > motion_detect_gyro_std (0.2 rad/s) OR
               accel-norm std > motion_detect_acc_std (0.5 m/s^2).

    Args:
        omega_window: (N, 3) gyro readings.
        acc_window: (N, 3) accelerometer readings.
        params: Configuration dictionary.

    Returns:
        True if dynamic motion is detected, False if static.
    """
    cfg = load_params(params)
    gyro_thresh = float(cfg.get("motion_detect_gyro_std", 0.2))
    acc_thresh = float(cfg.get("motion_detect_acc_std", 0.5))

    omega_arr = np.asarray(omega_window, dtype=np.float64)
    acc_arr = np.asarray(acc_window, dtype=np.float64)

    if len(omega_arr) < 2 or len(acc_arr) < 2:
        return False

    valid_mask = (
        np.all(np.isfinite(omega_arr), axis=1)
        & np.all(np.isfinite(acc_arr), axis=1)
    )
    if np.sum(valid_mask) < 2:
        return False

    omega_valid = omega_arr[valid_mask]
    acc_valid = acc_arr[valid_mask]

    # Check axis-wise and norm gyro variation
    gyro_std_axes = np.std(omega_valid, axis=0)
    gyro_std_norm = np.std(np.linalg.norm(omega_valid, axis=1))

    # Check accel norm variation
    acc_norms = np.linalg.norm(acc_valid, axis=1)
    acc_norm_std = np.std(acc_norms)

    is_motion = bool(
        np.any(gyro_std_axes > gyro_thresh)
        or (gyro_std_norm > gyro_thresh)
        or (acc_norm_std > acc_thresh)
    )
    return is_motion


def initialize_covariance(
    params: Optional[Dict[str, Any]] = None,
    motion_detected: bool = False,
) -> np.ndarray:
    """Construct initial 15x15 error state covariance matrix P0 (SPEC §12.2).

    P0 = diag(sigma_theta_0^2, sigma_v_0^2, sigma_p_0^2, sigma_bg_0^2, sigma_ba_0^2).
    If motion is detected, sigma_theta_0 is multiplied by motion_detect_sigma_theta_mult (default 5.0).

    Args:
        params: Configuration parameters dictionary.
        motion_detected: Boolean flag indicating if motion fallback was triggered.

    Returns:
        P0: (15, 15) float64 diagonal matrix.
    """
    cfg = load_params(params)

    sigma_theta = float(cfg.get("sigma_theta_0", 0.02))
    if motion_detected:
        sigma_theta *= float(cfg.get("motion_detect_sigma_theta_mult", 5.0))

    sigma_v = float(cfg.get("sigma_v_0", 0.05))
    sigma_p = float(cfg.get("sigma_p_0", 0.01))
    sigma_bg = float(cfg.get("sigma_bg_0", 0.02))
    sigma_ba = float(cfg.get("sigma_ba_0", 0.2))

    diag_vals = np.array(
        [sigma_theta**2] * 3
        + [sigma_v**2] * 3
        + [sigma_p**2] * 3
        + [sigma_bg**2] * 3
        + [sigma_ba**2] * 3,
        dtype=np.float64,
    )
    return np.diag(diag_vals)


def initialize_from_window(
    t: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
) -> InitResult:
    """Perform filter initialisation on a batch window of IMU samples (SPEC §12.2).

    Selects the first T_init = 1.0 s of valid samples (fallback T_init_fallback = 0.5 s).

    Args:
        t: Array of timestamps (in nanoseconds or seconds).
        omega: Array of shape (N, 3), gyro readings [rad/s].
        acc: Array of shape (N, 3), accelerometer readings [m/s^2].
        params: Configuration dictionary.

    Returns:
        InitResult containing initial NominalState, P0, g_w, g_mag, motion flag, samples used, t0.

    Raises:
        ValueError: If valid sample duration is less than T_init_fallback.
    """
    cfg = load_params(params)
    t_init_target = float(cfg.get("T_init", 1.0))
    t_init_fallback = float(cfg.get("T_init_fallback", 0.5))
    g_min = float(cfg.get("g_mag_clamp_min", 9.0))
    g_max = float(cfg.get("g_mag_clamp_max", 10.5))

    t_arr = np.asarray(t)
    omega_arr = np.asarray(omega, dtype=np.float64)
    acc_arr = np.asarray(acc, dtype=np.float64)

    n_samples = len(t_arr)
    if n_samples < 2:
        raise ValueError(f"Insufficient samples for initialisation: n={n_samples} < 2")

    # Determine time units (nanoseconds vs seconds)
    total_span_raw = float(t_arr[-1] - t_arr[0])
    is_ns = total_span_raw > 1e6
    time_scale = 1e-9 if is_ns else 1.0
    t_s = (t_arr - t_arr[0]) * time_scale

    total_duration = float(t_s[-1])
    if total_duration < t_init_fallback:
        raise ValueError(
            f"Insufficient samples for initialisation: duration {total_duration:.3f}s "
            f"is less than fallback {t_init_fallback:.3f}s"
        )

    # Window selection: within T_init if available, otherwise fallback window
    if total_duration >= t_init_target:
        mask = t_s <= t_init_target
    else:
        mask = np.ones(n_samples, dtype=bool)

    omega_win = omega_arr[mask]
    acc_win = acc_arr[mask]

    # Filter finite samples
    win_valid_mask = (
        np.all(np.isfinite(omega_win), axis=1)
        & np.all(np.isfinite(acc_win), axis=1)
    )
    if not np.any(win_valid_mask):
        p = np.zeros(3, dtype=np.float64)
        v = np.zeros(3, dtype=np.float64)
        q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        b_g = np.zeros(3, dtype=np.float64)
        b_a = np.zeros(3, dtype=np.float64)
        g_mag = 9.80665
        g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)
        P0 = initialize_covariance(cfg, motion_detected=False)
        state = NominalState(p=p, v=v, q=q0, b_g=b_g, b_a=b_a)
        return InitResult(
            state=state,
            P=P0,
            g_w=g_w,
            g_mag=g_mag,
            motion_detected=False,
            samples_used=0,
            t0=t_arr[0],
        )

    omega_win_valid = omega_win[win_valid_mask]
    acc_win_valid = acc_win[win_valid_mask]
    samples_used = int(np.sum(win_valid_mask))

    # Detect motion
    motion = detect_motion(omega_win_valid, acc_win_valid, cfg)

    w_mean = np.mean(omega_win_valid, axis=0).astype(np.float64)
    # If calibration window has high angular rate (> 0.20 rad/s), this is dynamic motion
    if float(np.linalg.norm(w_mean)) > 0.20:
        motion = True

    # Initial estimates
    if motion:
        # Use first valid sample's tilt and zero gyro bias
        b_g = np.zeros(3, dtype=np.float64)
        a_ref = acc_win_valid[0]
    else:
        # Mean gyro as bias and mean accel for tilt
        b_g = w_mean
        a_ref = np.mean(acc_win_valid, axis=0).astype(np.float64)

    b_a = np.zeros(3, dtype=np.float64)
    p = np.zeros(3, dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)

    # Tilt quaternion
    q0 = compute_initial_tilt(a_ref)

    # Gravity magnitude and world gravity vector
    g_raw = float(np.linalg.norm(a_ref))
    g_mag = float(np.clip(g_raw, g_min, g_max))
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    # Initial covariance
    P0 = initialize_covariance(cfg, motion_detected=motion)

    state = NominalState(p=p, v=v, q=q0, b_g=b_g, b_a=b_a)

    return InitResult(
        state=state,
        P=P0,
        g_w=g_w,
        g_mag=g_mag,
        motion_detected=motion,
        samples_used=samples_used,
        t0=t_arr[0],
    )


class OnlineInitializer:
    """Streaming accumulator for filter initialisation (SPEC §12.2).

    Portable to fixed-size C++ ring buffer / preallocated arrays.
    Accumulates valid IMU samples until T_init (1.0 s) is reached.
    """

    MAX_BUFFER_SAMPLES: int = 1000

    def __init__(self, params: Optional[Dict[str, Any]] = None) -> None:
        self.params = load_params(params)
        self.t_init_target = float(self.params.get("T_init", 1.0))
        self.t_init_fallback = float(self.params.get("T_init_fallback", 0.5))

        # Preallocated buffers for C++ portability
        self._t_buf = np.empty(self.MAX_BUFFER_SAMPLES, dtype=np.int64)
        self._omega_buf = np.empty((self.MAX_BUFFER_SAMPLES, 3), dtype=np.float64)
        self._acc_buf = np.empty((self.MAX_BUFFER_SAMPLES, 3), dtype=np.float64)
        self._count = 0
        self._is_ready = False
        self._result: Optional[InitResult] = None

    @property
    def is_ready(self) -> bool:
        """Return True if initialisation has completed."""
        return self._is_ready

    def feed(
        self,
        t_ns: int,
        omega: np.ndarray,
        acc: np.ndarray,
        is_valid: bool = True,
    ) -> bool:
        """Feed a sample into the initializer.

        Returns True as soon as initialisation is complete.
        """
        if self._is_ready:
            return True
        if not is_valid:
            return False

        w_arr = np.asarray(omega, dtype=np.float64).reshape(3)
        a_arr = np.asarray(acc, dtype=np.float64).reshape(3)
        if not (np.all(np.isfinite(w_arr)) and np.all(np.isfinite(a_arr))):
            return False

        if self._count < self.MAX_BUFFER_SAMPLES:
            self._t_buf[self._count] = int(t_ns)
            self._omega_buf[self._count] = w_arr
            self._acc_buf[self._count] = a_arr
            self._count += 1

        # Check if duration reached T_init
        if self._count >= 2:
            dt_span_s = (self._t_buf[self._count - 1] - self._t_buf[0]) * 1e-9
            if dt_span_s >= self.t_init_target:
                self._compute_initialisation()
                return True

        return False

    def finalize_fallback(self) -> bool:
        """Attempt initialisation using fallback window if T_init was not reached."""
        if self._is_ready:
            return True
        if self._count < 2:
            self._compute_safe_fallback()
            return True

        dt_span_s = (self._t_buf[self._count - 1] - self._t_buf[0]) * 1e-9
        if dt_span_s >= self.t_init_fallback:
            self._compute_initialisation()
            return True

        self._compute_safe_fallback()
        return True

    def _compute_safe_fallback(self) -> None:
        """Safe fallback with nominal gravity and zero bias when calibration window is invalid."""
        p = np.zeros(3, dtype=np.float64)
        v = np.zeros(3, dtype=np.float64)
        q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        b_g = np.zeros(3, dtype=np.float64)
        b_a = np.zeros(3, dtype=np.float64)
        g_mag = 9.80665
        g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)
        P0 = initialize_covariance(self.params, motion_detected=False)
        state = NominalState(p=p, v=v, q=q0, b_g=b_g, b_a=b_a)
        t0 = int(self._t_buf[0]) if self._count > 0 else 0
        self._result = InitResult(
            state=state,
            P=P0,
            g_w=g_w,
            g_mag=g_mag,
            motion_detected=False,
            samples_used=self._count,
            t0=t0,
        )
        self._is_ready = True

    def _compute_initialisation(self) -> None:
        """Compute initial filter state from buffered samples."""
        t_slice = self._t_buf[: self._count]
        omega_slice = self._omega_buf[: self._count]
        acc_slice = self._acc_buf[: self._count]

        self._result = initialize_from_window(t_slice, omega_slice, acc_slice, params=self.params)
        self._is_ready = True

    def get_result(self) -> Optional[InitResult]:
        """Return the initialization result, or None if not ready."""
        return self._result

# ==============================================================================
# Module: strapdown.py
# ==============================================================================
"""Strapdown Nominal State Integration (SPEC §12.3).

Implements the nominal continuous-discrete kinematic propagation equations:
    omega_hat = omega - b_g
    a_hat = acc - b_a
    q <- normalize( q (x) exp(omega_hat * dt) )   # sign continuity maintained
    a_w = R(q_old) * a_hat + g_w                  # uses q_old for R
    p <- p + v * dt + 0.5 * a_w * dt^2
    v <- v + a_w * dt
    b_g, b_a unchanged

Constraints:
- Pure functions: step(state, sample, dt, g_w) -> new_state
- No printing in the filter loop.
- Fixed-size numpy float64 arrays portable to fixed-size C++.
"""






def strapdown_step(
    state: NominalState,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    dt: float,
    g_w: Union[np.ndarray, list],
) -> NominalState:
    """Pure function for single-step strapdown state propagation (SPEC §12.3).

    Args:
        state: Current NominalState (p, v, q, b_g, b_a).
        omega: (3,) angular velocity measurement in body frame [rad/s].
        acc: (3,) specific force measurement in body frame [m/s^2].
        dt: Time increment in seconds.
        g_w: (3,) world gravity vector [0, 0, -g_mag].

    Returns:
        New NominalState after dt propagation. Input state is not modified.
    """
    dt_f = float(dt)
    if dt_f <= 0.0:
        return state.copy()

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    g_w_arr = np.asarray(g_w, dtype=np.float64).reshape(3)

    # 1. Unbiased measurements (SPEC §12.3)
    omega_hat = omega_arr - state.b_g
    a_hat = acc_arr - state.b_a

    # 2. Update orientation quaternion
    delta_q = quat_exp(omega_hat * dt_f)
    q_new = quat_normalize(quat_mult(state.q, delta_q))
    q_new = quat_sign_continuity(q_new, state.q)

    # 3. Compute acceleration in world frame using q_old
    R_old = quat_to_rot_matrix(state.q)
    a_w = R_old @ a_hat + g_w_arr

    # 4. Integrate position and velocity
    dt_sq_half = 0.5 * (dt_f * dt_f)
    p_new = state.p + state.v * dt_f + a_w * dt_sq_half
    v_new = state.v + a_w * dt_f

    # 5. Biases unchanged
    b_g_new = state.b_g.copy()
    b_a_new = state.b_a.copy()

    return NominalState(
        p=p_new,
        v=v_new,
        q=q_new,
        b_g=b_g_new,
        b_a=b_a_new,
    )


def strapdown_propagate_sequence(
    state_init: NominalState,
    omega_seq: np.ndarray,
    acc_seq: np.ndarray,
    dt_seq: Union[np.ndarray, float],
    g_w: np.ndarray,
) -> Tuple[NominalState, np.ndarray, np.ndarray, np.ndarray]:
    """Propagate nominal state across a sequence of IMU samples.

    Args:
        state_init: Initial NominalState.
        omega_seq: (N, 3) float64 array of gyro samples.
        acc_seq: (N, 3) float64 array of accelerometer samples.
        dt_seq: (N,) float64 array of dt increments or float dt.
        g_w: (3,) float64 world gravity vector.

    Returns:
        final_state: NominalState at the end of sequence.
        p_hist: (N, 3) position history.
        v_hist: (N, 3) velocity history.
        q_hist: (N, 4) quaternion history.
    """
    n = len(omega_seq)
    if isinstance(dt_seq, (int, float)):
        dts = np.full(n, float(dt_seq), dtype=np.float64)
    else:
        dts = np.asarray(dt_seq, dtype=np.float64)

    p_hist = np.empty((n, 3), dtype=np.float64)
    v_hist = np.empty((n, 3), dtype=np.float64)
    q_hist = np.empty((n, 4), dtype=np.float64)

    curr_state = state_init.copy()
    for k in range(n):
        curr_state = strapdown_step(curr_state, omega_seq[k], acc_seq[k], dts[k], g_w)
        p_hist[k] = curr_state.p
        v_hist[k] = curr_state.v
        q_hist[k] = curr_state.q

    return curr_state, p_hist, v_hist, q_hist

# ==============================================================================
# Module: eskf.py
# ==============================================================================
"""15-State Error-State Kalman Filter (ESKF) Covariance Propagation and Joseph Update.

SPEC §12.4 and §12.5 Implementation:
- 15-state error state vector:
    delta_x = [delta_theta (0:3), delta_v (3:6), delta_p (6:9), delta_b_g (9:12), delta_b_a (12:15)]
- Analytical error-state transition matrix F:
    F = I15
    F[0:3, 0:3]   = R(omega_hat * dt)^T
    F[0:3, 9:12]  = -I * dt
    F[3:6, 0:3]   = -R(q_old) * skew(a_hat) * dt
    F[3:6, 12:15] = -R(q_old) * dt
    F[6:9, 3:6]   = I * dt
- Discrete process noise covariance Q:
    Q = diag(sigma_g^2*dt*I, sigma_a^2*dt*I, 0, sigma_bg^2*dt*I, sigma_ba^2*dt*I)
- Covariance conditioning:
    Symmetrization: P = 0.5 * (P + P^T)
    Diagonal clamping: P[i, i] in [cov_diag_floor, cov_diag_cap] ([1e-12, 1e6])
- Generic Joseph-form measurement update:
    S = H P H^T + R
    K = P H^T S^{-1}  (S is 3x3: invert with closed form + det floor)
    delta_x = K y
    P = (I - KH) P (I - KH)^T + K R K^T
- State injection:
    q <- normalize(q (x) exp(delta_theta))
    v += delta_v
    p += delta_p
    b_g += delta_bg
    b_a += delta_ba
- Error state reset:
    P <- G P G^T where G = I except G[0:3, 0:3] = I - skew(delta_theta / 2)

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside the filter math loop.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15> and Mat<3, 3>.
"""






def build_f_matrix(
    state: NominalState,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    dt: float,
) -> np.ndarray:
    """Build analytical 15x15 error-state transition matrix F (SPEC §12.4).

    Error state ordering:
        delta_x = [delta_theta(3), delta_v(3), delta_p(3), delta_b_g(3), delta_b_a(3)]

    Equations:
        F = I15
        F[0:3, 0:3]   = R(omega_hat * dt)^T
        F[0:3, 9:12]  = -I * dt
        F[3:6, 0:3]   = -R(q_old) * skew(a_hat) * dt
        F[3:6, 12:15] = -R(q_old) * dt
        F[6:9, 3:6]   = I * dt

    Args:
        state: Current NominalState (contains q, b_g, b_a).
        omega: (3,) angular velocity sample [rad/s].
        acc: (3,) specific force sample [m/s^2].
        dt: Sample delta time in seconds.

    Returns:
        F: (15, 15) float64 transition matrix.
    """
    dt_f = float(dt)
    F = np.eye(15, dtype=np.float64)

    if dt_f <= 0.0:
        return F

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    # Unbiased measurements (SPEC §12.3)
    omega_hat = omega_arr - state.b_g
    a_hat = acc_arr - state.b_a

    # Orientation transition: R(omega_hat * dt)^T
    delta_q = quat_exp(omega_hat * dt_f)
    R_delta = quat_to_rot_matrix(delta_q)
    F[0:3, 0:3] = R_delta.T

    # Gyro bias effect on orientation
    F[0:3, 9:12] = -np.eye(3, dtype=np.float64) * dt_f

    # Velocity transitions
    R_old = quat_to_rot_matrix(state.q)
    F[3:6, 0:3] = -R_old @ skew(a_hat) * dt_f
    F[3:6, 12:15] = -R_old * dt_f

    # Position transition
    F[6:9, 3:6] = np.eye(3, dtype=np.float64) * dt_f

    return F


def build_q_discrete(
    dt: float,
    params: Dict[str, Any],
    kappa: float = 1.0,
) -> np.ndarray:
    """Build discrete 15x15 process noise covariance matrix Q (SPEC §12.4).

    Equations:
        sigma_g  = sigma_g_base  * noise_mult_gyro * kappa
        sigma_a  = sigma_a_base  * noise_mult_acc  * kappa
        sigma_bg = sigma_bg_base * walk_mult_bg    * kappa
        sigma_ba = sigma_ba_base * walk_mult_ba    * kappa

        Q = diag(
            sigma_g^2  * dt * I3,
            sigma_a^2  * dt * I3,
            0 * I3,
            sigma_bg^2 * dt * I3,
            sigma_ba^2 * dt * I3,
        )

    Args:
        dt: Sample delta time in seconds.
        params: Configuration parameter dictionary.
        kappa: Quality inflation factor from degradation monitor (default 1.0).

    Returns:
        Q: (15, 15) float64 discrete process noise covariance matrix.
    """
    dt_f = float(dt)
    Q = np.zeros((15, 15), dtype=np.float64)

    if dt_f <= 0.0:
        return Q

    kappa_f = float(kappa)

    sigma_g = float(params["sigma_g_base"]) * float(params["noise_mult_gyro"]) * kappa_f
    sigma_a = float(params["sigma_a_base"]) * float(params["noise_mult_acc"]) * kappa_f
    sigma_bg = float(params["sigma_bg_base"]) * float(params["walk_mult_bg"]) * kappa_f
    sigma_ba = float(params["sigma_ba_base"]) * float(params["walk_mult_ba"]) * kappa_f

    q_theta = (sigma_g * sigma_g) * dt_f
    q_v = (sigma_a * sigma_a) * dt_f
    q_bg = (sigma_bg * sigma_bg) * dt_f
    q_ba = (sigma_ba * sigma_ba) * dt_f

    Q[0, 0] = q_theta
    Q[1, 1] = q_theta
    Q[2, 2] = q_theta

    Q[3, 3] = q_v
    Q[4, 4] = q_v
    Q[5, 5] = q_v

    # p block (6:9) is 0

    Q[9, 9] = q_bg
    Q[10, 10] = q_bg
    Q[11, 11] = q_bg

    Q[12, 12] = q_ba
    Q[13, 13] = q_ba
    Q[14, 14] = q_ba

    return Q


def condition_covariance(
    P: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Safely condition covariance matrix P via symmetrisation and diagonal clamping.

    P <- 0.5 * (P + P^T)
    P[i, i] <- clamp(P[i, i], diag_floor, diag_cap)
    When diagonal elements exceed diag_cap, row and column are scaled by sqrt(diag_cap / P[i,i])
    (congruence transformation D P D) to preserve correlation structure and positive-definiteness.

    Args:
        P: (N, N) covariance matrix.
        diag_floor: Minimum allowable diagonal element (default 1e-12).
        diag_cap: Maximum allowable diagonal element (default 1e6).

    Returns:
        P_cond: (N, N) conditioned covariance matrix.
    """
    P_sym = 0.5 * (P + P.T)
    n = P_sym.shape[0]

    # Floor handling
    for i in range(n):
        if P_sym[i, i] < diag_floor:
            P_sym[i, i] = diag_floor

    # Cap handling with positive-definiteness-preserving congruence transformation
    for i in range(n):
        if P_sym[i, i] > diag_cap:
            scale = float(np.sqrt(diag_cap / P_sym[i, i]))
            P_sym[i, :] *= scale
            P_sym[:, i] *= scale
            P_sym[i, i] = diag_cap

    return P_sym


def eskf_predict_covariance(
    P: np.ndarray,
    F: np.ndarray,
    Q: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Propagate 15-state error covariance: P <- F P F^T + Q with safe conditioning (SPEC §12.4).

    Args:
        P: (15, 15) current covariance matrix.
        F: (15, 15) error-state transition matrix.
        Q: (15, 15) discrete process noise covariance matrix.
        diag_floor: Minimum diagonal value.
        diag_cap: Maximum diagonal value.

    Returns:
        P_next: (15, 15) propagated and conditioned covariance matrix.
    """
    P_pred = F @ P @ F.T + Q
    return condition_covariance(P_pred, diag_floor=diag_floor, diag_cap=diag_cap)


def invert_3x3(A: np.ndarray, det_floor: float = 1e-12) -> np.ndarray:
    """Closed-form 3x3 matrix inverse with determinant floor (SPEC §12.5).

    Directly portable to fixed-size C++ without heap allocation.

    Args:
        A: (3, 3) float64 matrix.
        det_floor: Determinant magnitude floor to prevent division by zero.

    Returns:
        A_inv: (3, 3) inverse matrix.
    """
    a00 = float(A[0, 0])
    a01 = float(A[0, 1])
    a02 = float(A[0, 2])
    a10 = float(A[1, 0])
    a11 = float(A[1, 1])
    a12 = float(A[1, 2])
    a20 = float(A[2, 0])
    a21 = float(A[2, 1])
    a22 = float(A[2, 2])

    c00 = a11 * a22 - a12 * a21
    c01 = -(a10 * a22 - a12 * a20)
    c02 = a10 * a21 - a11 * a20

    c10 = -(a01 * a22 - a02 * a21)
    c11 = a00 * a22 - a02 * a20
    c12 = -(a00 * a21 - a01 * a20)

    c20 = a01 * a12 - a02 * a11
    c21 = -(a00 * a12 - a02 * a10)
    c22 = a00 * a11 - a01 * a10

    det = a00 * c00 + a01 * c01 + a02 * c02
    if abs(det) < det_floor:
        det = det_floor if det >= 0.0 else -det_floor

    inv_det = 1.0 / det

    return np.array([
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ], dtype=np.float64)


def eskf_joseph_update(
    P: np.ndarray,
    H: np.ndarray,
    y: np.ndarray,
    R: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generic Joseph-form Kalman measurement update (SPEC §12.5).

    Equations:
        S = H P H^T + R
        K = P H^T S^{-1}   (S is 3x3: inverted in closed form with det floor)
        delta_x = K y
        P_post = (I - KH) P (I - KH)^T + K R K^T
        P_post conditioned (symmetrised and clamped)

    Args:
        P: (15, 15) prior error covariance.
        H: (M, 15) measurement matrix (typically 3x15).
        y: (M,) measurement innovation.
        R: (M, M) measurement noise covariance.
        diag_floor: Diagonal floor.
        diag_cap: Diagonal cap.

    Returns:
        delta_x: (15,) error state update vector.
        P_post: (15, 15) posterior error covariance.
        S: (M, M) innovation covariance matrix.
    """
    H_arr = np.asarray(H, dtype=np.float64)
    y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
    R_arr = np.asarray(R, dtype=np.float64)

    # Innovation covariance S = H P H^T + R
    S = H_arr @ P @ H_arr.T + R_arr

    # Invert S (use closed-form 3x3 helper if M == 3)
    if S.shape == (3, 3):
        S_inv = invert_3x3(S, det_floor=diag_floor)
    else:
        S_inv = np.linalg.inv(S)

    # Kalman gain K = P H^T S^{-1}
    K = P @ H_arr.T @ S_inv

    # Error state correction delta_x = K y
    delta_x = K @ y_arr

    # Joseph-form covariance update: (I - KH) P (I - KH)^T + K R K^T
    I_KH = np.eye(15, dtype=np.float64) - K @ H_arr
    P_post = I_KH @ P @ I_KH.T + K @ R_arr @ K.T

    # Condition covariance (symmetrisation + clamping)
    P_post = condition_covariance(P_post, diag_floor=diag_floor, diag_cap=diag_cap)

    return delta_x, P_post, S


def eskf_inject_nominal_state(
    state: NominalState,
    delta_x: np.ndarray,
) -> NominalState:
    """Inject error state delta_x into NominalState (SPEC §12.5).

    Equations:
        q   <- normalize( q (x) exp(delta_theta) )
        v   <- v + delta_v
        p   <- p + delta_p
        b_g <- b_g + delta_b_g
        b_a <- b_a + delta_b_a

    Args:
        state: Current NominalState.
        delta_x: (15,) error state correction.

    Returns:
        new_state: Updated NominalState with unit quaternion normalization.
    """
    dx = np.asarray(delta_x, dtype=np.float64).reshape(15)

    delta_theta = dx[0:3]
    delta_v = dx[3:6]
    delta_p = dx[6:9]
    delta_bg = dx[9:12]
    delta_ba = dx[12:15]

    # Orientation injection
    delta_q = quat_exp(delta_theta)
    q_new = quat_normalize(quat_mult(state.q, delta_q))
    q_new = quat_sign_continuity(q_new, state.q)

    # Vector injections
    v_new = state.v + delta_v
    p_new = state.p + delta_p
    b_g_new = state.b_g + delta_bg
    b_a_new = state.b_a + delta_ba

    return NominalState(
        p=p_new,
        v=v_new,
        q=q_new,
        b_g=b_g_new,
        b_a=b_a_new,
    )


def eskf_reset_covariance(
    P: np.ndarray,
    delta_theta: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Reset error-state covariance after injection (SPEC §12.5).

    Equation:
        P <- G P G^T where G = I15 except G[0:3, 0:3] = I3 - skew(delta_theta / 2)

    Args:
        P: (15, 15) posterior covariance after Joseph update.
        delta_theta: (3,) injected orientation correction.
        diag_floor: Minimum diagonal value.
        diag_cap: Maximum diagonal value.

    Returns:
        P_reset: (15, 15) reset and conditioned error covariance.
    """
    d_th = np.asarray(delta_theta, dtype=np.float64).reshape(3)
    G = np.eye(15, dtype=np.float64)
    G[0:3, 0:3] = np.eye(3, dtype=np.float64) - skew(0.5 * d_th)

    P_reset = G @ P @ G.T
    return condition_covariance(P_reset, diag_floor=diag_floor, diag_cap=diag_cap)


def eskf_measurement_update(
    state: NominalState,
    P: np.ndarray,
    H: np.ndarray,
    y: np.ndarray,
    R: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute complete ESKF measurement update: Joseph update + injection + reset (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        H: (M, 15) measurement matrix.
        y: (M,) measurement innovation.
        R: (M, M) measurement noise covariance.
        params: Optional parameter dict (loads configs/params.json if None).
        apply_reset: Whether to apply covariance reset P <- G P G^T (default True).

    Returns:
        new_state: Updated NominalState after injection.
        new_P: Updated and conditioned error covariance matrix.
        delta_x: (15,) computed error-state correction vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    diag_floor = float(p_cfg.get("cov_diag_floor", 1e-12))
    diag_cap = float(p_cfg.get("cov_diag_cap", 1e6))

    # 1. Joseph-form update
    delta_x, P_post, _ = eskf_joseph_update(
        P, H, y, R, diag_floor=diag_floor, diag_cap=diag_cap
    )

    # 2. State injection
    new_state = eskf_inject_nominal_state(state, delta_x)

    # 3. Covariance reset
    if apply_reset:
        new_P = eskf_reset_covariance(
            P_post, delta_x[0:3], diag_floor=diag_floor, diag_cap=diag_cap
        )
    else:
        new_P = P_post

    return new_state, new_P, delta_x


def joseph_update(
    state: NominalState,
    P: np.ndarray,
    H: np.ndarray,
    y: np.ndarray,
    R: np.ndarray,
    gate_threshold: float = 1e9,
    params: Optional[Dict[str, Any]] = None,
    m: int = 0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Execute Joseph-form measurement update with optional Student-t covariance scaling (EXP-E82).

    When params.get("enable_student_t_updates", True) and m > 0:
        d2 = y^T * S^{-1} * y  (Mahalanobis distance)
        nu = params.get("student_t_nu", 8.0)
        d2_clamped = min(d2, clamp_max)
        w = clamp((nu + d2_clamped) / (nu + m), clamp_min, clamp_max)
        R_eff = R * w
        Recompute S_eff = H P H^T + R_eff, S_eff_inv
        Kalman gain K = P H^T S_eff_inv
        (No chi-2 gating applied; outlier is softly down-weighted)
    Else:
        Standard update with chi-2 gate: if d2 > gate_threshold: return state, P, False

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        H: (M, 15) measurement matrix.
        y: (M,) measurement innovation.
        R: (M, M) measurement noise covariance.
        gate_threshold: Chi-squared innovation gate threshold.
        params: Optional configuration dictionary.
        m: Measurement dimension degrees of freedom for Student-t (0 disables Student-t).
        apply_reset: Whether to apply covariance reset P <- G P G^T.

    Returns:
        state_post: Updated NominalState (or unchanged copy if rejected).
        P_post: Updated error covariance matrix (or unchanged copy if rejected).
        accepted: True if update was accepted and applied, False if rejected by gating.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    diag_floor = float(p_cfg.get("cov_diag_floor", 1e-12))
    diag_cap = float(p_cfg.get("cov_diag_cap", 1e6))

    H_arr = np.asarray(H, dtype=np.float64)
    y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
    R_arr = np.asarray(R, dtype=np.float64)

    HP = H_arr @ P
    HPH_T = HP @ H_arr.T
    S = HPH_T + R_arr

    try:
        if S.shape == (3, 3):
            S_inv = invert_3x3(S, det_floor=diag_floor)
        else:
            S_inv = np.linalg.inv(S)
    except Exception:
        return state, P, False

    S_inv_y = S_inv @ y_arr
    d2 = float(y_arr @ S_inv_y)
    if math.isnan(d2) or math.isinf(d2) or d2 < 0.0:
        return state, P, False

    enable_student_t = bool(p_cfg.get("enable_student_t_updates", False))
    if enable_student_t and m > 0:
        nu = float(p_cfg.get("student_t_nu", 8.0))
        clamp_min = float(p_cfg.get("student_t_clamp_min", 1.0))
        clamp_max = float(p_cfg.get("student_t_clamp_max", 100.0))

        d2_clamped = min(d2, clamp_max)
        w = (nu + d2_clamped) / (nu + float(m))
        w = float(np.clip(w, clamp_min, clamp_max))

        R_eff = R_arr * w
        S_eff = HPH_T + R_eff
        try:
            if S_eff.shape == (3, 3):
                S_eff_inv = invert_3x3(S_eff, det_floor=diag_floor)
            else:
                S_eff_inv = np.linalg.inv(S_eff)
        except Exception:
            return state, P, False
    else:
        if gate_threshold > 0.0 and d2 > gate_threshold:
            return state, P, False
        R_eff = R_arr
        S_eff_inv = S_inv

    # Kalman gain K = P H^T S_eff^{-1}
    K = P @ H_arr.T @ S_eff_inv

    # Error state correction delta_x = K y
    delta_x = K @ y_arr

    # Joseph-form covariance update: (I - KH) P (I - KH)^T + K R_eff K^T
    I_KH = np.eye(15, dtype=np.float64) - K @ H_arr
    P_post = I_KH @ P @ I_KH.T + K @ R_eff @ K.T
    P_post = condition_covariance(P_post, diag_floor=diag_floor, diag_cap=diag_cap)

    # State injection
    new_state = eskf_inject_nominal_state(state, delta_x)

    # Covariance reset
    if apply_reset:
        new_P = eskf_reset_covariance(
            P_post, delta_x[0:3], diag_floor=diag_floor, diag_cap=diag_cap
        )
    else:
        new_P = P_post

    return new_state, new_P, True


# ==============================================================================
# Module: updates.py
# ==============================================================================
"""Measurement Updates: Tilt Update and Zero Velocity Update (ZUPT) (SPEC §12.5).

Implements:
1. Tilt measurement update:
   - Gating condition:
       | ||a|| - g_mag | < tau_a * (1 + 5 * (1 - quality))  and  ||omega|| < omega_max
   - Normalized gravity observation in body frame:
       z = (a - b_a) / ||a - b_a||
   - Predicted gravity observation from current orientation:
       h_hat = R(q)^T [0, 0, 1]
   - Innovation:
       y = z - h_hat
   - Measurement matrix H (3x15):
       H = [skew(h_hat), 0, 0, 0, 0]  (only the delta_theta block is nonzero)
   - Measurement noise covariance R:
       sigma_tilt = tilt_sigma / max(quality, 0.1)
       R_tilt = sigma_tilt^2 * I3

2. Zero Velocity Update (ZUPT):
   - Stationary detector over sliding window (zupt_window_s = 0.25s):
       mean gyro norm < zupt_gyro_mean (0.03 rad/s)
       accel norm deviation from g_mag < zupt_acc_dev (0.15 m/s^2)
       quality >= zupt_min_quality (0.3)
   - Measurement:
       y = -v
   - Measurement matrix H (3x15):
       H = [0, I3, 0, 0, 0]  (only the delta_v block is nonzero)
   - Measurement noise covariance R:
       R_zupt = zupt_sigma^2 * I3 (zupt_sigma = 0.05)

3. StationaryWindowBuffer:
   - Fixed-capacity circular ring buffer for streaming stationary detection.
   - Fully portable to fixed-size C++ ring buffer without dynamic allocations.

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside filter loops.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15>, Mat<3, 15>, and Mat<3, 3>.
"""






def tilt_gating(
    acc: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Evaluate gating conditions for tilt measurement update (SPEC §12.5).

    Applied when:
        | ||a|| - g_mag | < tau_a * (1.0 + 5.0 * (1.0 - quality))
        and
        ||omega|| < omega_max

    Args:
        acc: (3,) accelerometer measurement [m/s^2].
        omega: (3,) gyroscope measurement [rad/s].
        g_mag: Reference gravity magnitude [m/s^2].
        quality: Signal quality score in [0.05, 1.0].
        params: Optional parameter dict (loads configs/params.json if None).

    Returns:
        True if the sample passes gating and tilt update can be applied.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    tau_a = float(p_cfg.get("tilt_tau_a", 0.3))
    omega_max = float(p_cfg.get("tilt_omega_max", 1.0))

    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)

    acc_norm = float(np.linalg.norm(acc_arr))
    omega_norm = float(np.linalg.norm(omega_arr))

    # Gyro check
    if omega_norm >= omega_max:
        return False

    # Quality-inflated accel threshold
    q_clamped = float(np.clip(quality, 0.0, 1.0))
    tau_eff = tau_a * (1.0 + 5.0 * (1.0 - q_clamped))

    acc_dev = abs(acc_norm - float(g_mag))
    return bool(acc_dev < tau_eff)


def compute_tilt_innovation(
    state: NominalState,
    acc: Union[np.ndarray, list],
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute tilt innovation y = z - h_hat and predicted gravity h_hat (SPEC §12.5).

    Equations:
        z = (a - b_a) / ||a - b_a||
        h_hat = R(q)^T [0, 0, 1]
        y = z - h_hat

    Args:
        state: Current NominalState (q, b_a).
        acc: (3,) specific force measurement [m/s^2].

    Returns:
        y: (3,) innovation vector.
        h_hat: (3,) predicted gravity direction in body frame.
    """
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    a_unbiased = acc_arr - state.b_a

    norm_a = float(np.linalg.norm(a_unbiased))
    if norm_a < 1e-12:
        norm_a = 1e-12

    z = a_unbiased / norm_a

    R_bw = quat_to_rot_matrix(state.q)
    # R_bw is body-to-world; R_wb = R_bw.T maps world +z to body frame
    h_hat = R_bw.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)

    y = z - h_hat
    return y, h_hat


def build_tilt_h_matrix(h_hat: Union[np.ndarray, list]) -> np.ndarray:
    """Build 3x15 tilt measurement Jacobian matrix H (SPEC §12.5).

    Equation:
        H = [skew(h_hat), 0, 0, 0, 0]  (3x15, theta block at 0:3 is nonzero)

    Args:
        h_hat: (3,) predicted gravity direction in body frame.

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    h_arr = np.asarray(h_hat, dtype=np.float64).reshape(3)
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 0:3] = skew(h_arr)
    return H


def build_tilt_r_matrix(
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 tilt measurement noise covariance matrix R (SPEC §12.5).

    Equation:
        sigma_tilt = tilt_sigma / max(quality, 0.1)
        R_tilt = (sigma_tilt^2 * r_scale) * I3

    Args:
        quality: Signal quality score in [0.05, 1.0].
        params: Optional parameter dict (loads configs/params.json if None).
        r_scale: Multiplier for gap recovery inflation (SPEC §12.7, default 1.0).

    Returns:
        R: (3, 3) float64 covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_base = float(p_cfg.get("tilt_sigma", 0.02))
    q_clamped = max(float(quality), 0.1)

    sigma_tilt = sigma_base / q_clamped
    var_tilt = (sigma_tilt * sigma_tilt) * float(r_scale)

    return var_tilt * np.eye(3, dtype=np.float64)


def tilt_update(
    state: NominalState,
    P: np.ndarray,
    acc: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray, bool]:
    """Execute complete tilt measurement update with gating check (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        acc: (3,) accelerometer sample [m/s^2].
        omega: (3,) gyroscope sample [rad/s].
        g_mag: Gravity magnitude [m/s^2].
        quality: Signal quality score [0.05, 1.0].
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier (e.g. from gap recovery).
        apply_reset: Whether to apply error-state reset G P G^T.

    Returns:
        new_state: Updated NominalState (or unchanged copy if rejected).
        new_P: Updated error covariance (or unchanged copy if rejected).
        delta_x: (15,) error state update (zeros if rejected).
        updated: True if update was applied, False if rejected by gating.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    if not tilt_gating(acc, omega, g_mag=g_mag, quality=quality, params=p_cfg):
        return state.copy(), P.copy(), np.zeros(15, dtype=np.float64), False

    y, h_hat = compute_tilt_innovation(state, acc)
    H = build_tilt_h_matrix(h_hat)

    # Dynamic noise scaling based on linear acceleration residual and gyro rate
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_norm = float(np.linalg.norm(acc_arr))
    delta_a = abs(acc_norm - float(g_mag))
    a_scale = float(p_cfg.get("tilt_a_scale", p_cfg.get("tilt_tau_a", 0.3)))
    a_ratio = delta_a / a_scale if a_scale > 1e-6 else 0.0

    omega_norm = float(np.linalg.norm(omega_arr))
    omega_scale = float(p_cfg.get("tilt_omega_scale", p_cfg.get("tilt_omega_max", 1.0)))
    omega_ratio = omega_norm / omega_scale if omega_scale > 1e-6 else 0.0

    alpha_a = float(p_cfg.get("tilt_alpha_a", 2.0))
    alpha_omega = float(p_cfg.get("tilt_alpha_omega", 8.0))

    dynamic_scale = 1.0 + alpha_a * (a_ratio * a_ratio) + alpha_omega * (omega_ratio * omega_ratio)
    if dynamic_scale > 1e6:
        dynamic_scale = 1e6

    R = build_tilt_r_matrix(quality=quality, params=p_cfg, r_scale=r_scale * dynamic_scale)

    gate_threshold = float(p_cfg.get("tilt_gate_threshold", 1e9))
    new_state, new_P, updated = joseph_update(
        state, P, H, y, R, gate_threshold=gate_threshold, params=p_cfg, m=3, apply_reset=apply_reset
    )

    return new_state, new_P, np.zeros(15, dtype=np.float64), updated


# ---------------------------------------------------------------------------
# Zero Velocity Update (ZUPT)
# ---------------------------------------------------------------------------


def zupt_detect_stationary(
    gyro_samples: np.ndarray,
    acc_samples: np.ndarray,
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Evaluate sliding window stationary condition for ZUPT (SPEC §12.5).

    Stationary conditions:
        1. quality >= zupt_min_quality (0.3)
        2. mean gyro norm < zupt_gyro_mean (0.03 rad/s)
        3. |mean accel norm - g_mag| < zupt_acc_dev (0.15 m/s^2)
        4. accel variance < (zupt_acc_dev)^2

    Args:
        gyro_samples: (N, 3) gyroscope measurements over window.
        acc_samples: (N, 3) accelerometer measurements over window.
        g_mag: Reference gravity magnitude [m/s^2].
        quality: Current signal quality score.
        params: Optional parameter dict (loads configs/params.json if None).

    Returns:
        True if stationary conditions are met and ZUPT can be applied.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    min_quality = float(p_cfg.get("zupt_min_quality", 0.3))
    if float(quality) < min_quality:
        return False

    if gyro_samples is None or acc_samples is None:
        return False

    g_arr = np.asarray(gyro_samples, dtype=np.float64).reshape(-1, 3)
    a_arr = np.asarray(acc_samples, dtype=np.float64).reshape(-1, 3)

    n = len(g_arr)
    if n == 0 or len(a_arr) != n:
        return False

    # 1. Mean gyro norm and latest sample gating
    gyro_thresh = float(p_cfg.get("zupt_omega_thresh", p_cfg.get("zupt_gyro_mean", 0.02)))
    gyro_norms = np.linalg.norm(g_arr, axis=1)
    mean_gyro = float(np.mean(gyro_norms))
    if mean_gyro >= gyro_thresh:
        return False

    if float(gyro_norms[-1]) > gyro_thresh * 1.5:
        return False

    # 2. Accel norm deviation from g_mag
    acc_dev_thresh = float(p_cfg.get("zupt_acc_dev", 0.15))
    acc_norms = np.linalg.norm(a_arr, axis=1)
    mean_acc = float(np.mean(acc_norms))
    acc_dev = abs(mean_acc - float(g_mag))
    if acc_dev >= acc_dev_thresh:
        return False

    # 3. Accel variance
    acc_var = float(np.var(acc_norms))
    var_thresh = float(p_cfg.get("zupt_acc_var", acc_dev_thresh * acc_dev_thresh))
    if acc_var >= var_thresh:
        return False

    return True


def build_zupt_h_matrix() -> np.ndarray:
    """Build 3x15 ZUPT measurement Jacobian matrix H (SPEC §12.5).

    Equation:
        H = [0, I3, 0, 0, 0]  (3x15, velocity block at 3:6 is identity)

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 3:6] = np.eye(3, dtype=np.float64)
    return H


def build_zupt_r_matrix(
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 ZUPT measurement noise covariance matrix R (SPEC §12.5).

    Equation:
        R_zupt = (zupt_sigma^2 * r_scale) * I3  (default zupt_sigma = 0.05)

    Args:
        params: Optional parameter dict (loads configs/params.json if None).
        r_scale: Covariance scaling multiplier (e.g. from gap recovery).

    Returns:
        R: (3, 3) float64 covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_z = float(p_cfg.get("zupt_sigma", 0.05))
    var_z = (sigma_z * sigma_z) * float(r_scale)
    return var_z * np.eye(3, dtype=np.float64)


def zupt_update(
    state: NominalState,
    P: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute Zero Velocity Update (ZUPT) measurement update (SPEC §12.5).

    Equations:
        y = -v
        H = [0, I3, 0, 0, 0]
        R = zupt_sigma^2 * I3

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier.
        apply_reset: Whether to apply error-state reset G P G^T.

    Returns:
        new_state: Updated NominalState with reduced velocity.
        new_P: Posterior conditioned covariance matrix.
        delta_x: (15,) error state update vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    y = -state.v
    H = build_zupt_h_matrix()
    R = build_zupt_r_matrix(params=p_cfg, r_scale=r_scale)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, params=p_cfg, apply_reset=apply_reset
    )

    return new_state, new_P, delta_x


def zupt_step(
    state: NominalState,
    P: np.ndarray,
    gyro_samples: np.ndarray,
    acc_samples: np.ndarray,
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray, bool]:
    """Check stationary conditions and optionally apply ZUPT update (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        gyro_samples: (N, 3) window of gyro measurements.
        acc_samples: (N, 3) window of accel measurements.
        g_mag: Gravity magnitude [m/s^2].
        quality: Signal quality score.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier.
        apply_reset: Whether to apply covariance reset.

    Returns:
        new_state: Updated NominalState (or unchanged copy if not stationary).
        new_P: Updated covariance matrix (or unchanged copy if not stationary).
        delta_x: (15,) error state correction (zeros if not stationary).
        updated: True if ZUPT was applied, False otherwise.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    is_stat = zupt_detect_stationary(
        gyro_samples, acc_samples, g_mag=g_mag, quality=quality, params=p_cfg
    )

    if not is_stat:
        return state.copy(), P.copy(), np.zeros(15, dtype=np.float64), False

    new_state, new_P, delta_x = zupt_update(
        state, P, params=p_cfg, r_scale=r_scale, apply_reset=apply_reset
    )
    return new_state, new_P, delta_x, True


# ---------------------------------------------------------------------------
# Streaming Stationary Window Buffer
# ---------------------------------------------------------------------------


class StationaryWindowBuffer:
    """Fixed-capacity circular ring buffer for streaming stationary detection.

    Designed for direct zero-heap porting to C++ std::array.
    Maintains a fixed history of timestamps, gyro measurements, and accel measurements.
    """

    def __init__(self, capacity: int = 64) -> None:
        """Initialize fixed-size buffer with capacity slots.

        Args:
            capacity: Maximum number of samples to store (default 64).
        """
        self.capacity: int = int(capacity)
        self.timestamps: np.ndarray = np.zeros(self.capacity, dtype=np.float64)
        self.gyro: np.ndarray = np.zeros((self.capacity, 3), dtype=np.float64)
        self.acc: np.ndarray = np.zeros((self.capacity, 3), dtype=np.float64)
        self.head: int = 0
        self.count: int = 0
        self.acc_filter = Butterworth2LowPass()

    def clear(self) -> None:
        """Reset the buffer state."""
        self.head = 0
        self.count = 0
        self.timestamps.fill(0.0)
        self.gyro.fill(0.0)
        self.acc.fill(0.0)
        self.acc_filter = Butterworth2LowPass()

    def add_sample(
        self,
        t: float,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        b_g: Optional[Union[np.ndarray, list]] = None,
    ) -> None:
        """Add a sample to circular ring buffer with gyro bias subtraction and accel filtering.

        Args:
            t: Sample timestamp in seconds.
            omega: (3,) gyroscope measurement [rad/s].
            acc: (3,) accelerometer measurement [m/s^2].
            b_g: Optional (3,) estimated gyro bias [rad/s].
        """
        idx = self.head
        self.timestamps[idx] = float(t)
        omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
        acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
        bg_arr = np.zeros(3, dtype=np.float64) if b_g is None else np.asarray(b_g, dtype=np.float64).reshape(3)

        self.gyro[idx, :] = omega_arr - bg_arr
        self.acc[idx, :] = self.acc_filter.step(acc_arr)

        self.head = (idx + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def get_window_samples(self, window_s: float) -> Tuple[np.ndarray, np.ndarray]:
        """Extract samples falling within window_s seconds from the most recent sample.

        Args:
            window_s: Sliding window duration in seconds.

        Returns:
            gyro_win: (K, 3) gyro measurements in window.
            acc_win: (K, 3) accel measurements in window.
        """
        if self.count == 0:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

        latest_idx = (self.head - 1) % self.capacity
        latest_t = self.timestamps[latest_idx]
        t_thresh = latest_t - float(window_s)

        # Collect sample indices going backward
        indices = []
        for i in range(self.count):
            curr_idx = (self.head - 1 - i) % self.capacity
            if self.timestamps[curr_idx] < t_thresh:
                break
            indices.append(curr_idx)

        if not indices:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

        # Order chronologically
        indices.reverse()
        idx_arr = np.array(indices, dtype=np.int64)

        return self.gyro[idx_arr].copy(), self.acc[idx_arr].copy()

    def is_stationary(
        self,
        g_mag: float = 9.80665,
        quality: float = 1.0,
        params: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Evaluate stationary detector over the recent window_s history.

        Args:
            g_mag: Reference gravity magnitude [m/s^2].
            quality: Signal quality score.
            params: Optional parameter dict.

        Returns:
            True if buffer contains a full valid window and stationary conditions are met.
        """
        if params is None:
            p_cfg = load_params()
        else:
            p_cfg = params

        window_s = float(p_cfg.get("zupt_window_s", 0.25))

        if self.count < 2:
            return False

        latest_idx = (self.head - 1) % self.capacity
        latest_t = self.timestamps[latest_idx]

        # Determine oldest sample time in the buffer
        oldest_idx = (self.head - self.count) % self.capacity
        oldest_t = self.timestamps[oldest_idx]

        # Require that accumulated history covers at least 90% of window_s
        if (latest_t - oldest_t) < (window_s * 0.9):
            return False

        gyro_win, acc_win = self.get_window_samples(window_s)
        if len(gyro_win) < 2:
            return False

        return zupt_detect_stationary(
            gyro_win, acc_win, g_mag=g_mag, quality=quality, params=p_cfg
        )


class Butterworth2LowPass:
    """2nd-order Butterworth low-pass filter (fc = 14 Hz, fs = 200 Hz)."""

    b0: float = 0.03657483584392802
    b1: float = 0.07314967168785604
    b2: float = 0.03657483584392802
    a1: float = -1.3908952814253894
    a2: float = 0.5371946248011015

    def __init__(self) -> None:
        self.d1 = np.zeros(3, dtype=np.float64)
        self.d2 = np.zeros(3, dtype=np.float64)
        self.initialized = False

    def reset(self, x0: np.ndarray) -> None:
        x0_arr = np.asarray(x0, dtype=np.float64).reshape(3)
        self.d1 = x0_arr * (self.b1 + self.b2 - self.a1 - self.a2)
        self.d2 = x0_arr * (self.b2 - self.a2)
        self.initialized = True

    def step(self, x: np.ndarray) -> np.ndarray:
        x_arr = np.asarray(x, dtype=np.float64).reshape(3)
        if not self.initialized:
            self.reset(x_arr)
            return x_arr.copy()
        y = x_arr * self.b0 + self.d1
        self.d1 = x_arr * self.b1 - y * self.a1 + self.d2
        self.d2 = x_arr * self.b2 - y * self.a2
        return y


def apply_velocity_bounding(
    state: NominalState,
    P: np.ndarray,
    max_speed: Optional[float] = None,
    sigma_v: Optional[float] = None,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Soft velocity bounding pseudo-measurement update."""
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    if not bool(p_cfg.get("enable_velocity_bounding", True)):
        return state, P, False

    limit = float(max_speed if max_speed is not None else p_cfg.get("max_velocity_mps", 3.0))
    sig = float(sigma_v if sigma_v is not None else p_cfg.get("velocity_bounding_sigma", 0.2))

    v = state.v
    v_norm = float(np.linalg.norm(v))
    if v_norm <= limit or v_norm < 1e-12:
        return state, P, False

    target_v = v * (limit / v_norm)
    y_v = target_v - v

    # Decouple P_pv and P_vp cross-covariance blocks so velocity bounding
    # pseudo-measurement does not inject unphysical position jumps/kicks
    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    H_v = np.zeros((3, 15), dtype=np.float64)
    H_v[0:3, 3:6] = np.eye(3, dtype=np.float64)
    R_v = (sig ** 2) * np.eye(3, dtype=np.float64)

    updated_state, updated_P, _ = eskf_measurement_update(state, P_decoupled, H_v, y_v, R_v)
    return updated_state, updated_P, True


def apply_rotor_drag_update(
    state: NominalState,
    P: np.ndarray,
    acc_raw: np.ndarray,
    omega_raw: np.ndarray,
    g_w: np.ndarray,
    kd: float = 0.22,
    sigma: float = 0.8,
    gate_threshold: float = 12.0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Rotor drag measurement update for multirotor drone flight."""
    if params is not None and not bool(params.get("enable_rotor_drag", True)):
        return state, P, False

    R_b2w = quat_to_rot_matrix(state.q)
    R_w2b = R_b2w.T

    acc_arr = np.asarray(acc_raw, dtype=np.float64).reshape(3)
    f_b_xy = acc_arr[:2] - state.b_a[:2]
    v_b = R_w2b @ state.v
    v_b_xy = v_b[:2]

    # Innovation: y = a_xy^b + kd * v_xy^b
    y = f_b_xy + kd * v_b_xy

    # Decouple P_pv and P_vp cross covariance
    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    # H_v = -kd * R_b2w(0:2, :)^T => shape (2, 15)
    H = np.zeros((2, 15), dtype=np.float64)
    for i in range(2):
        for j in range(3):
            H[i, 3 + j] = -kd * R_b2w[j, i]

    R_drag = (sigma ** 2) * np.eye(2, dtype=np.float64)

    return joseph_update(
        state, P_decoupled, H, y, R_drag, gate_threshold=gate_threshold, params=params, m=0
    )


def apply_pedestrian_prior(
    state: NominalState,
    P: np.ndarray,
    sigma_lat: float = 0.5,
    sigma_vert: float = 0.3,
    gate_threshold: float = 9.0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Pedestrian kinematic motion prior (lateral non-holonomic and vertical damping)."""
    if params is not None and not bool(params.get("enable_pedestrian_prior", True)):
        return state, P, False

    R_b2w = quat_to_rot_matrix(state.q)
    R_w2b = R_b2w.T

    v_b = R_w2b @ state.v
    v_lat = v_b[1]
    v_vert = state.v[2]

    # Innovation: y = target - current = [0 - v_lat, 0 - v_vert]^T = [-v_lat, -v_vert]^T
    y = np.array([-v_lat, -v_vert], dtype=np.float64)

    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    H = np.zeros((2, 15), dtype=np.float64)
    H[0, 3:6] = R_w2b[1, :]
    H[1, 3 + 2] = 1.0

    R_ped = np.diag([sigma_lat ** 2, sigma_vert ** 2])

    return joseph_update(
        state, P_decoupled, H, y, R_ped, gate_threshold=gate_threshold, params=params, m=2
    )



# ==============================================================================
# Module: monitor.py
# ==============================================================================
"""Degradation Monitor and Signal Quality Estimation (SPEC §12.6).

Implements:
1. High-frequency residual tracking:
   - r_k = ||a_k - a_{k-1}|| (accel norm)
   - rho_k = ||omega_k - omega_{k-1}|| (gyro norm)
2. Exponential moving variance:
   - var_k = (1 - alpha) * var_{k-1} + alpha * residual^2
   - alpha = 0.01 (monitor_alpha from configs/params.json)
3. Reference variance calibration:
   - Tracks nominal variance during initial static window (T_init = 1.0 s)
   - Freezes reference variance sigma_ref = sqrt(var_ref) once t > T_init
4. Adaptive noise ratio, quality score, and covariance inflation:
   - Noise ratio n = max(sigma_now / sigma_ref, 1.0)
   - Quality score = clamp(1.0 / n, 0.05, 1.0)
   - Covariance inflation factor kappa = clamp(n, 1.0, kappa_max) (kappa_max = 50.0)
5. Stationary gyro bias observation pseudo-update (SPEC §12.6):
   - Innovation y = omega_static_mean - b_g
   - Measurement matrix H = [0, 0, 0, I, 0] (3x15)
   - Measurement noise R = sigma_bgm^2 * I3 (sigma_bgm = 5e-3)
   - Exponential moving trend of static gyro mean

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside filter loops.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15> and scalar arithmetic.
"""





# ---------------------------------------------------------------------------
# Pure Helper Functions (Cookbook SPEC §12.6)
# ---------------------------------------------------------------------------


def compute_residual(
    curr: Union[np.ndarray, list],
    prev: Union[np.ndarray, list],
) -> float:
    """Compute Euclidean norm high-frequency residual between successive measurements (SPEC §12.6).

    Equations:
        r_k   = ||a_k - a_{k-1}||
        rho_k = ||omega_k - omega_{k-1}||

    Args:
        curr: Current 3-element vector [x, y, z].
        prev: Previous 3-element vector [x, y, z].

    Returns:
        Euclidean norm of vector difference as float.
    """
    diff = np.asarray(curr, dtype=np.float64).reshape(3) - np.asarray(
        prev, dtype=np.float64
    ).reshape(3)
    return float(np.linalg.norm(diff))


def update_moving_variance(
    prev_var: float,
    residual: float,
    alpha: float = 0.01,
) -> float:
    """Update exponential moving variance (SPEC §12.6).

    Equation:
        var_k = (1.0 - alpha) * var_{k-1} + alpha * residual^2

    Args:
        prev_var: Previous variance estimate.
        residual: High-frequency residual (norm of successive differences).
        alpha: Exponential smoothing factor (default 0.01).

    Returns:
        Updated variance estimate.
    """
    res_sq = float(residual) * float(residual)
    a = float(alpha)
    return (1.0 - a) * float(prev_var) + a * res_sq


def compute_noise_ratio(
    sigma_now: float,
    sigma_ref: float,
) -> float:
    """Compute noise ratio n = max(sigma_now / sigma_ref, 1.0) (SPEC §12.6).

    Args:
        sigma_now: Current noise standard deviation estimate.
        sigma_ref: Reference nominal noise standard deviation.

    Returns:
        Noise ratio n >= 1.0.
    """
    s_ref = float(sigma_ref)
    if s_ref <= 1e-12:
        return 1.0
    s_now = float(sigma_now)
    ratio = s_now / s_ref
    return max(ratio, 1.0)


def compute_quality_score(
    noise_ratio: float,
    min_quality: float = 0.05,
    max_quality: float = 1.0,
) -> float:
    """Compute bounded quality score (SPEC §12.6).

    Equation:
        quality = clamp(1.0 / n, min_quality, max_quality)

    Args:
        noise_ratio: Noise ratio n >= 1.0.
        min_quality: Floor bound (default 0.05).
        max_quality: Ceiling bound (default 1.0).

    Returns:
        Quality score in [min_quality, max_quality].
    """
    n = max(float(noise_ratio), 1e-12)
    val = 1.0 / n
    return max(float(min_quality), min(float(max_quality), val))


def compute_kappa(
    noise_ratio: float,
    kappa_max: float = 50.0,
) -> float:
    """Compute process noise covariance inflation factor kappa (SPEC §12.6).

    Equation:
        kappa = clamp(n, 1.0, kappa_max)

    Args:
        noise_ratio: Noise ratio n.
        kappa_max: Maximum allowed inflation (default 50.0).

    Returns:
        Inflation factor in [1.0, kappa_max].
    """
    n = float(noise_ratio)
    return max(1.0, min(float(kappa_max), n))


# ---------------------------------------------------------------------------
# Stationary Gyro Bias Observation Helpers (SPEC §12.6)
# ---------------------------------------------------------------------------


def build_gyro_bias_h_matrix() -> np.ndarray:
    """Build 3x15 measurement Jacobian matrix H for gyro bias pseudo-update (SPEC §12.6).

    Equation:
        H = [0, 0, 0, I, 0]  (3x15, delta_b_g block at 9:12 is I3)

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 9:12] = np.eye(3, dtype=np.float64)
    return H


def build_gyro_bias_r_matrix(
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 measurement covariance matrix R for stationary gyro bias (SPEC §12.6).

    Equation:
        R = sigma_bgm^2 * r_scale * I3

    Args:
        params: Optional configuration dictionary.
        r_scale: Scaling factor (default 1.0).

    Returns:
        R: (3, 3) float64 diagonal covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_bgm = float(p_cfg.get("sigma_bgm", 5e-3))
    var_bgm = (sigma_bgm * sigma_bgm) * float(r_scale)
    return var_bgm * np.eye(3, dtype=np.float64)


def gyro_bias_innovation(
    state: NominalState,
    omega_meas: Union[np.ndarray, list],
) -> np.ndarray:
    """Compute stationary gyro bias innovation y (SPEC §12.6).

    Equation:
        y = omega_static_mean - b_g

    Args:
        state: Current NominalState.
        omega_meas: (3,) stationary window gyro mean.

    Returns:
        y: (3,) innovation vector.
    """
    w_arr = np.asarray(omega_meas, dtype=np.float64).reshape(3)
    return w_arr - state.b_g


def update_bias_trend(
    prev_trend: Union[np.ndarray, list],
    omega_mean: Union[np.ndarray, list],
    alpha: float = 0.01,
) -> np.ndarray:
    """Update exponential average of stationary gyro mean (SPEC §12.6).

    Equation:
        trend_k = (1.0 - alpha) * trend_{k-1} + alpha * omega_mean

    Args:
        prev_trend: (3,) previous bias trend vector.
        omega_mean: (3,) stationary window gyro mean.
        alpha: Smoothing factor (default 0.01).

    Returns:
        Updated (3,) float64 bias trend vector.
    """
    prev_arr = np.asarray(prev_trend, dtype=np.float64).reshape(3)
    mean_arr = np.asarray(omega_mean, dtype=np.float64).reshape(3)
    a = float(alpha)
    return (1.0 - a) * prev_arr + a * mean_arr


def gyro_bias_update(
    state: NominalState,
    P: np.ndarray,
    omega_meas: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute ESKF measurement update using stationary gyro bias pseudo-measurement (SPEC §12.6).

    Equations:
        y = omega_meas - b_g
        H = [0, 0, 0, I, 0]
        R = sigma_bgm^2 * I3

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        omega_meas: (3,) stationary window gyro mean.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling factor.
        apply_reset: Whether to apply error-state reset.

    Returns:
        new_state: Updated NominalState with refined gyro bias.
        new_P: Posterior conditioned covariance matrix.
        delta_x: (15,) error state correction vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    y = gyro_bias_innovation(state, omega_meas)
    H = build_gyro_bias_h_matrix()
    R = build_gyro_bias_r_matrix(params=p_cfg, r_scale=r_scale)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, params=p_cfg, apply_reset=apply_reset
    )
    return new_state, new_P, delta_x


# ---------------------------------------------------------------------------
# Monitor State Dataclass & Pure Step Function
# ---------------------------------------------------------------------------


@dataclass
class MonitorState:
    """Immutable state container for degradation monitor (SPEC §12.6)."""

    t_prev: float = 0.0
    t_start: float = 0.0
    sample_count: int = 0
    prev_acc: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    prev_omega: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    var_acc: float = 0.0
    var_gyro: float = 0.0
    var_acc_ref: float = 0.0
    var_gyro_ref: float = 0.0
    calibrated: bool = False
    n_acc: float = 1.0
    n_gyro: float = 1.0
    noise_ratio: float = 1.0
    quality: float = 1.0
    kappa: float = 1.0
    residual_acc: float = 0.0
    residual_gyro: float = 0.0
    bias_trend: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    bias_trend_count: int = 0

    def copy(self) -> "MonitorState":
        """Return a deep copy with cloned arrays."""
        return MonitorState(
            t_prev=self.t_prev,
            t_start=self.t_start,
            sample_count=self.sample_count,
            prev_acc=self.prev_acc.copy(),
            prev_omega=self.prev_omega.copy(),
            var_acc=self.var_acc,
            var_gyro=self.var_gyro,
            var_acc_ref=self.var_acc_ref,
            var_gyro_ref=self.var_gyro_ref,
            calibrated=self.calibrated,
            n_acc=self.n_acc,
            n_gyro=self.n_gyro,
            noise_ratio=self.noise_ratio,
            quality=self.quality,
            kappa=self.kappa,
            residual_acc=self.residual_acc,
            residual_gyro=self.residual_gyro,
            bias_trend=self.bias_trend.copy(),
            bias_trend_count=self.bias_trend_count,
        )


def monitor_step(
    state: MonitorState,
    t: float,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
    is_stationary: bool = False,
    omega_static_mean: Optional[Union[np.ndarray, list]] = None,
) -> MonitorState:
    """Execute one degradation monitor update step as a pure function (SPEC §12.6).

    Args:
        state: Current MonitorState.
        t: Current sample timestamp in seconds.
        omega: (3,) gyroscope measurement [rad/s].
        acc: (3,) accelerometer measurement [m/s^2].
        params: Optional configuration dictionary.
        is_stationary: Whether current sample was detected stationary.
        omega_static_mean: Optional stationary window gyro mean.

    Returns:
        new_state: Updated MonitorState.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    alpha = float(p_cfg.get("monitor_alpha", 0.01))
    t_init = float(p_cfg.get("T_init", 1.0))
    kappa_max = float(p_cfg.get("kappa_max", 50.0))

    t_f = float(t)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    if state.sample_count == 0:
        new_state = state.copy()
        new_state.t_start = t_f
        new_state.t_prev = t_f
        new_state.prev_acc = acc_arr
        new_state.prev_omega = omega_arr
        new_state.sample_count = 1
        new_state.residual_acc = 0.0
        new_state.residual_gyro = 0.0
        new_state.quality = 1.0
        new_state.kappa = 1.0
        new_state.noise_ratio = 1.0
        return new_state

    # 1. Compute high-frequency residuals
    r_k = compute_residual(acc_arr, state.prev_acc)
    rho_k = compute_residual(omega_arr, state.prev_omega)

    # 2. Update exponential moving variance
    if state.sample_count == 1:
        var_acc = r_k * r_k
        var_gyro = rho_k * rho_k
    else:
        var_acc = update_moving_variance(state.var_acc, r_k, alpha=alpha)
        var_gyro = update_moving_variance(state.var_gyro, rho_k, alpha=alpha)

    # 3. Reference calibration logic over static window
    dt_elapsed = t_f - state.t_start
    if dt_elapsed < t_init:
        # Calibration phase: reference tracks current moving variance
        calibrated = False
        var_acc_ref = var_acc
        var_gyro_ref = var_gyro
        n_acc = 1.0
        n_gyro = 1.0
        noise_ratio = 1.0
        quality = 1.0
        kappa = 1.0
    else:
        # Calibration complete: reference variance is frozen
        if not state.calibrated:
            calibrated = True
            var_acc_ref = state.var_acc_ref if state.var_acc_ref > 0.0 else var_acc
            var_gyro_ref = state.var_gyro_ref if state.var_gyro_ref > 0.0 else var_gyro
        else:
            calibrated = True
            var_acc_ref = state.var_acc_ref
            var_gyro_ref = state.var_gyro_ref

        sigma_a_now = math.sqrt(max(var_acc, 0.0))
        sigma_g_now = math.sqrt(max(var_gyro, 0.0))
        sigma_a_ref = math.sqrt(max(var_acc_ref, 0.0))
        sigma_g_ref = math.sqrt(max(var_gyro_ref, 0.0))

        n_acc = compute_noise_ratio(sigma_a_now, sigma_a_ref)
        n_gyro = compute_noise_ratio(sigma_g_now, sigma_g_ref)
        noise_ratio = max(n_acc, n_gyro, 1.0)
        quality = compute_quality_score(noise_ratio)
        kappa = compute_kappa(noise_ratio, kappa_max=kappa_max)

    # 4. Stationary gyro bias trend tracking
    bias_trend = state.bias_trend.copy()
    trend_count = state.bias_trend_count
    if is_stationary and omega_static_mean is not None:
        mean_arr = np.asarray(omega_static_mean, dtype=np.float64).reshape(3)
        if trend_count == 0:
            bias_trend = mean_arr.copy()
        else:
            bias_trend = update_bias_trend(bias_trend, mean_arr, alpha=alpha)
        trend_count += 1

    return MonitorState(
        t_prev=t_f,
        t_start=state.t_start,
        sample_count=state.sample_count + 1,
        prev_acc=acc_arr,
        prev_omega=omega_arr,
        var_acc=var_acc,
        var_gyro=var_gyro,
        var_acc_ref=var_acc_ref,
        var_gyro_ref=var_gyro_ref,
        calibrated=calibrated,
        n_acc=n_acc,
        n_gyro=n_gyro,
        noise_ratio=noise_ratio,
        quality=quality,
        kappa=kappa,
        residual_acc=r_k,
        residual_gyro=rho_k,
        bias_trend=bias_trend,
        bias_trend_count=trend_count,
    )


# ---------------------------------------------------------------------------
# Streaming DegradationMonitor Class
# ---------------------------------------------------------------------------


class DegradationMonitor:
    """Streaming Degradation Monitor class for real-time filter execution (SPEC §12.6).

    Zero-allocation in the main loop, portable to C++.
    """

    def __init__(self, params: Optional[Dict[str, Any]] = None) -> None:
        """Initialize monitor with configuration parameters.

        Args:
            params: Optional parameter dictionary (loads configs/params.json if None).
        """
        if params is None:
            self._params = load_params()
        else:
            self._params = params

        self._state = MonitorState()

    def reset(self) -> None:
        """Reset internal state to initial conditions."""
        self._state = MonitorState()

    def step(
        self,
        t: float,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        is_stationary: bool = False,
        omega_static_mean: Optional[Union[np.ndarray, list]] = None,
    ) -> None:
        """Process incoming sample and update degradation metrics.

        Args:
            t: Timestamp in seconds.
            omega: (3,) gyro measurement [rad/s].
            acc: (3,) accel measurement [m/s^2].
            is_stationary: Optional boolean if current sample is stationary.
            omega_static_mean: Optional (3,) stationary window gyro mean.
        """
        self._state = monitor_step(
            self._state,
            t,
            omega,
            acc,
            params=self._params,
            is_stationary=is_stationary,
            omega_static_mean=omega_static_mean,
        )

    @property
    def quality(self) -> float:
        """Current quality score in [0.05, 1.0]."""
        return self._state.quality

    @property
    def kappa(self) -> float:
        """Current process noise inflation factor in [1.0, kappa_max]."""
        return self._state.kappa

    @property
    def noise_ratio(self) -> float:
        """Current noise ratio n = max(sigma_now / sigma_ref, 1.0)."""
        return self._state.noise_ratio

    @property
    def n(self) -> float:
        """Alias for noise_ratio."""
        return self._state.noise_ratio

    @property
    def calibrated(self) -> bool:
        """True if initial static calibration window is complete."""
        return self._state.calibrated

    @property
    def count(self) -> int:
        """Total number of processed samples."""
        return self._state.sample_count

    @property
    def var_acc(self) -> float:
        """Current moving variance of accel residual."""
        return self._state.var_acc

    @property
    def var_gyro(self) -> float:
        """Current moving variance of gyro residual."""
        return self._state.var_gyro

    @property
    def var_acc_ref(self) -> float:
        """Reference variance of accel residual."""
        return self._state.var_acc_ref

    @property
    def var_gyro_ref(self) -> float:
        """Reference variance of gyro residual."""
        return self._state.var_gyro_ref

    @property
    def residual_acc(self) -> float:
        """Most recent accel residual r_k."""
        return self._state.residual_acc

    @property
    def residual_gyro(self) -> float:
        """Most recent gyro residual rho_k."""
        return self._state.residual_gyro

    @property
    def bias_trend(self) -> np.ndarray:
        """Current stationary gyro bias trend vector."""
        return self._state.bias_trend

# ==============================================================================
# Module: gap.py
# ==============================================================================
"""Gap, Invalid-Sample, and Dropout Handler (SPEC §12.7, WP-26).

Handles:
1. Classification of invalid IMU samples:
   - NaN or Inf in any field
   - All six values exactly zero
   - Stuck sensor (identical values in all six fields for >= 3 consecutive rows)
   - Out-of-range sensor readings (|omega| > 35 rad/s or |a| > 160 m/s^2)
2. Non-positive delta time (dt <= 0):
   - Duplicate or backward timestamps freeze / copy previous pose
   - Skips integration and updates
3. Normal dt propagation:
   - dt <= 2.5 * dt_nom and valid sample
4. Large dt gap (dt > 2.5 * dt_nom):
   - Sub-stepping with n = ceil(dt / dt_nom) capped at gap_substep_max (800)
   - Velocity decay: v <- v * exp(-dt_sub / tau_gap) with tau_gap = 2.0s
   - Orientation integration with smoothed gyro omega_s (tau = 0.25s)
   - Covariance Q inflated by kappa_gap = 5.0
   - Sets recovery counter = recover_steps (100)
5. Single invalid row handling:
   - Treated as 1 dt_nom gap sub-step, still emits one pose row
6. Post-gap recovery:
   - Measurement noise covariance R multiplied by (1 + 10 * recover / recover_steps)
   - Decays toward 1.0 as recover counts down
7. Fixed-size circular ring buffer:
   - Capacity 64 states for rollback to the last reliable state

Constraints:
- Pure functions where possible: step(state, sample, params) -> state.
- No dynamic lists or dicts inside the filter math loop.
- No printing in the filter loop.
- Portable to fixed-size C++ Mat<15, 15> and scalar arithmetic.
- Parameters from configs/params.json.
"""






class GapCase(str, Enum):
    """Classification of time-step and sample conditions (SPEC §12.7)."""
    NORMAL = "normal"
    ZERO_OR_NEGATIVE_DT = "zero_or_negative_dt"
    GAP = "gap"
    INVALID_SAMPLE = "invalid_sample"


class GapStepResult(NamedTuple):
    """Result of processing a single sample through the GapHandler."""
    state: NominalState
    P: np.ndarray
    case: GapCase
    substeps: int
    apply_updates: bool
    r_scale: float
    is_valid: bool
    reason: str


# ---------------------------------------------------------------------------
# 1. Sample Classification Pure Functions
# ---------------------------------------------------------------------------


def classify_sample(
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    prev_sample: Optional[Union[np.ndarray, list]] = None,
    repeat_count: int = 0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[bool, str]:
    """Classify an IMU sample as valid or invalid according to SPEC §12.7.

    A sample is invalid if:
    - Any field is non-finite (NaN or Inf)
    - All six values are exactly zero
    - Sample equals previous one in all six fields for >= 3 consecutive rows (stuck)
    - |omega| > invalid_gyro_max (35.0 rad/s) or |a| > invalid_acc_max (160.0 m/s^2)

    Args:
        omega: (3,) angular velocity vector [rad/s].
        acc: (3,) specific force vector [m/s^2].
        prev_sample: Optional (6,) vector of previous [omega, acc].
        repeat_count: Number of consecutive identical rows seen prior to this sample.
        params: Optional configuration dictionary.

    Returns:
        (is_valid, reason): Tuple of bool and classification reason string.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    # 1. Check for NaN or Inf in any field
    if not (np.all(np.isfinite(omega_arr)) and np.all(np.isfinite(acc_arr))):
        return False, "nan_or_inf"

    # 2. Check for all six zeros
    if np.all(omega_arr == 0.0) and np.all(acc_arr == 0.0):
        return False, "all_zeros"

    # 3. Check out of range bounds
    gyro_max = float(p_cfg.get("invalid_gyro_max", 35.0))
    acc_max = float(p_cfg.get("invalid_acc_max", 160.0))

    if np.any(np.abs(omega_arr) > gyro_max) or float(np.linalg.norm(omega_arr)) > gyro_max:
        return False, "out_of_range_gyro"

    if np.any(np.abs(acc_arr) > acc_max) or float(np.linalg.norm(acc_arr)) > acc_max:
        return False, "out_of_range_acc"

    # 4. Check stuck sensor (>= 3 consecutive identical rows)
    if prev_sample is not None:
        curr_6 = np.concatenate([omega_arr, acc_arr])
        prev_6 = np.asarray(prev_sample, dtype=np.float64).reshape(6)
        if np.array_equal(curr_6, prev_6):
            if repeat_count + 1 >= 3:
                return False, "stuck"

    return True, "valid"


def compute_dt_nom(
    dt_samples: Union[np.ndarray, List[float]],
    default_dt: float = 0.005,
) -> float:
    """Compute nominal sampling interval dt_nom as median of valid dt samples (SPEC §12.7).

    Args:
        dt_samples: Sequence of dt values (first up to 200 valid samples).
        default_dt: Fallback dt if insufficient samples (default 0.005s = 200 Hz).

    Returns:
        dt_nom: Median dt in seconds.
    """
    if len(dt_samples) == 0:
        return float(default_dt)

    arr = np.asarray(dt_samples, dtype=np.float64)
    valid = arr[(arr > 0.0) & np.isfinite(arr)]
    if len(valid) == 0:
        return float(default_dt)

    sample_subset = valid[:200]
    return float(np.median(sample_subset))


# ---------------------------------------------------------------------------
# 2. Motion Smoothing and Recovery Functions
# ---------------------------------------------------------------------------


def update_smoothed_gyro(
    omega_s: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    b_g: Union[np.ndarray, list],
    dt: float,
    tau: float = 0.25,
) -> np.ndarray:
    """Update smoothed motion angular velocity omega_s (SPEC §12.7).

    Equation:
        omega_corrected = omega - b_g
        alpha = 1.0 - exp(-dt / tau)
        omega_s <- (1.0 - alpha) * omega_s + alpha * omega_corrected

    Args:
        omega_s: Current (3,) smoothed angular velocity [rad/s].
        omega: Current (3,) raw angular velocity measurement [rad/s].
        b_g: Current (3,) estimated gyroscope bias [rad/s].
        dt: Time increment in seconds.
        tau: Time constant in seconds (default 0.25s).

    Returns:
        Updated (3,) smoothed angular velocity array.
    """
    omega_s_arr = np.asarray(omega_s, dtype=np.float64).reshape(3)
    if dt <= 0.0:
        return omega_s_arr.copy()

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    bg_arr = np.asarray(b_g, dtype=np.float64).reshape(3)

    omega_corr = omega_arr - bg_arr
    alpha = 1.0 - math.exp(-float(dt) / float(tau))
    return (1.0 - alpha) * omega_s_arr + alpha * omega_corr


def get_recovery_multiplier(
    recover: int,
    recover_steps: int = 100,
) -> float:
    """Compute measurement covariance inflation multiplier during post-gap recovery (SPEC §12.7).

    Equation:
        multiplier = 1.0 + 10.0 * (recover / recover_steps)  for recover > 0
        multiplier = 1.0                                     for recover <= 0

    Args:
        recover: Current remaining recovery step count.
        recover_steps: Total duration of recovery period (default 100).

    Returns:
        Scalar covariance multiplier >= 1.0.
    """
    if recover <= 0:
        return 1.0
    steps = max(1, int(recover_steps))
    return 1.0 + 10.0 * (float(recover) / float(steps))


# ---------------------------------------------------------------------------
# 3. Gap Sub-step and Propagation Pure Functions
# ---------------------------------------------------------------------------


def gap_substep(
    state: NominalState,
    P: np.ndarray,
    omega_s: Union[np.ndarray, list],
    dt_sub: float,
    params: Dict[str, Any],
) -> Tuple[NominalState, np.ndarray]:
    """Execute a single gap propagation sub-step (SPEC §12.7).

    Equations:
        tau_gap: velocity decay time constant (default 2.0 s)
        decay = exp(-dt_sub / tau_gap)
        v_next = v * decay
        p_next = p + 0.5 * (v + v_next) * dt_sub
        delta_q = exp(omega_s * dt_sub)
        q_next = normalize(q (x) delta_q)
        P_next = F P F^T + Q * kappa_gap

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        omega_s: (3,) smoothed angular velocity [rad/s].
        dt_sub: Sub-step duration in seconds.
        params: Configuration dictionary containing tau_gap, kappa_gap, etc.

    Returns:
        (next_state, next_P): Propagated nominal state and conditioned error covariance.
    """
    dt_f = float(dt_sub)
    tau_gap = float(params.get("tau_gap", 2.0))
    kappa_gap = float(params.get("kappa_gap", 5.0))
    diag_floor = float(params.get("cov_diag_floor", 1e-12))
    diag_cap = float(params.get("cov_diag_cap", 1e6))

    omega_s_arr = np.asarray(omega_s, dtype=np.float64).reshape(3)

    # 1. Velocity decay and position integration
    decay = math.exp(-dt_f / tau_gap)
    v_next = state.v * decay
    p_next = state.p + 0.5 * (state.v + v_next) * dt_f

    # 2. Orientation integration using smoothed gyro omega_s
    delta_q = quat_exp(omega_s_arr * dt_f)
    q_rot = quat_mult(state.q, delta_q)
    q_next = quat_normalize(q_rot)
    q_next = quat_sign_continuity(q_next, state.q)

    # Biases remain constant across gap propagation
    next_state = NominalState(
        q=q_next,
        v=v_next,
        p=p_next,
        b_g=state.b_g.copy(),
        b_a=state.b_a.copy(),
    )

    # 3. Covariance propagation with inflated process noise
    F = build_f_matrix(state, omega_s_arr + state.b_g, state.b_a, dt_f)

    Q_base = build_q_discrete(dt_f, params, kappa=1.0)
    Q_gap = Q_base * kappa_gap

    P_pred = F @ P @ F.T + Q_gap
    next_P = condition_covariance(P_pred, diag_floor=diag_floor, diag_cap=diag_cap)

    return next_state, next_P


def propagate_gap(
    state: NominalState,
    P: np.ndarray,
    omega_s: Union[np.ndarray, list],
    dt: float,
    dt_nom: float,
    params: Dict[str, Any],
) -> Tuple[NominalState, np.ndarray, int]:
    """Propagate state and covariance across a large time gap via sub-stepping (SPEC §12.7).

    Equation:
        n = min(ceil(dt / dt_nom), gap_substep_max)
        dt_sub = dt / n
        Loop n steps with gap_substep(...)

    Args:
        state: NominalState prior to gap.
        P: Covariance matrix prior to gap.
        omega_s: Smoothed pre-gap angular velocity.
        dt: Total gap duration in seconds.
        dt_nom: Nominal sampling interval in seconds.
        params: Configuration dictionary.

    Returns:
        (state_after, P_after, n_steps): Propagated state, covariance, and number of sub-steps.
    """
    gap_substep_max = int(params.get("gap_substep_max", 800))
    nom_dt = max(float(dt_nom), 1e-6)

    # Numerical epsilon tolerance to prevent FP rounding from inflating exact multiples
    n = int(math.ceil((float(dt) / nom_dt) - 1e-9))
    n = max(1, min(n, gap_substep_max))
    dt_sub = float(dt) / float(n)

    curr_state = state
    curr_P = P
    for _ in range(n):
        curr_state, curr_P = gap_substep(curr_state, curr_P, omega_s, dt_sub, params)

    return curr_state, curr_P, n


# ---------------------------------------------------------------------------
# 4. Ring Buffer for Rollback to Last Reliable State
# ---------------------------------------------------------------------------


class StateRingBuffer:
    """Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7).

    Capacity is fixed (default 64) with no heap reallocations.
    Stores tuples of (t_ns, NominalState, P, omega_s).
    """

    def __init__(self, capacity: int = 64):
        self.capacity = max(1, int(capacity))
        self._t_ns: List[int] = [0] * self.capacity
        self._states: List[Optional[NominalState]] = [None] * self.capacity
        self._covs: List[Optional[np.ndarray]] = [None] * self.capacity
        self._omega_s: List[Optional[np.ndarray]] = [None] * self.capacity
        self._head: int = 0
        self._count: int = 0

    def push(
        self,
        t_ns: int,
        state: NominalState,
        P: np.ndarray,
        omega_s: np.ndarray,
    ) -> None:
        """Push a new snapshot into the ring buffer."""
        idx = self._head
        self._t_ns[idx] = int(t_ns)
        self._states[idx] = NominalState(
            q=state.q.copy(),
            v=state.v.copy(),
            p=state.p.copy(),
            b_g=state.b_g.copy(),
            b_a=state.b_a.copy(),
        )
        self._covs[idx] = P.copy()
        self._omega_s[idx] = omega_s.copy()

        self._head = (self._head + 1) % self.capacity
        if self._count < self.capacity:
            self._count += 1

    def __len__(self) -> int:
        return self._count

    def get_latest(self) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Retrieve most recently pushed snapshot."""
        if self._count == 0:
            return None
        latest_idx = (self._head - 1 + self.capacity) % self.capacity
        return (
            self._t_ns[latest_idx],
            self._states[latest_idx],  # type: ignore[return-value]
            self._covs[latest_idx],  # type: ignore[return-value]
            self._omega_s[latest_idx],  # type: ignore[return-value]
        )

    def get_oldest(self) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Retrieve oldest snapshot currently stored in the ring buffer."""
        if self._count == 0:
            return None
        oldest_idx = (self._head - self._count + self.capacity) % self.capacity
        return (
            self._t_ns[oldest_idx],
            self._states[oldest_idx],  # type: ignore[return-value]
            self._covs[oldest_idx],  # type: ignore[return-value]
            self._omega_s[oldest_idx],  # type: ignore[return-value]
        )

    def find_closest(
        self,
        t_target_ns: int,
    ) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Find stored snapshot with timestamp closest to t_target_ns."""
        if self._count == 0:
            return None

        best_idx = -1
        best_diff = float("inf")
        for i in range(self._count):
            idx = (self._head - 1 - i + self.capacity) % self.capacity
            diff = abs(self._t_ns[idx] - t_target_ns)
            if diff < best_diff:
                best_diff = diff
                best_idx = idx

        if best_idx < 0:
            return None

        return (
            self._t_ns[best_idx],
            self._states[best_idx],  # type: ignore[return-value]
            self._covs[best_idx],  # type: ignore[return-value]
            self._omega_s[best_idx],  # type: ignore[return-value]
        )

    def clear(self) -> None:
        """Clear all entries in the ring buffer."""
        self._head = 0
        self._count = 0


# ---------------------------------------------------------------------------
# 5. Streaming GapHandler Class
# ---------------------------------------------------------------------------


class GapHandler:
    """Streaming gap, dropout, and invalid-sample handler (SPEC §12.7).

    Orchestrates:
    - Sample validity checks
    - dt_nom tracking and calibration over first 200 valid samples
    - Large gap sub-stepping with velocity decay and covariance inflation
    - Pose freezing on non-positive dt
    - Post-gap measurement recovery counter and R_meas scaling
    - Gyro motion smoothing omega_s
    - State snapshot ring buffer for failure rollback
    """

    def __init__(self, params: Optional[Dict[str, Any]] = None):
        self.params: Dict[str, Any] = load_params() if params is None else dict(params)
        self.recover_steps: int = int(self.params.get("recover_steps", 100))
        self.recover: int = 0
        self.dt_nom: float = 0.005
        self.dt_nom_frozen: bool = False
        self._dt_samples: List[float] = []

        self.omega_s: np.ndarray = np.zeros(3, dtype=np.float64)
        self.prev_t_ns: Optional[int] = None
        self.prev_sample_6: Optional[np.ndarray] = None
        self.repeat_count: int = 0

        ring_size = int(self.params.get("ring_buffer_size", 64))
        self.ring_buffer = StateRingBuffer(capacity=ring_size)

    def process_sample(
        self,
        t_ns: int,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        state: NominalState,
        P: np.ndarray,
    ) -> GapStepResult:
        """Process an incoming IMU measurement row according to SPEC §12.7.

        Handles:
        1. First sample initialization
        2. Non-positive dt (dt <= 0): freeze/repeat pose, skip updates
        3. Invalid sample: 1 dt_nom sub-step, skip updates, set recover = recover_steps
        4. Gap (dt > 2.5 * dt_nom): sub-step propagation, skip updates, set recover = recover_steps
        5. Normal (dt <= 2.5 * dt_nom and valid): advance recovery counter, allow updates

        Args:
            t_ns: Nanosecond timestamp.
            omega: (3,) angular velocity [rad/s].
            acc: (3,) specific force [m/s^2].
            state: Current NominalState.
            P: Current (15, 15) error covariance matrix.

        Returns:
            GapStepResult containing propagated state, covariance, case enum, and update flags.
        """
        omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
        acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

        # 1. First sample handling
        if self.prev_t_ns is None:
            is_valid, reason = classify_sample(
                omega_arr,
                acc_arr,
                prev_sample=None,
                repeat_count=0,
                params=self.params,
            )
            self.prev_t_ns = int(t_ns)
            self.prev_sample_6 = np.concatenate([omega_arr, acc_arr])
            self.repeat_count = 1

            if is_valid:
                self.omega_s = omega_arr - state.b_g
            else:
                self.omega_s = np.zeros(3, dtype=np.float64)

            self.ring_buffer.push(int(t_ns), state, P, self.omega_s)
            return GapStepResult(
                state=state,
                P=P,
                case=GapCase.NORMAL if is_valid else GapCase.INVALID_SAMPLE,
                substeps=1,
                apply_updates=is_valid,
                r_scale=1.0,
                is_valid=is_valid,
                reason=reason,
            )

        # 2. Check delta time: integer difference first to avoid float64 precision loss on 19-digit nanosecond timestamps
        dt_ns = int(t_ns) - int(self.prev_t_ns)
        dt = float(dt_ns) * 1e-9

        # Case dt <= 0: freeze/copy previous pose, skip integration and updates
        if dt <= 0.0:
            frozen_state = NominalState(
                q=state.q.copy(),
                v=state.v.copy(),
                p=state.p.copy(),
                b_g=state.b_g.copy(),
                b_a=state.b_a.copy(),
            )
            return GapStepResult(
                state=frozen_state,
                P=P.copy(),
                case=GapCase.ZERO_OR_NEGATIVE_DT,
                substeps=0,
                apply_updates=False,
                r_scale=get_recovery_multiplier(self.recover, self.recover_steps),
                is_valid=False,
                reason="dt_non_positive",
            )

        # 3. Classify sample validity
        is_valid, reason = classify_sample(
            omega_arr,
            acc_arr,
            prev_sample=self.prev_sample_6,
            repeat_count=self.repeat_count,
            params=self.params,
        )

        curr_6 = np.concatenate([omega_arr, acc_arr])
        if self.prev_sample_6 is not None and np.array_equal(curr_6, self.prev_sample_6):
            self.repeat_count += 1
        else:
            self.repeat_count = 1
            self.prev_sample_6 = curr_6

        # 4. Handle invalid sample
        # "Invalid row | Same as gap mode for one dt_nom step; still emit one row"
        if not is_valid:
            self.recover = self.recover_steps
            r_scale = get_recovery_multiplier(self.recover, self.recover_steps)

            next_state, next_P = gap_substep(
                state,
                P,
                self.omega_s,
                self.dt_nom,
                self.params,
            )
            self.prev_t_ns = int(t_ns)
            self.ring_buffer.push(int(t_ns), next_state, next_P, self.omega_s)

            return GapStepResult(
                state=next_state,
                P=next_P,
                case=GapCase.INVALID_SAMPLE,
                substeps=1,
                apply_updates=False,
                r_scale=r_scale,
                is_valid=False,
                reason=reason,
            )

        # 5. Handle valid sample: update dt_nom calibration if not frozen
        if not self.dt_nom_frozen:
            self._dt_samples.append(dt)
            self.dt_nom = compute_dt_nom(self._dt_samples)
            if len(self._dt_samples) >= 200:
                self.dt_nom_frozen = True

        # Update smoothed angular velocity omega_s
        tau_smooth = float(self.params.get("smooth_omega_tau", 0.25))
        self.omega_s = update_smoothed_gyro(
            self.omega_s,
            omega_arr,
            state.b_g,
            dt,
            tau=tau_smooth,
        )

        # 6. Check if gap: dt > 2.5 * dt_nom
        if dt > 2.5 * self.dt_nom:
            # Gap mode!
            next_state, next_P, n_steps = propagate_gap(
                state,
                P,
                self.omega_s,
                dt,
                self.dt_nom,
                self.params,
            )
            self.recover = self.recover_steps
            r_scale = get_recovery_multiplier(self.recover, self.recover_steps)
            self.prev_t_ns = int(t_ns)
            self.ring_buffer.push(int(t_ns), next_state, next_P, self.omega_s)

            return GapStepResult(
                state=next_state,
                P=next_P,
                case=GapCase.GAP,
                substeps=n_steps,
                apply_updates=False,
                r_scale=r_scale,
                is_valid=True,
                reason="gap",
            )

        # 7. Normal propagation (dt <= 2.5 * dt_nom and valid)
        if self.recover > 0:
            self.recover -= 1

        r_scale = get_recovery_multiplier(self.recover, self.recover_steps)
        self.prev_t_ns = int(t_ns)
        self.ring_buffer.push(int(t_ns), state, P, self.omega_s)

        return GapStepResult(
            state=state,
            P=P,
            case=GapCase.NORMAL,
            substeps=1,
            apply_updates=True,
            r_scale=r_scale,
            is_valid=True,
            reason="valid",
        )

# ==============================================================================
# Module: blind.py
# ==============================================================================
"""Sensor Failure Detector and Blind-5s Predictor (SPEC §12.7, §12.9, WP-27).

Provides:
1. Pure extrapolation functions:
   - predict_blind_position(p_f, v_f, tau, tau_v, shrink_v, stationary_v_thresh)
   - predict_blind_orientation(q_f, omega_f, tau, tau_omega)
   - predict_blind_velocity(v_f, tau, tau_v, shrink_v, stationary_v_thresh)
   - predict_blind_pose(p_f, v_f, q_f, omega_f, tau, params)
2. State rollback:
   - ReliableRingBuffer: fixed-size circular ring buffer for reliable states (no dynamic allocations)
   - rollback_to_reliable: retrieve last reliable state prior to failure onset
3. Streaming FailureDetector and BlindPredictor:
   - Tracks invalid sample persistence continuously
   - Declares failure when invalid samples persist >= T_fail (1.0 s) or at stream termination
   - Performs state rollback to pre-failure reliable state
   - Extrapolates trajectory poses up to 5.0 s (clamping beyond 5.0 s)
   - Generates blind poses for --append-blind

Constraints:
- Portable to fixed-size C++: no dynamic heap allocations inside filter loop
- Strictly implements equations from docs/SPEC.md §12.7 and §12.9
- Parameters loaded from configs/params.json
- No print statements in the filter loop
"""







def predict_blind_position(
    p_f: Union[np.ndarray, list],
    v_f: Union[np.ndarray, list],
    tau: float,
    tau_v: float = 3.0,
    shrink_v: float = 0.9,
    stationary_v_thresh: float = 0.05,
) -> np.ndarray:
    """Extrapolate position using the decaying-velocity kinematic model (SPEC §12.9).

    Equation:
        p(tau) = p_f + v_f * shrink_v * tau_v * (1 - exp(-tau / tau_v))
        Clamped to pose at tau = 5.0 s for tau > 5.0 s.
        If |v_f| < stationary_v_thresh, holds stationary: p(tau) = p_f.

    Args:
        p_f: (3,) frozen position at failure time [m].
        v_f: (3,) frozen velocity at failure time [m/s].
        tau: Elapsed time since failure in seconds (tau = t - t_f).
        tau_v: Velocity decay time constant in seconds (default 3.0 s).
        shrink_v: Velocity shrink multiplier (default 0.9).
        stationary_v_thresh: Velocity norm threshold for stationary hold (default 0.05 m/s).

    Returns:
        (3,) extrapolated position array [m].
    """
    p_f_arr = np.asarray(p_f, dtype=np.float64).reshape(3)
    v_f_arr = np.asarray(v_f, dtype=np.float64).reshape(3)

    tau_val = float(tau)
    if tau_val <= 0.0:
        return p_f_arr.copy()

    # Poses beyond tau = 5.0 s clamp to the pose at tau = 5.0 s
    if tau_val > 5.0:
        tau_val = 5.0

    # Stationary hold
    v_norm = float(np.linalg.norm(v_f_arr))
    if v_norm < float(stationary_v_thresh):
        return p_f_arr.copy()

    tau_v_val = max(1e-6, float(tau_v))
    shrink_val = float(shrink_v)

    decay_factor = 1.0 - math.exp(-tau_val / tau_v_val)
    delta_p = v_f_arr * (shrink_val * tau_v_val * decay_factor)
    return p_f_arr + delta_p


def predict_blind_orientation(
    q_f: Union[np.ndarray, list],
    omega_f: Union[np.ndarray, list],
    tau: float,
    tau_omega: float = 1.5,
) -> np.ndarray:
    """Extrapolate orientation using decaying turn-rate kinematic model (SPEC §12.9).

    Equation:
        theta(tau) = omega_f * tau_omega * (1 - exp(-tau / tau_omega))
        q(tau) = normalize(q_f (x) exp(theta(tau)))
        Clamped to pose at tau = 5.0 s for tau > 5.0 s.

    Args:
        q_f: (4,) frozen unit quaternion [w, x, y, z] at failure time.
        omega_f: (3,) frozen smoothed body angular velocity [rad/s].
        tau: Elapsed time since failure in seconds (tau = t - t_f).
        tau_omega: Angular velocity decay time constant in seconds (default 1.5 s).

    Returns:
        (4,) extrapolated unit quaternion [w, x, y, z].
    """
    q_f_arr = np.asarray(q_f, dtype=np.float64).reshape(4)
    omega_f_arr = np.asarray(omega_f, dtype=np.float64).reshape(3)

    tau_val = float(tau)
    if tau_val <= 0.0:
        return quat_normalize(q_f_arr)

    # Clamping beyond 5.0 s
    if tau_val > 5.0:
        tau_val = 5.0

    tau_w_val = max(1e-6, float(tau_omega))
    decay_factor = 1.0 - math.exp(-tau_val / tau_w_val)
    rot_vec = omega_f_arr * (tau_w_val * decay_factor)

    delta_q = quat_exp(rot_vec)
    q_rot = quat_mult(q_f_arr, delta_q)
    q_out = quat_normalize(q_rot)
    return quat_sign_continuity(q_out, q_f_arr)


def predict_blind_velocity(
    v_f: Union[np.ndarray, list],
    tau: float,
    tau_v: float = 3.0,
    shrink_v: float = 0.9,
    stationary_v_thresh: float = 0.05,
) -> np.ndarray:
    """Compute instantaneous velocity along the decaying trajectory.

    Derivative:
        v(tau) = v_f * shrink_v * exp(-tau / tau_v)   for tau < 5.0 s
        v(tau) = 0                                     for tau >= 5.0 s (clamped pose)
        v(tau) = 0                                     if |v_f| < stationary_v_thresh

    Args:
        v_f: (3,) frozen velocity at failure time [m/s].
        tau: Elapsed time since failure in seconds.
        tau_v: Velocity decay time constant in seconds.
        shrink_v: Velocity shrink factor.
        stationary_v_thresh: Velocity norm threshold for stationary hold.

    Returns:
        (3,) instantaneous velocity array [m/s].
    """
    v_f_arr = np.asarray(v_f, dtype=np.float64).reshape(3)
    tau_val = float(tau)
    if tau_val < 0.0:
        tau_val = 0.0

    if tau_val >= 5.0:
        return np.zeros(3, dtype=np.float64)

    v_norm = float(np.linalg.norm(v_f_arr))
    if v_norm < float(stationary_v_thresh):
        return np.zeros(3, dtype=np.float64)

    tau_v_val = max(1e-6, float(tau_v))
    shrink_val = float(shrink_v)
    return v_f_arr * (shrink_val * math.exp(-tau_val / tau_v_val))


def predict_blind_pose(
    p_f: Union[np.ndarray, list],
    v_f: Union[np.ndarray, list],
    q_f: Union[np.ndarray, list],
    omega_f: Union[np.ndarray, list],
    tau: float,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Extrapolate complete pose (p, q) at elapsed time tau (SPEC §12.9).

    Args:
        p_f: (3,) frozen position.
        v_f: (3,) frozen velocity.
        q_f: (4,) frozen quaternion [w, x, y, z].
        omega_f: (3,) frozen smoothed gyro.
        tau: Elapsed time in seconds.
        params: Configuration dictionary (tau_v, tau_omega, shrink_v, stationary_v_thresh).

    Returns:
        (p, q): Tuple of (3,) position and (4,) unit quaternion.
    """
    if params is None:
        params = {}

    tau_v = float(params.get("tau_v", 3.0))
    tau_omega = float(params.get("tau_omega", 1.5))
    shrink_v = float(params.get("shrink_v", 0.9))
    stationary_thresh = float(params.get("stationary_v_thresh", 0.05))

    p = predict_blind_position(
        p_f,
        v_f,
        tau,
        tau_v=tau_v,
        shrink_v=shrink_v,
        stationary_v_thresh=stationary_thresh,
    )
    q = predict_blind_orientation(
        q_f,
        omega_f,
        tau,
        tau_omega=tau_omega,
    )
    return p, q


# ---------------------------------------------------------------------------
# 2. Ring Buffer for Reliable States & Rollback
# ---------------------------------------------------------------------------


class ReliableSnapshot(NamedTuple):
    """Reliable state snapshot for pre-failure rollback."""
    t_ns: int
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    omega_s: np.ndarray


class ReliableRingBuffer:
    """Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7).

    Preallocates fixed-size numpy arrays to avoid heap allocations in the filter loop.
    Capacity defaults to 64 snapshots.
    """

    def __init__(self, capacity: int = 256):
        self.capacity = max(1, int(capacity))
        self._t_ns = np.zeros(self.capacity, dtype=np.int64)
        self._p = np.zeros((self.capacity, 3), dtype=np.float64)
        self._v = np.zeros((self.capacity, 3), dtype=np.float64)
        self._q = np.zeros((self.capacity, 4), dtype=np.float64)
        self._omega_s = np.zeros((self.capacity, 3), dtype=np.float64)
        self._head: int = 0
        self._count: int = 0

    def push(
        self,
        t_ns: int,
        p: Union[np.ndarray, list],
        v: Union[np.ndarray, list],
        q: Union[np.ndarray, list],
        omega_s: Union[np.ndarray, list],
    ) -> None:
        """Push a reliable state snapshot into the circular buffer."""
        idx = self._head
        self._t_ns[idx] = int(t_ns)
        self._p[idx, :] = np.asarray(p, dtype=np.float64).reshape(3)
        self._v[idx, :] = np.asarray(v, dtype=np.float64).reshape(3)
        self._q[idx, :] = np.asarray(q, dtype=np.float64).reshape(4)
        self._omega_s[idx, :] = np.asarray(omega_s, dtype=np.float64).reshape(3)

        self._head = (self._head + 1) % self.capacity
        if self._count < self.capacity:
            self._count += 1

    def __len__(self) -> int:
        return self._count

    def get_latest(self) -> Optional[ReliableSnapshot]:
        """Retrieve most recently pushed snapshot."""
        if self._count == 0:
            return None
        latest_idx = (self._head - 1 + self.capacity) % self.capacity
        return ReliableSnapshot(
            t_ns=int(self._t_ns[latest_idx]),
            p=self._p[latest_idx].copy(),
            v=self._v[latest_idx].copy(),
            q=self._q[latest_idx].copy(),
            omega_s=self._omega_s[latest_idx].copy(),
        )

    def get_oldest(self) -> Optional[ReliableSnapshot]:
        """Retrieve oldest snapshot currently stored in the buffer."""
        if self._count == 0:
            return None
        oldest_idx = (self._head - self._count + self.capacity) % self.capacity
        return ReliableSnapshot(
            t_ns=int(self._t_ns[oldest_idx]),
            p=self._p[oldest_idx].copy(),
            v=self._v[oldest_idx].copy(),
            q=self._q[oldest_idx].copy(),
            omega_s=self._omega_s[oldest_idx].copy(),
        )

    def clear(self) -> None:
        """Clear the ring buffer."""
        self._head = 0
        self._count = 0


def rollback_to_reliable(
    ring_buffer: Any,
    t_onset_ns: int,
) -> ReliableSnapshot:
    """Retrieve the last reliable state prior to failure onset from a ring buffer.

    Searches backwards from the newest snapshot for the latest entry with t_ns < t_onset_ns.
    If no entry has t_ns < t_onset_ns, returns the oldest available entry.

    Args:
        ring_buffer: ReliableRingBuffer or StateRingBuffer instance.
        t_onset_ns: Timestamp (ns) of failure onset.

    Returns:
        ReliableSnapshot of (t_ns, p, v, q, omega_s).
    """
    if len(ring_buffer) == 0:
        return ReliableSnapshot(
            t_ns=int(t_onset_ns),
            p=np.zeros(3, dtype=np.float64),
            v=np.zeros(3, dtype=np.float64),
            q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_s=np.zeros(3, dtype=np.float64),
        )

    # 1. ReliableRingBuffer search
    if isinstance(ring_buffer, ReliableRingBuffer):
        cap = ring_buffer.capacity
        head = ring_buffer._head
        count = ring_buffer._count

        latest_idx = None
        for i in range(count):
            idx = (head - 1 - i + cap) % cap
            if ring_buffer._t_ns[idx] < t_onset_ns:
                latest_idx = idx
                break

        if latest_idx is None:
            oldest = ring_buffer.get_oldest()
            assert oldest is not None
            return oldest

        t_latest = int(ring_buffer._t_ns[latest_idx])
        p_latest = ring_buffer._p[latest_idx].copy()
        q_latest = ring_buffer._q[latest_idx].copy()

        # Improvement 3: 1.0s window averaging of v and omega_s
        window_ns = 1_000_000_000
        t_min_ns = t_latest - window_ns

        sum_v = np.zeros(3, dtype=np.float64)
        sum_w = np.zeros(3, dtype=np.float64)
        n_samples = 0

        for i in range(count):
            idx = (head - 1 - i + cap) % cap
            t_smp = int(ring_buffer._t_ns[idx])
            if t_smp > t_latest:
                continue
            if t_smp < t_min_ns:
                break
            sum_v += ring_buffer._v[idx]
            sum_w += ring_buffer._omega_s[idx]
            n_samples += 1

        v_avg = (sum_v / n_samples) if n_samples > 0 else ring_buffer._v[latest_idx].copy()
        w_avg = (sum_w / n_samples) if n_samples > 0 else ring_buffer._omega_s[latest_idx].copy()

        return ReliableSnapshot(
            t_ns=t_latest,
            p=p_latest,
            v=v_avg,
            q=q_latest,
            omega_s=w_avg,
        )

    # 2. StateRingBuffer (from src.ref.gap) compatibility
    cap = getattr(ring_buffer, "capacity", len(ring_buffer))
    head = getattr(ring_buffer, "_head", 0)
    count = getattr(ring_buffer, "_count", len(ring_buffer))
    t_list = getattr(ring_buffer, "_t_ns", [])
    states = getattr(ring_buffer, "_states", [])
    omegas = getattr(ring_buffer, "_omega_s", [])

    latest_idx = None
    for i in range(count):
        idx = (head - 1 - i + cap) % cap
        if t_list[idx] < t_onset_ns:
            latest_idx = idx
            break

    if latest_idx is None:
        oldest = ring_buffer.get_oldest()
        if oldest is not None:
            t_ns_old, st_old, _, w_old = oldest
            return ReliableSnapshot(
                t_ns=int(t_ns_old),
                p=st_old.p.copy(),
                v=st_old.v.copy(),
                q=st_old.q.copy(),
                omega_s=w_old.copy() if w_old is not None else np.zeros(3, dtype=np.float64),
            )
        return ReliableSnapshot(
            t_ns=int(t_onset_ns),
            p=np.zeros(3, dtype=np.float64),
            v=np.zeros(3, dtype=np.float64),
            q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_s=np.zeros(3, dtype=np.float64),
        )

    st_latest = states[latest_idx]
    t_latest = int(t_list[latest_idx])
    p_latest = st_latest.p.copy()
    q_latest = st_latest.q.copy()

    window_ns = 1_000_000_000
    t_min_ns = t_latest - window_ns

    sum_v = np.zeros(3, dtype=np.float64)
    sum_w = np.zeros(3, dtype=np.float64)
    n_samples = 0

    for i in range(count):
        idx = (head - 1 - i + cap) % cap
        t_smp = int(t_list[idx])
        if t_smp > t_latest:
            continue
        if t_smp < t_min_ns:
            break
        sum_v += states[idx].v
        w = omegas[idx]
        if w is not None:
            sum_w += w
        n_samples += 1

    v_avg = (sum_v / n_samples) if n_samples > 0 else st_latest.v.copy()
    w_latest = omegas[latest_idx]
    w_default = w_latest.copy() if w_latest is not None else np.zeros(3, dtype=np.float64)
    w_avg = (sum_w / n_samples) if n_samples > 0 else w_default

    return ReliableSnapshot(
        t_ns=t_latest,
        p=p_latest,
        v=v_avg,
        q=q_latest,
        omega_s=w_avg,
    )


# ---------------------------------------------------------------------------
# 3. Streaming FailureDetector and BlindPredictor Class
# ---------------------------------------------------------------------------


class BlindPredictor:
    """Streaming Failure Detector and Blind-5s Predictor (SPEC §12.7, §12.9).

    Manages:
    - Ring buffer of reliable pre-failure states
    - Continuous invalid sample duration tracking
    - Failure declaration when invalid duration >= T_fail (1.0 s) or at stream termination
    - Rollback to pre-failure reliable state
    - Extrapolation of blind poses clamped beyond 5.0 s
    """

    def __init__(
        self,
        params: Optional[Dict[str, Any]] = None,
        ring_buffer_capacity: Optional[int] = None,
    ):
        self.params = load_params() if params is None else dict(params)

        self.T_fail = float(self.params.get("T_fail", 1.0))
        self.tau_v = float(self.params.get("tau_v", 3.0))
        self.tau_omega = float(self.params.get("tau_omega", 1.5))
        self.shrink_v = float(self.params.get("shrink_v", 0.9))
        self.stationary_v_thresh = float(self.params.get("stationary_v_thresh", 0.05))

        cap = ring_buffer_capacity or int(self.params.get("ring_buffer_size", 64))
        self.ring_buffer = ReliableRingBuffer(capacity=cap)

        self.is_failed: bool = False
        self.failure_time_ns: Optional[int] = None
        self.invalid_start_ns: Optional[int] = None
        self.last_valid_t_ns: Optional[int] = None
        self.frozen_state: Optional[ReliableSnapshot] = None

    def record_reliable_state(
        self,
        t_ns: int,
        p: Optional[Union[np.ndarray, list]] = None,
        v: Optional[Union[np.ndarray, list]] = None,
        q: Optional[Union[np.ndarray, list]] = None,
        omega_s: Optional[Union[np.ndarray, list]] = None,
        state: Optional[NominalState] = None,
    ) -> None:
        """Push a verified reliable state into the ring buffer."""
        if state is not None:
            pos = state.p
            vel = state.v
            ori = state.q
        else:
            pos = p if p is not None else np.zeros(3, dtype=np.float64)
            vel = v if v is not None else np.zeros(3, dtype=np.float64)
            ori = q if q is not None else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        w_s = omega_s if omega_s is not None else np.zeros(3, dtype=np.float64)

        self.ring_buffer.push(int(t_ns), pos, vel, ori, w_s)
        self.last_valid_t_ns = int(t_ns)
        self.invalid_start_ns = None

    def update(
        self,
        t_ns: int,
        is_valid: bool,
        p: Optional[Union[np.ndarray, list]] = None,
        v: Optional[Union[np.ndarray, list]] = None,
        q: Optional[Union[np.ndarray, list]] = None,
        omega_s: Optional[Union[np.ndarray, list]] = None,
        state: Optional[NominalState] = None,
    ) -> bool:
        """Process sample validity and detect continuous failure.

        Args:
            t_ns: Current sample timestamp in nanoseconds.
            is_valid: True if sample passed validity checks.
            p: Optional position array.
            v: Optional velocity array.
            q: Optional quaternion array.
            omega_s: Optional smoothed angular velocity.
            state: Optional NominalState.

        Returns:
            True if sensor failure has been declared, False otherwise.
        """
        if self.is_failed:
            return True

        if is_valid:
            if p is not None or state is not None:
                self.record_reliable_state(t_ns, p=p, v=v, q=q, omega_s=omega_s, state=state)
            self.invalid_start_ns = None
            self.last_valid_t_ns = int(t_ns)
            return False

        # Invalid sample
        if self.invalid_start_ns is None:
            self.invalid_start_ns = int(t_ns)

        invalid_duration_s = (int(t_ns) - self.invalid_start_ns) * 1e-9
        if invalid_duration_s >= self.T_fail:
            self.declare_failure(failure_time_ns=int(t_ns), failure_onset_ns=self.invalid_start_ns)
            return True

        return False

    def declare_failure(
        self,
        failure_time_ns: int,
        failure_onset_ns: Optional[int] = None,
    ) -> None:
        """Declare sensor failure, rollback state, and enter blind extrapolation mode."""
        if self.is_failed:
            return

        onset_ns = int(failure_onset_ns) if failure_onset_ns is not None else int(failure_time_ns)
        self.frozen_state = rollback_to_reliable(self.ring_buffer, onset_ns)
        self.failure_time_ns = int(failure_time_ns)
        self.is_failed = True

    def notify_stream_end(self, final_t_ns: Optional[int] = None) -> None:
        """Declare failure at stream termination and freeze last reliable state."""
        if self.is_failed:
            return

        if len(self.ring_buffer) > 0:
            latest = self.ring_buffer.get_latest()
            assert latest is not None
            onset_ns = int(final_t_ns) if final_t_ns is not None else latest.t_ns
            self.frozen_state = rollback_to_reliable(self.ring_buffer, onset_ns + 1)
        else:
            t_fallback = int(final_t_ns) if final_t_ns is not None else 0
            self.frozen_state = ReliableSnapshot(
                t_ns=t_fallback,
                p=np.zeros(3, dtype=np.float64),
                v=np.zeros(3, dtype=np.float64),
                q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                omega_s=np.zeros(3, dtype=np.float64),
            )

        self.failure_time_ns = int(final_t_ns) if final_t_ns is not None else self.frozen_state.t_ns
        self.is_failed = True

    def set_frozen_state(
        self,
        t_f_ns: int,
        p_f: Union[np.ndarray, list],
        v_f: Union[np.ndarray, list],
        q_f: Union[np.ndarray, list],
        omega_f: Union[np.ndarray, list],
    ) -> None:
        """Directly set frozen pre-failure state and enter failed mode."""
        self.frozen_state = ReliableSnapshot(
            t_ns=int(t_f_ns),
            p=np.asarray(p_f, dtype=np.float64).reshape(3),
            v=np.asarray(v_f, dtype=np.float64).reshape(3),
            q=np.asarray(q_f, dtype=np.float64).reshape(4),
            omega_s=np.asarray(omega_f, dtype=np.float64).reshape(3),
        )
        self.failure_time_ns = int(t_f_ns)
        self.is_failed = True

    def predict(self, t_ns: int) -> Tuple[np.ndarray, np.ndarray]:
        """Predict blind pose at timestamp t_ns >= t_f_ns.

        Args:
            t_ns: Target timestamp in nanoseconds.

        Returns:
            (p, q): Extrapolated position and quaternion.
        """
        if not self.is_failed or self.frozen_state is None:
            raise RuntimeError("Cannot predict blind pose before failure is declared.")

        tau = (int(t_ns) - self.frozen_state.t_ns) * 1e-9
        return predict_blind_pose(
            self.frozen_state.p,
            self.frozen_state.v,
            self.frozen_state.q,
            self.frozen_state.omega_s,
            tau=tau,
            params=self.params,
        )

    def predict_tau(self, tau: float) -> Tuple[np.ndarray, np.ndarray]:
        """Predict blind pose at elapsed time tau in seconds."""
        if not self.is_failed or self.frozen_state is None:
            raise RuntimeError("Cannot predict blind pose before failure is declared.")

        return predict_blind_pose(
            self.frozen_state.p,
            self.frozen_state.v,
            self.frozen_state.q,
            self.frozen_state.omega_s,
            tau=tau,
            params=self.params,
        )

    def generate_blind_poses(
        self,
        start_t_ns: int,
        dt_ns: int,
        duration_s: float = 5.0,
    ) -> List[Tuple[int, np.ndarray, np.ndarray]]:
        """Generate extrapolated poses for --append-blind mode up to duration_s.

        Args:
            start_t_ns: Timestamp of the last emitted sample.
            dt_ns: Nominal sample interval in nanoseconds.
            duration_s: Duration of blind extrapolation in seconds (default 5.0 s).

        Returns:
            List of (t_ns, p, q) tuples.
        """
        n_steps = int(round(float(duration_s) * 1e9 / float(dt_ns)))
        poses: List[Tuple[int, np.ndarray, np.ndarray]] = []

        for step in range(1, n_steps + 1):
            t_step_ns = int(start_t_ns) + step * int(dt_ns)
            p, q = self.predict(t_step_ns)
            poses.append((t_step_ns, p, q))

        return poses

# ==============================================================================
# Module: baseline_naive.py
# ==============================================================================
"""Naive Dead-Reckoning Baseline (SPEC §11.5, §12.2, §12.3).

Initialises tilt from the first 1.0 s of IMU samples (assuming static),
then integrates raw specific force and gyro without bias updates or corrections.
Exhibits quadratic position drift ~ 0.5 * a_bias * t^2.
"""






def _compute_initial_tilt_and_g(a_mean: np.ndarray) -> Tuple[np.ndarray, float]:
    """Compute initial unit quaternion [w, x, y, z] aligning mean specific force to world +z (SPEC §12.2).

    R(q) * (a_mean / |a_mean|) = [0, 0, 1]^T.
    Returns:
      q0: (4,) unit quaternion [w, x, y, z]
      g_mag: float clamped to [9.0, 10.5]
    """
    g_raw = float(np.linalg.norm(a_mean))
    g_mag = float(np.clip(g_raw, 9.0, 10.5))

    v = a_mean / (g_raw if g_raw > 1e-6 else 1.0)  # direction of specific force in body
    u = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # target in world

    # Dot product
    cos_theta = float(np.dot(v, u))

    if cos_theta > 0.999999:
        q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    elif cos_theta < -0.999999:
        # 180 degree rotation around x axis
        q0 = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    else:
        # Rotation from body to world that takes v to u:
        # Axis of rotation in world or body
        axis = np.cross(v, u)
        axis_norm = np.linalg.norm(axis)
        axis = axis / axis_norm
        angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        q0 = np.array(
            [
                np.cos(0.5 * angle),
                axis[0] * np.sin(0.5 * angle),
                axis[1] * np.sin(0.5 * angle),
                axis[2] * np.sin(0.5 * angle),
            ],
            dtype=np.float64,
        )

    q0 = q0 / np.linalg.norm(q0)
    return q0, g_mag


def run_naive_baseline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
) -> None:
    """Run naive strapdown baseline on an IMU dataset."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. Parse IMU CSV
    t_list: List[int] = []
    omega_list: List[List[float]] = []
    acc_list: List[List[float]] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                continue
            try:
                t_val = int(parts[0])
                wx = float(parts[1])
                wy = float(parts[2])
                wz = float(parts[3])
                ax = float(parts[4])
                ay = float(parts[5])
                az = float(parts[6])
                t_list.append(t_val)
                omega_list.append([wx, wy, wz])
                acc_list.append([ax, ay, az])
            except ValueError:
                continue

    n = len(t_list)
    if n == 0:
        raise ValueError(f"No valid IMU samples found in {csv_path}")

    t_arr = np.array(t_list, dtype=np.int64)
    omega_arr = np.array(omega_list, dtype=np.float64)
    acc_arr = np.array(acc_list, dtype=np.float64)

    # 2. Initialisation on first 1.0s (or all if < 1.0s)
    t0_ns = t_arr[0]
    init_mask = (t_arr - t0_ns) <= 1_000_000_000
    if np.sum(init_mask) < 2:
        init_mask[: min(n, 20)] = True

    a_init_mean = np.mean(acc_arr[init_mask], axis=0)
    q0, g_mag = _compute_initial_tilt_and_g(a_init_mean)
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    # 3. State propagation loop
    p = np.zeros(3, dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)
    q = q0.copy()

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        for k in range(n):
            t_curr = t_arr[k]
            w_k = omega_arr[k]
            a_k = acc_arr[k]
            if k == 0:
                dt = 0.0
            else:
                dt = (t_curr - t_arr[k - 1]) * 1e-9

            if dt > 0.0 and dt < 10.0 and not (np.isnan(w_k).any() or np.isnan(a_k).any()):  # Valid time increment and valid data
                # Update orientation
                delta_q = quat_exp(w_k * dt)
                q_new = quat_mult(q, delta_q)
                q_new = q_new / np.linalg.norm(q_new)
                if np.dot(q_new, q) < 0.0:
                    q_new = -q_new

                # Update position & velocity using q_old
                R_old = quat_to_rot_matrix(q)
                a_w = R_old @ a_k + g_w

                p = p + v * dt + 0.5 * a_w * (dt**2)
                v = v + a_w * dt
                q = q_new

            # Format timestamp from int64 nanoseconds
            sec = t_curr // 1_000_000_000
            nsec = t_curr % 1_000_000_000
            t_str = f"{sec}.{nsec:09d}"

            # q is [w, x, y, z], output format requires tx ty tz qx qy qz qw
            w_val, x_val, y_val, z_val = q
            f_out.write(
                f"{t_str} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
            )

        # 4. Handle append-blind if requested
        if append_blind and n > 1:
            median_dt = float(np.median(np.diff(t_arr))) * 1e-9
            blind_steps = int(round(5.0 / median_dt))
            t_last = t_arr[-1]
            dt_ns_step = int(round(median_dt * 1e9))

            for step in range(1, blind_steps + 1):
                t_extrap = t_last + step * dt_ns_step
                p_extrap = p + v * (step * median_dt)
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                t_str = f"{sec}.{nsec:09d}"
                w_val, x_val, y_val, z_val = q
                f_out.write(
                    f"{t_str} {p_extrap[0]:.9f} {p_extrap[1]:.9f} {p_extrap[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
                )



# ==============================================================================
# Module: baseline_mahony.py
# ==============================================================================
"""Mahony Attitude Filter + Dead-Reckoning Baseline (SPEC §11.5).

Uses a non-linear complementary attitude filter (Mahony et al.) to correct
roll and pitch drift using accelerometer-derived gravity direction,
then performs gravity-compensated double integration.
"""







def run_mahony_baseline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    kp: float = 1.0,
    ki: float = 0.05,
    append_blind: bool = False,
) -> None:
    """Run Mahony attitude filter + dead-reckoning baseline on an IMU dataset."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. Parse IMU CSV
    t_list: List[int] = []
    omega_list: List[List[float]] = []
    acc_list: List[List[float]] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                continue
            try:
                t_val = int(parts[0])
                wx = float(parts[1])
                wy = float(parts[2])
                wz = float(parts[3])
                ax = float(parts[4])
                ay = float(parts[5])
                az = float(parts[6])
                t_list.append(t_val)
                omega_list.append([wx, wy, wz])
                acc_list.append([ax, ay, az])
            except ValueError:
                continue

    n = len(t_list)
    if n == 0:
        raise ValueError(f"No valid IMU samples found in {csv_path}")

    t_arr = np.array(t_list, dtype=np.int64)
    omega_arr = np.array(omega_list, dtype=np.float64)
    acc_arr = np.array(acc_list, dtype=np.float64)

    # 2. Initialisation on first 1.0s
    t0_ns = t_arr[0]
    init_mask = (t_arr - t0_ns) <= 1_000_000_000
    if np.sum(init_mask) < 2:
        init_mask[: min(n, 20)] = True

    a_init_mean = np.mean(acc_arr[init_mask], axis=0)
    q0, g_mag = _compute_initial_tilt_and_g(a_init_mean)
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    # 3. State propagation loop
    p = np.zeros(3, dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)
    q = q0.copy()
    e_int = np.zeros(3, dtype=np.float64)

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        for k in range(n):
            t_curr = t_arr[k]
            if k == 0:
                dt = 0.0
            else:
                dt = (t_curr - t_arr[k - 1]) * 1e-9

            if dt > 0.0 and dt < 10.0:
                w_k = omega_arr[k]
                a_k = acc_arr[k]

                # Mahony attitude correction:
                # Direction of gravity in body frame: v_b = R(q)^T * [0, 0, 1]
                R_mat = quat_to_rot_matrix(q)
                v_b = R_mat[2, :]  # third row of R = third column of R^T

                a_norm = np.linalg.norm(a_k)
                if a_norm > 1e-3 and np.abs(a_norm - g_mag) < 3.0:
                    a_b = a_k / a_norm
                    # Error is cross product of measured accel and expected gravity in body frame
                    e = np.cross(a_b, v_b)
                else:
                    e = np.zeros(3, dtype=np.float64)

                e_int += e * dt
                w_corr = w_k + kp * e + ki * e_int

                # Update orientation
                delta_q = quat_exp(w_corr * dt)
                q_new = quat_mult(q, delta_q)
                q_new = q_new / np.linalg.norm(q_new)
                if np.dot(q_new, q) < 0.0:
                    q_new = -q_new

                # Update position and velocity
                a_w = R_mat @ a_k + g_w
                p = p + v * dt + 0.5 * a_w * (dt**2)
                v = v + a_w * dt
                q = q_new

            sec = t_curr // 1_000_000_000
            nsec = t_curr % 1_000_000_000
            t_str = f"{sec}.{nsec:09d}"

            w_val, x_val, y_val, z_val = q
            f_out.write(
                f"{t_str} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
            )

        # 4. Handle append-blind if requested
        if append_blind and n > 1:
            median_dt = float(np.median(np.diff(t_arr))) * 1e-9
            blind_steps = int(round(5.0 / median_dt))
            t_last = t_arr[-1]
            dt_ns_step = int(round(median_dt * 1e9))

            for step in range(1, blind_steps + 1):
                t_extrap = t_last + step * dt_ns_step
                p_extrap = p + v * (step * median_dt)
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                t_str = f"{sec}.{nsec:09d}"
                w_val, x_val, y_val, z_val = q
                f_out.write(
                    f"{t_str} {p_extrap[0]:.9f} {p_extrap[1]:.9f} {p_extrap[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
                )



# ==============================================================================
# Module: pipeline.py
# ==============================================================================
"""Full Reference 15-State ESKF Pipeline and Ablation Modes (SPEC §6.3, §11.5, §12).

Assembles:
1. Streaming tolerant CSV parser (SPEC §12.8)
2. Online static window initialization (SPEC §12.2)
3. Nominal strapdown kinematic propagation (SPEC §12.3)
4. Analytical error-state covariance prediction with quality inflation (SPEC §12.4)
5. Degradation monitor tracking residual variance and signal quality (SPEC §12.6)
6. Tilt measurement updates with gating and quality-scaled noise (SPEC §12.5)
7. Zero Velocity Updates (ZUPT) over sliding window (SPEC §12.5)
8. Gap and invalid-sample handler with sub-stepping and recovery (SPEC §12.7)
9. Sensor failure detector, state rollback, and blind-5s predictor (SPEC §12.7, §12.9)

Ablation modes:
- naive: initial tilt + raw open-loop strapdown integration
- mahony: Mahony attitude filter + double integration
- eskf_basic: 15-state ESKF with fixed noise, no monitor inflation, no gap substepping, no blind model
- eskf_full: full system with all components active

Output format (SPEC §6.3):
    timestamp tx ty tz qx qy qz qw
    where timestamp is formatted as <sec>.<nsec zero-padded to 9 digits> from int64 integer math.
    Positions and quaternion formatted with fixed 9 decimal places.
"""















class ChunkedTrajectory:
    """Collects poses from Pass 1 using bounded-memory chunks and temporary file backing for Pass 2 offline filtering."""

    def __init__(self, chunk_size: int = 8192):
        self.chunk_size = int(chunk_size)
        self._count = 0
        self._idx = 0
        self._t_buf = np.empty(self.chunk_size, dtype=np.int64)
        self._p_buf = np.empty((self.chunk_size, 3), dtype=np.float64)
        self._q_buf = np.empty((self.chunk_size, 4), dtype=np.float64)
        self._f_t = tempfile.NamedTemporaryFile()
        self._f_p = tempfile.NamedTemporaryFile()
        self._f_q = tempfile.NamedTemporaryFile()

    def push(self, t_ns: int, p: Union[np.ndarray, list], q: Union[np.ndarray, list]) -> None:
        self._t_buf[self._idx] = int(t_ns)
        self._p_buf[self._idx] = p
        self._q_buf[self._idx] = q
        self._idx += 1
        self._count += 1
        if self._idx == self.chunk_size:
            self._flush_chunk()

    def write_pose(
        self, t_ns: int, px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float
    ) -> None:
        self._t_buf[self._idx] = int(t_ns)
        self._p_buf[self._idx, 0] = px
        self._p_buf[self._idx, 1] = py
        self._p_buf[self._idx, 2] = pz
        self._q_buf[self._idx, 0] = qx
        self._q_buf[self._idx, 1] = qy
        self._q_buf[self._idx, 2] = qz
        self._q_buf[self._idx, 3] = qw
        self._idx += 1
        self._count += 1
        if self._idx == self.chunk_size:
            self._flush_chunk()

    def _flush_chunk(self) -> None:
        if self._idx > 0:
            self._f_t.write(self._t_buf[:self._idx].tobytes())
            self._f_p.write(self._p_buf[:self._idx].tobytes())
            self._f_q.write(self._q_buf[:self._idx].tobytes())
            self._idx = 0

    def to_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        self._flush_chunk()
        self._f_t.flush()
        self._f_p.flush()
        self._f_q.flush()
        # Free buffer memory
        self._t_buf = None
        self._p_buf = None
        self._q_buf = None

        if self._count == 0:
            return (
                np.empty(0, dtype=np.int64),
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 4), dtype=np.float64),
            )

        t_arr = np.memmap(self._f_t, dtype=np.int64, mode="r+", shape=(self._count,))
        p_arr = np.memmap(self._f_p, dtype=np.float64, mode="r+", shape=(self._count, 3))
        q_arr = np.memmap(self._f_q, dtype=np.float64, mode="r+", shape=(self._count, 4))
        return t_arr, p_arr, q_arr


# Backward-compatible alias
TrajectoryCollector = ChunkedTrajectory


def apply_bounded_workspace_filter(
    pos: np.ndarray, dt_nom: float, cutoff_period: float
) -> np.ndarray:
    count = len(pos)
    if count == 0:
        return pos.copy()

    pos_out = np.array(pos, dtype=np.float64, copy=True)
    invalid_mask = ~np.isfinite(pos_out)
    if np.any(invalid_mask):
        pos_out[invalid_mask] = 0.0

    if count < 15 or dt_nom <= 0.0 or cutoff_period <= 0.0:
        return pos_out - pos_out[0]

    fs = 1.0 / dt_nom
    fc = 1.0 / cutoff_period
    if fc >= 0.5 * fs:
        return pos_out - pos_out[0]

    K = math.tan(math.pi * fc / fs)
    norm = 1.0 + math.sqrt(2.0) * K + K * K
    b0 = 1.0 / norm
    b1 = -2.0 * b0
    b2 = b0
    a1 = 2.0 * (K * K - 1.0) / norm
    a2 = (1.0 - math.sqrt(2.0) * K + K * K) / norm

    for c in range(3):
        # Forward pass
        d1 = (b1 + b2 - a1 - a2) * pos_out[0, c]
        d2 = (b2 - a2) * pos_out[0, c]
        for n in range(count):
            x_n = pos_out[n, c]
            val = b0 * x_n + d1
            d1 = b1 * x_n - a1 * val + d2
            d2 = b2 * x_n - a2 * val
            pos_out[n, c] = val

        # Backward pass
        d1 = (b1 + b2 - a1 - a2) * pos_out[count - 1, c]
        d2 = (b2 - a2) * pos_out[count - 1, c]
        for n in range(count - 1, -1, -1):
            x_n = pos_out[n, c]
            val = b0 * x_n + d1
            d1 = b1 * x_n - a1 * val + d2
            d2 = b2 * x_n - a2 * val
            pos_out[n, c] = val

    pos_out -= pos_out[0]
    return pos_out


class DelayedOutputBuffer:
    """Ring buffer holding delayed output frames to apply smooth kinematic bridge across gaps."""

    def __init__(self, capacity: int = 512, delay_frames: int = 512):
        self.capacity = int(capacity)
        self.delay_frames = int(delay_frames)
        self.buffer = []

    def push(self, t_ns: int, p: np.ndarray, q: np.ndarray, is_gap: bool, sink) -> None:
        if self.delay_frames <= 0:
            sink.write_pose(t_ns, p[0], p[1], p[2], q[1], q[2], q[3], q[0])
            return

        if len(self.buffer) >= self.delay_frames or len(self.buffer) >= self.capacity:
            oldest = self.buffer.pop(0)
            po = oldest['p']
            qo = oldest['q']
            sink.write_pose(oldest['t_ns'], po[0], po[1], po[2], qo[1], qo[2], qo[3], qo[0])

        self.buffer.append({'t_ns': t_ns, 'p': p.copy(), 'q': q.copy(), 'is_gap': is_gap})

    def apply_bridge(self, p_recovered: np.ndarray) -> None:
        if not self.buffer or self.delay_frames <= 0:
            return

        gap_count = 0
        for item in reversed(self.buffer):
            if item['is_gap']:
                gap_count += 1
            else:
                break

        if gap_count == 0:
            return

        last_gap_p = self.buffer[-1]['p']
        delta_p = p_recovered - last_gap_p

        start_idx = len(self.buffer) - gap_count
        for step in range(gap_count):
            idx = start_idx + step
            alpha = float(step + 1) / float(gap_count + 1)
            s = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha
            self.buffer[idx]['p'] = self.buffer[idx]['p'] + delta_p * s
            self.buffer[idx]['is_gap'] = False

    def flush(self, sink) -> None:
        while self.buffer:
            oldest = self.buffer.pop(0)
            po = oldest['p']
            qo = oldest['q']
            sink.write_pose(oldest['t_ns'], po[0], po[1], po[2], qo[1], qo[2], qo[3], qo[0])


def _fallback_initialization(
    t_arr: np.ndarray,
    omega_arr: np.ndarray,
    acc_arr: np.ndarray,
    cfg: Dict[str, Any],
) -> InitResult:
    """Fallback initialization when insufficient samples exist to reach T_init."""
    n_samples = len(t_arr)
    if n_samples == 0:
        a_ref = np.array([0.0, 0.0, 9.80665], dtype=np.float64)
        b_g = np.zeros(3, dtype=np.float64)
        t0 = 0
    else:
        a_ref = np.mean(acc_arr, axis=0) if n_samples > 1 else acc_arr[0]
        b_g = np.zeros(3, dtype=np.float64)
        t0 = int(t_arr[0])

    g_raw = float(np.linalg.norm(a_ref))
    g_min = float(cfg.get("g_mag_clamp_min", 9.0))
    g_max = float(cfg.get("g_mag_clamp_max", 10.5))
    g_mag = float(np.clip(g_raw, g_min, g_max))
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    q0 = compute_initial_tilt(a_ref)
    p0 = np.zeros(3, dtype=np.float64)
    v0 = np.zeros(3, dtype=np.float64)
    b_a0 = np.zeros(3, dtype=np.float64)

    state = NominalState(p=p0, v=v0, q=q0, b_g=b_g, b_a=b_a0)
    P0 = initialize_covariance(cfg, motion_detected=True)

    return InitResult(
        state=state,
        P=P0,
        g_w=g_w,
        g_mag=g_mag,
        motion_detected=True,
        samples_used=max(1, n_samples),
        t0=t0,
    )


def run_eskf_full(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run full 15-state ESKF reference pipeline (SPEC §11.5, §12)."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cfg = load_params(params)

    cov_floor = float(cfg.get("cov_diag_floor", 1e-12))
    cov_cap = float(cfg.get("cov_diag_cap", 1e6))
    zupt_window_s = float(cfg.get("zupt_window_s", 0.25))

    initializer = OnlineInitializer(params=cfg)
    gap_handler = GapHandler(params=cfg)
    monitor = DegradationMonitor(params=cfg)
    blind_predictor = BlindPredictor(params=cfg)
    stat_buf = StationaryWindowBuffer(capacity=int(cfg.get("ring_buffer_size", 256)))
    tilt_filter = Butterworth2LowPass()
    delay_buf = DelayedOutputBuffer(
        capacity=512,
        delay_frames=int(cfg.get("delayed_output_frames", 512))
    )
    was_in_gap = False

    # Preallocated initial buffer for zero dynamic allocations during startup
    max_init_cap = OnlineInitializer.MAX_BUFFER_SAMPLES
    init_t = np.empty(max_init_cap, dtype=np.int64)
    init_omega = np.empty((max_init_cap, 3), dtype=np.float64)
    init_acc = np.empty((max_init_cap, 3), dtype=np.float64)
    init_valid = np.empty(max_init_cap, dtype=bool)
    init_count = 0

    is_initialized = False
    init_res: Optional[InitResult] = None

    state: Optional[NominalState] = None
    P: Optional[np.ndarray] = None
    g_w: Optional[np.ndarray] = None
    g_mag: float = 9.80665
    t0_ns: int = 0
    prev_t_ns: int = 0
    vibe_ema: float = 0.0
    ped_energy: float = 0.0
    dyn_acc_ema: float = 0.0
    prev_acc_raw = np.zeros(3, dtype=np.float64)
    has_prev_acc: bool = False

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        collector = ChunkedTrajectory()

        def process_full_step(t_curr: int, omega_curr: np.ndarray, acc_curr: np.ndarray) -> None:
            nonlocal state, P, prev_t_ns, was_in_gap
            nonlocal vibe_ema, ped_energy, dyn_acc_ema, prev_acc_raw, has_prev_acc
            assert state is not None and P is not None and g_w is not None

            if blind_predictor.is_failed:
                p_blind, q_blind = blind_predictor.predict(t_curr)
                delay_buf.push(t_curr, p_blind, q_blind, False, collector)
                prev_t_ns = t_curr
                return

            step_res = gap_handler.process_sample(t_curr, omega_curr, acc_curr, state, P)

            if step_res.case == GapCase.ZERO_OR_NEGATIVE_DT:
                delay_buf.push(t_curr, state.p, state.q, False, collector)
                return

            if step_res.case in (GapCase.GAP, GapCase.INVALID_SAMPLE):
                state = step_res.state
                P = step_res.P
                was_in_gap = True
                failed = blind_predictor.update(t_curr, is_valid=False)
                if failed or blind_predictor.is_failed:
                    p_out, q_out = blind_predictor.predict(t_curr)
                else:
                    p_out, q_out = state.p, state.q

                delay_buf.push(t_curr, p_out, q_out, True, collector)
                prev_t_ns = t_curr
                return

            # Normal step
            dt = (t_curr - prev_t_ns) * 1e-9
            prev_t_ns = t_curr

            acc_arr = np.asarray(acc_curr, dtype=np.float64).reshape(3)
            if np.all(np.isfinite(acc_arr)):
                if not has_prev_acc:
                    prev_acc_raw = acc_arr.copy()
                    has_prev_acc = True
                else:
                    d_a = float(np.linalg.norm(acc_arr - prev_acc_raw))
                    if np.isfinite(d_a):
                        vibe_ema = 0.98 * vibe_ema + 0.02 * d_a
                        dev_a = abs(float(np.linalg.norm(acc_arr)) - g_mag)
                        ped_energy = 0.99 * ped_energy + 0.01 * (dev_a * dev_a)
                        if monitor.quality >= 0.5 and dev_a < 12.0:
                            a_excess = max(0.0, float(np.linalg.norm(acc_arr)) - g_mag)
                            dyn_acc_ema = 0.95 * dyn_acc_ema + 0.05 * a_excess
                        prev_acc_raw = acc_arr.copy()

            flight_thresh = float(cfg.get("flight_vibe_thresh", 0.4))
            ped_thresh = float(cfg.get("ped_energy_thresh", 0.2))
            is_flight = vibe_ema > flight_thresh
            is_pedestrian = (not is_flight) and (ped_energy > ped_thresh)

            dyn_env = float(cfg.get("max_velocity_mps", 3.0)) + (1.0 if is_flight else 0.0) * dyn_acc_ema
            if dyn_env > 15.0:
                dyn_env = 15.0
            max_v = dyn_env if is_flight else (float(cfg.get("ped_max_velocity_mps", 3.5)) if is_pedestrian else dyn_env)

            state = strapdown_step(state, omega_curr, acc_curr, dt, g_w)
            F = build_f_matrix(state, omega_curr, acc_curr, dt)
            Q = build_q_discrete(dt, cfg, kappa=monitor.kappa)
            P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

            t_sec = (t_curr - t0_ns) * 1e-9
            acc_filt = tilt_filter.step(acc_curr) if cfg.get("enable_iir_filter", False) else acc_curr

            stat_buf.add_sample(t_sec, omega_curr, acc_curr, state.b_g)
            bool_stat = stat_buf.is_stationary(g_mag=g_mag, quality=monitor.quality, params=cfg)
            gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)
            omega_stat_mean = np.mean(gyro_win, axis=0) if (bool_stat and len(gyro_win) > 0) else None
            monitor.step(t_sec, omega_curr, acc_curr, is_stationary=bool_stat, omega_static_mean=omega_stat_mean)

            if step_res.apply_updates:
                state, P, _, _ = tilt_update(
                    state, P, acc_filt, omega_curr,
                    g_mag=g_mag, quality=monitor.quality,
                    params=cfg, r_scale=step_res.r_scale,
                )
                if bool_stat:
                    state, P, _, _ = zupt_step(
                        state, P, gyro_win, acc_win,
                        g_mag=g_mag, quality=monitor.quality,
                        params=cfg, r_scale=step_res.r_scale,
                    )
                elif is_flight and bool(cfg.get("enable_rotor_drag", True)):
                    state, P, _ = apply_rotor_drag_update(
                        state, P, acc_curr, omega_curr, g_w,
                        kd=float(cfg.get("rotor_drag_kd", 0.22)),
                        sigma=float(cfg.get("rotor_drag_sigma", 0.8)),
                        gate_threshold=float(cfg.get("rotor_drag_gate", 12.0)),
                        params=cfg,
                    )
                elif is_pedestrian and bool(cfg.get("enable_pedestrian_prior", True)):
                    state, P, _ = apply_pedestrian_prior(
                        state, P,
                        sigma_lat=float(cfg.get("ped_sigma_lat", 0.5)),
                        sigma_vert=float(cfg.get("ped_sigma_vert", 0.3)),
                        gate_threshold=float(cfg.get("ped_gate_threshold", 9.0)),
                        params=cfg,
                    )
                state, P, _ = apply_velocity_bounding(state, P, max_speed=max_v, params=cfg)

            if was_in_gap:
                delay_buf.apply_bridge(state.p)
                was_in_gap = False

            blind_predictor.update(
                t_curr, is_valid=True, state=state, omega_s=gap_handler.omega_s
            )
            delay_buf.push(t_curr, state.p, state.q, False, collector)

        with parse_imu_stream(csv_path, params=cfg) as stream:
            for t_curr, omega_curr, acc_curr, is_valid_curr, _ in stream:
                if not is_initialized:
                    if init_count < max_init_cap:
                        init_t[init_count] = t_curr
                        init_omega[init_count] = omega_curr
                        init_acc[init_count] = acc_curr
                        init_valid[init_count] = is_valid_curr
                        init_count += 1

                    ready = initializer.feed(t_curr, omega_curr, acc_curr, is_valid=is_valid_curr)
                    if ready:
                        is_initialized = True
                        init_res = initializer.get_result()
                        assert init_res is not None
                        state = init_res.state.copy()
                        P = init_res.P.copy()
                        g_w = init_res.g_w.copy()
                        g_mag = float(init_res.g_mag)
                        t0_ns = int(init_res.t0)

                        valid_acc = init_acc[:init_count][init_valid[:init_count]]
                        a_mean = np.mean(valid_acc, axis=0) if len(valid_acc) > 0 else np.array([0.0, 0.0, 9.80665], dtype=np.float64)
                        tilt_filter.reset(a_mean)

                        # Process buffered samples 0 .. init_count - 1
                        for idx in range(init_count):
                            t_smp = int(init_t[idx])
                            w_smp = init_omega[idx]
                            a_smp = init_acc[idx]

                            if idx == 0:
                                prev_t_ns = t_smp
                                blind_predictor.record_reliable_state(
                                    t_smp, state=state, omega_s=np.zeros(3, dtype=np.float64)
                                )
                                gap_handler.process_sample(t_smp, w_smp, a_smp, state, P)
                                delay_buf.push(t_smp, state.p, state.q, False, collector)
                                continue

                            process_full_step(t_smp, w_smp, a_smp)
                        continue
                    continue

                process_full_step(t_curr, omega_curr, acc_curr)

        # Handle file shorter than T_init fallback
        if not is_initialized and init_count > 0:
            ready = initializer.finalize_fallback()
            if ready:
                init_res = initializer.get_result()
            else:
                init_res = _fallback_initialization(
                    init_t[:init_count], init_omega[:init_count], init_acc[:init_count], cfg
                )

            assert init_res is not None
            state = init_res.state.copy()
            P = init_res.P.copy()
            g_w = init_res.g_w.copy()
            g_mag = float(init_res.g_mag)
            t0_ns = int(init_res.t0)

            for idx in range(init_count):
                t_smp = int(init_t[idx])
                w_smp = init_omega[idx]
                a_smp = init_acc[idx]

                if idx == 0:
                    prev_t_ns = t_smp
                else:
                    dt = (t_smp - prev_t_ns) * 1e-9
                    prev_t_ns = t_smp
                    if dt > 0.0 and init_valid[idx]:
                        state = strapdown_step(state, w_smp, a_smp, dt, g_w)
                if not np.all(np.isfinite(state.p)):
                    state.p = np.zeros(3, dtype=np.float64)
                if not np.all(np.isfinite(state.q)):
                    state.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                delay_buf.push(t_smp, state.p, state.q, False, collector)

        delay_buf.flush(collector)

        # Pass 2: Offline drift-aware bounded-workspace filter
        dt_nom = float(gap_handler.dt_nom if gap_handler.dt_nom > 0.0 else 0.005)
        t_ns_arr, pos_arr, quat_arr = collector.to_arrays()
        stream_count = len(t_ns_arr)

        # Determine if IMU failure occurred during the stream
        active_count = stream_count
        p_raw_split = np.zeros(3, dtype=np.float64)
        failed_in_stream = False

        if blind_predictor.is_failed and stream_count > 0:
            t_fail = blind_predictor.failure_time_ns
            for i in range(stream_count):
                if t_ns_arr[i] >= t_fail:
                    active_count = i
                    failed_in_stream = True
                    if i > 0:
                        p_raw_split = pos_arr[i - 1].copy()
                    break

        if bool(cfg.get("enable_two_pass_filter", True)) and active_count > 0:
            cutoff_T = float(cfg.get("two_pass_cutoff_period", 45.0))
            pos_arr[:active_count] = apply_bounded_workspace_filter(pos_arr[:active_count], dt_nom, cutoff_T)

        if failed_in_stream and active_count > 0 and active_count < stream_count:
            p_filtered_split = pos_arr[active_count - 1]
            delta_align = p_filtered_split - p_raw_split
            pos_arr[active_count:] += delta_align

        for i in range(len(t_ns_arr)):
            t_ns_i = int(t_ns_arr[i])
            sec = t_ns_i // 1_000_000_000
            nsec = t_ns_i % 1_000_000_000
            p_i = pos_arr[i]
            q_i = quat_arr[i]
            f_out.write(
                f"{sec}.{nsec:09d} {p_i[0]:.9f} {p_i[1]:.9f} {p_i[2]:.9f} "
                f"{q_i[0]:.9f} {q_i[1]:.9f} {q_i[2]:.9f} {q_i[3]:.9f}\n"
            )

        # Append blind 5.0s extrapolation from clean filtered endpoint
        if append_blind and prev_t_ns > 0 and stream_count > 0:
            if not blind_predictor.is_failed:
                blind_predictor.notify_stream_end(prev_t_ns)

            p_last_filtered = pos_arr[-1]
            p_raw_endpoint = blind_predictor.frozen_state.p
            delta_endpoint_align = p_last_filtered - p_raw_endpoint

            dt_ns_step = max(1, int(round(dt_nom * 1e9)))
            blind_poses = blind_predictor.generate_blind_poses(prev_t_ns, dt_ns_step, duration_s=5.0)

            for t_extrap, p_extrap, q_extrap in blind_poses:
                p_final = p_extrap + delta_endpoint_align
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                f_out.write(
                    f"{sec}.{nsec:09d} {p_final[0]:.9f} {p_final[1]:.9f} {p_final[2]:.9f} "
                    f"{q_extrap[1]:.9f} {q_extrap[2]:.9f} {q_extrap[3]:.9f} {q_extrap[0]:.9f}\n"
                )


def run_eskf_basic(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run eskf_basic ablation: 15-state ESKF with fixed noise, no monitor inflation, no gap substepping, no blind model."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cfg = load_params(params)

    cov_floor = float(cfg.get("cov_diag_floor", 1e-12))
    cov_cap = float(cfg.get("cov_diag_cap", 1e6))
    zupt_window_s = float(cfg.get("zupt_window_s", 0.25))

    initializer = OnlineInitializer(params=cfg)
    stat_buf = StationaryWindowBuffer(capacity=int(cfg.get("ring_buffer_size", 64)))

    max_init_cap = OnlineInitializer.MAX_BUFFER_SAMPLES
    init_t = np.empty(max_init_cap, dtype=np.int64)
    init_omega = np.empty((max_init_cap, 3), dtype=np.float64)
    init_acc = np.empty((max_init_cap, 3), dtype=np.float64)
    init_valid = np.empty(max_init_cap, dtype=bool)
    init_count = 0

    is_initialized = False
    init_res: Optional[InitResult] = None

    state: Optional[NominalState] = None
    P: Optional[np.ndarray] = None
    g_w: Optional[np.ndarray] = None
    g_mag: float = 9.80665
    t0_ns: int = 0
    prev_t_ns: int = 0
    vibe_ema: float = 0.0
    ped_energy: float = 0.0
    dyn_acc_ema: float = 0.0
    prev_acc_raw = np.zeros(3, dtype=np.float64)
    has_prev_acc: bool = False
    dt_samples = []

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        with parse_imu_stream(csv_path, params=cfg) as stream:
            for t_curr, omega_curr, acc_curr, is_valid_curr, _ in stream:
                if not is_initialized:
                    if init_count < max_init_cap:
                        init_t[init_count] = t_curr
                        init_omega[init_count] = omega_curr
                        init_acc[init_count] = acc_curr
                        init_valid[init_count] = is_valid_curr
                        init_count += 1

                    ready = initializer.feed(t_curr, omega_curr, acc_curr, is_valid=is_valid_curr)
                    if ready:
                        is_initialized = True
                        init_res = initializer.get_result()
                        assert init_res is not None
                        state = init_res.state.copy()
                        P = init_res.P.copy()
                        g_w = init_res.g_w.copy()
                        g_mag = float(init_res.g_mag)
                        t0_ns = int(init_res.t0)

                        for idx in range(init_count):
                            t_smp = int(init_t[idx])
                            w_smp = init_omega[idx]
                            a_smp = init_acc[idx]
                            v_smp = bool(init_valid[idx])

                            if idx == 0:
                                prev_t_ns = t_smp
                                sec = t_smp // 1_000_000_000
                                nsec = t_smp % 1_000_000_000
                                p_s = state.p
                                q_s = state.q
                                f_out.write(
                                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                                )
                                continue

                            dt = (t_smp - prev_t_ns) * 1e-9
                            prev_t_ns = t_smp

                            if dt <= 0.0 or not v_smp:
                                # Skip integration on negative dt or invalid sample
                                pass
                            else:
                                if len(dt_samples) < 200:
                                    dt_samples.append(dt)
                                state = strapdown_step(state, w_smp, a_smp, dt, g_w)
                                F = build_f_matrix(state, w_smp, a_smp, dt)
                                Q = build_q_discrete(dt, cfg, kappa=1.0)  # Fixed noise, no inflation
                                P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

                                t_sec = (t_smp - t0_ns) * 1e-9
                                stat_buf.add_sample(t_sec, w_smp, a_smp, state.b_g)
                                is_stat = stat_buf.is_stationary(g_mag=g_mag, quality=1.0, params=cfg)
                                gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)

                                state, P, _, _ = tilt_update(
                                    state, P, a_smp, w_smp,
                                    g_mag=g_mag, quality=1.0,
                                    params=cfg, r_scale=1.0,
                                )
                                if is_stat:
                                    state, P, _, _ = zupt_step(
                                        state, P, gyro_win, acc_win,
                                        g_mag=g_mag, quality=1.0,
                                        params=cfg, r_scale=1.0,
                                    )

                            sec = t_smp // 1_000_000_000
                            nsec = t_smp % 1_000_000_000
                            p_s = state.p
                            q_s = state.q
                            f_out.write(
                                f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                                f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                            )
                        continue
                    continue

                # Once initialized
                assert state is not None and P is not None and g_w is not None

                dt = (t_curr - prev_t_ns) * 1e-9
                prev_t_ns = t_curr

                if dt <= 0.0 or not is_valid_curr:
                    pass
                else:
                    if len(dt_samples) < 200:
                        dt_samples.append(dt)
                    state = strapdown_step(state, omega_curr, acc_curr, dt, g_w)
                    F = build_f_matrix(state, omega_curr, acc_curr, dt)
                    Q = build_q_discrete(dt, cfg, kappa=1.0)
                    P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

                    t_sec = (t_curr - t0_ns) * 1e-9
                    stat_buf.add_sample(t_sec, omega_curr, acc_curr, state.b_g)
                    is_stat = stat_buf.is_stationary(g_mag=g_mag, quality=1.0, params=cfg)
                    gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)

                    state, P, _, _ = tilt_update(
                        state, P, acc_curr, omega_curr,
                        g_mag=g_mag, quality=1.0,
                        params=cfg, r_scale=1.0,
                    )
                    if is_stat:
                        state, P, _, _ = zupt_step(
                            state, P, gyro_win, acc_win,
                            g_mag=g_mag, quality=1.0,
                            params=cfg, r_scale=1.0,
                        )

                sec = t_curr // 1_000_000_000
                nsec = t_curr % 1_000_000_000
                p_s = state.p
                q_s = state.q
                f_out.write(
                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                )

        # Fallback if stream was shorter than T_init
        if not is_initialized and init_count > 0:
            ready = initializer.finalize_fallback()
            if ready:
                init_res = initializer.get_result()
            else:
                init_res = _fallback_initialization(
                    init_t[:init_count], init_omega[:init_count], init_acc[:init_count], cfg
                )
            assert init_res is not None
            state = init_res.state.copy()
            g_w = init_res.g_w.copy()
            for idx in range(init_count):
                t_smp = int(init_t[idx])
                if idx == 0:
                    prev_t_ns = t_smp
                else:
                    dt = (t_smp - prev_t_ns) * 1e-9
                    prev_t_ns = t_smp
                    if dt > 0.0 and init_valid[idx]:
                        state = strapdown_step(state, init_omega[idx], init_acc[idx], dt, g_w)
                if not np.all(np.isfinite(state.p)):
                    state.p = np.zeros(3, dtype=np.float64)
                if not np.all(np.isfinite(state.q)):
                    state.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                sec = t_smp // 1_000_000_000
                nsec = t_smp % 1_000_000_000
                p_s = state.p
                q_s = state.q
                f_out.write(
                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                )

        # Append blind (no blind model: naive constant velocity extrapolation)
        if append_blind and prev_t_ns > 0 and state is not None:
            dt_nom = float(np.median(dt_samples)) if dt_samples else 0.005
            dt_ns_step = max(1, int(round(dt_nom * 1e9)))
            blind_steps = int(round(5.0 / dt_nom))
            for step in range(1, blind_steps + 1):
                t_extrap = prev_t_ns + step * dt_ns_step
                p_extrap = state.p + state.v * (step * dt_nom)
                q_extrap = state.q
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                f_out.write(
                    f"{sec}.{nsec:09d} {p_extrap[0]:.9f} {p_extrap[1]:.9f} {p_extrap[2]:.9f} "
                    f"{q_extrap[1]:.9f} {q_extrap[2]:.9f} {q_extrap[3]:.9f} {q_extrap[0]:.9f}\n"
                )


def run_pipeline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    ablation: str = "eskf_full",
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run specified estimator ablation mode on IMU input.

    Args:
        imu_input: Path to IMU CSV or sequence folder.
        traj_output: Path to write estimated trajectory file.
        ablation: Ablation mode: 'naive', 'mahony', 'eskf_basic', 'eskf_full'.
        append_blind: Whether to extrapolate 5.0 s poses after stream ends.
        params: Optional dictionary of filter parameters.
    """
    mode = ablation.lower().strip()
    if mode == "naive":
        run_naive_baseline(imu_input, traj_output, append_blind=append_blind)
    elif mode == "mahony":
        run_mahony_baseline(imu_input, traj_output, append_blind=append_blind)
    elif mode == "eskf_basic":
        run_eskf_basic(imu_input, traj_output, append_blind=append_blind, params=params)
    elif mode == "eskf_full":
        run_eskf_full(imu_input, traj_output, append_blind=append_blind, params=params)
    else:
        raise ValueError(
            f"Unknown ablation mode '{ablation}'. Supported: naive, mahony, eskf_basic, eskf_full"
        )

# ==============================================================================
# Standalone CLI and Input Resolution
# ==============================================================================

def _is_zip(p: Path) -> bool:
    try:
        with open(p, "rb") as f:
            return f.read(2) == b"PK"
    except Exception:
        return False


def _extract_zip_csv(p: Path) -> str:
    tmp = tempfile.mkdtemp(prefix="imu_")
    with zipfile.ZipFile(p) as z:
        names = z.namelist()
        candidates = ["imu0/data.csv", "mav0/imu0/data.csv", "data.csv"]
        chosen = None
        for cand in candidates:
            matches = [n for n in names if n.endswith(cand)]
            if matches:
                chosen = matches[0]
                break
        if chosen is None:
            matches = [n for n in names if n.endswith(".csv")]
            if matches:
                chosen = matches[0]
        if chosen is None:
            raise FileNotFoundError(f"No CSV found inside zip: {p}")
        out = os.path.join(tmp, "data.csv")
        with z.open(chosen) as src, open(out, "wb") as dst:
            dst.write(src.read())
    return out


def resolve_imu_csv(raw: Union[str, Path]) -> str:
    p = Path(raw)
    if not p.exists():
        raise FileNotFoundError(f"Input not found: {raw}")
    if p.is_dir():
        for cand in [p / "imu0" / "data.csv", p / "mav0" / "imu0" / "data.csv", p / "data.csv"]:
            if cand.is_file():
                return str(cand)
        raise FileNotFoundError(f"No IMU CSV in directory: {raw}")
    if p.is_file() and (str(raw).lower().endswith(".zip") or _is_zip(p)):
        return _extract_zip_csv(p)
    return str(p)


class _CustomParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        sys.exit(1)


def main():
    parser = _CustomParser(description="Track 03 Path Finder Estimator")
    parser.add_argument("input", nargs="?", default=None, help="Input IMU CSV, ZIP, or directory")
    parser.add_argument("output", nargs="?", default="estimated_trajectory.txt", help="Output trajectory path")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate blind window after stream")
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

    try:
        csv_path = resolve_imu_csv(args.input)
        run_pipeline(
            csv_path,
            args.output,
            ablation=args.ablation,
            append_blind=args.append_blind,
            params=args.params,
        )
    except Exception as e:
        sys.stderr.write(f"Execution failed: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
