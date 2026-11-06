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

from enum import IntFlag
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, TextIO, Tuple, Union
import numpy as np


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


def get_default_params_path() -> Path:
    """Return path to configs/params.json."""
    return Path(__file__).resolve().parent.parent.parent / "configs" / "params.json"


def load_params(params: Optional[Union[Dict[str, Any], str, Path]] = None) -> Dict[str, Any]:
    """Load parameter dictionary or return default parameters from configs/params.json."""
    if params is None:
        params_path = get_default_params_path()
        if params_path.is_file():
            with open(params_path, "r", encoding="utf-8") as f:
                return json.load(f)
        return {"invalid_gyro_max": 35.0, "invalid_acc_max": 160.0}
    if isinstance(params, (str, Path)):
        with open(params, "r", encoding="utf-8") as f:
            return json.load(f)
    return dict(params)


def is_comment_or_blank(line: str) -> bool:
    """Check if line is a comment (#...) or empty/blank whitespace."""
    s = line.strip()
    return len(s) == 0 or s.startswith("#")


def parse_imu_line(
    line: str,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[Optional[int], np.ndarray, np.ndarray, bool, int]:
    """Pure function to parse a single IMU line in EuRoC format.

    Format: timestamp [ns], w_RS_S_x, w_RS_S_y, w_RS_S_z, a_RS_S_x, a_RS_S_y, a_RS_S_z

    Args:
        line: Raw text line.
        params: Configuration dictionary containing invalid_gyro_max and invalid_acc_max.

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

    # 2. Parse 6 measurement fields: wx, wy, wz, ax, ay, az
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

            t_ns, omega, acc, is_valid, flags = parse_imu_line(raw_line, self.params)

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
) -> IMUStream:
    """Create streaming IMU CSV iterator yielding (t_ns, omega, acc, is_valid, flags)."""
    return IMUStream(lines_or_file, params=params, stats=stats)
