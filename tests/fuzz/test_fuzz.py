"""Adversarial Fuzz Test Suite for C++ IMU Estimator Binary.

Tests `src/cpp/build/main` against edge cases, corrupted inputs,
extreme physical spikes, malformed files, and adversarial data.
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from typing import List

import pytest

from tools.gen_fuzz_cases import FuzzCase, generate_fuzz_cases, get_all_fuzz_cases
from tools.validate_trajectory import validate_trajectory

CPP_BINARY = Path("src/cpp/build/main")


@pytest.fixture(scope="session", autouse=True)
def ensure_cpp_binary() -> None:
    """Ensure the C++ binary is built before running fuzz tests."""
    if not CPP_BINARY.exists():
        res = subprocess.run(["make", "-C", "src/cpp"], capture_output=True, text=True)
        assert res.returncode == 0, f"Failed to build C++ binary: {res.stderr}"
    assert CPP_BINARY.exists(), f"C++ binary {CPP_BINARY} not found"


ALL_FUZZ_CASES = get_all_fuzz_cases()
FUZZ_CASE_IDS = [case.name for case in ALL_FUZZ_CASES]


@pytest.mark.parametrize("case", ALL_FUZZ_CASES, ids=FUZZ_CASE_IDS)
def test_fuzz_case(tmp_path: Path, case: FuzzCase) -> None:
    """Run C++ binary on an adversarial case and verify robustness."""
    in_path = tmp_path / f"{case.name}.csv"
    out_path = tmp_path / f"{case.name}_out.txt"

    in_path.write_bytes(case.content)

    # Execute binary with a strict 5.0s timeout
    proc = subprocess.run(
        [str(CPP_BINARY), str(in_path), str(out_path)],
        capture_output=True,
        timeout=5.0,
    )

    # 1. Verify clean termination (no SIGSEGV 139, SIGABRT 134, SIGFPE 136, SIGBUS 135)
    # Subprocess return codes are negative on signals (-11, -6, etc.) or 128+signum
    CRASH_CODES = {-11, 139, -6, 134, -8, 136, -7, 135}
    assert proc.returncode not in CRASH_CODES, (
        f"Case '{case.name}' crashed with returncode {proc.returncode}!\n"
        f"Stderr: {proc.stderr.decode('utf-8', errors='replace')}"
    )
    stderr_lower = proc.stderr.decode("utf-8", errors="replace").lower()
    assert "sanitizer" not in stderr_lower, (
        f"Sanitizer error in case '{case.name}':\n{proc.stderr.decode('utf-8', errors='replace')}"
    )
    assert "runtime error:" not in stderr_lower, (
        f"UBSan error in case '{case.name}':\n{proc.stderr.decode('utf-8', errors='replace')}"
    )
    assert proc.returncode in case.expected_exit_codes, (
        f"Case '{case.name}' returned {proc.returncode}, expected {case.expected_exit_codes}.\n"
        f"Stderr: {proc.stderr.decode('utf-8', errors='replace')}"
    )

    # 2. Check output trajectory if created
    if out_path.exists() and out_path.stat().st_size > 0:
        content = out_path.read_text(encoding="utf-8", errors="replace")
        content_lower = content.lower()

        # Must not contain NaN or Inf anywhere
        assert "nan" not in content_lower, f"Case '{case.name}' output contains NaN:\n{content[:500]}"
        assert "inf" not in content_lower, f"Case '{case.name}' output contains Inf:\n{content[:500]}"

        lines = [line.strip() for line in content.splitlines() if line.strip()]

        if case.expected_rows is not None:
            assert len(lines) == case.expected_rows, (
                f"Case '{case.name}' expected {case.expected_rows} rows, got {len(lines)}"
            )

        # Verify formatting: 8 columns per row
        for idx, line in enumerate(lines):
            tokens = line.split()
            assert len(tokens) == 8, (
                f"Case '{case.name}' line {idx} does not have 8 columns: {line}"
            )

            # Col 0: timestamp
            t_val = float(tokens[0])
            assert math.isfinite(t_val), f"Non-finite timestamp at line {idx}"

            # Cols 1-3: tx, ty, tz
            tx, ty, tz = float(tokens[1]), float(tokens[2]), float(tokens[3])
            assert math.isfinite(tx) and math.isfinite(ty) and math.isfinite(tz), (
                f"Non-finite position at line {idx}: {tx}, {ty}, {tz}"
            )

            # Cols 4-7: qx, qy, qz, qw
            qx, qy, qz, qw = float(tokens[4]), float(tokens[5]), float(tokens[6]), float(tokens[7])
            assert all(math.isfinite(q) for q in (qx, qy, qz, qw)), (
                f"Non-finite quaternion at line {idx}: {qx}, {qy}, {qz}, {qw}"
            )

            # Quaternion unit norm check
            q_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
            assert abs(q_norm_sq - 1.0) < 0.05, (
                f"Case '{case.name}' line {idx} quaternion norm squared {q_norm_sq} deviates from 1.0"
            )

    # 3. If expected to pass full trajectory validation, check with tools/validate_trajectory.py
    if case.expect_validator_pass and out_path.exists() and out_path.stat().st_size > 0:
        valid, errors = validate_trajectory(in_path, out_path)
        assert valid, f"Case '{case.name}' failed trajectory validation:\n" + "\n".join(errors)


KEY_CASES_FOR_MODES = [
    "empty_file",
    "single_row",
    "nan_burst",
    "inf_burst",
    "accel_1000g_spike",
    "huge_timestamp_gap",
    "random_binary_noise",
]


@pytest.mark.parametrize("mode", ["eskf_full", "eskf_basic", "mahony", "naive"])
@pytest.mark.parametrize("case_name", KEY_CASES_FOR_MODES)
def test_fuzz_ablation_modes(tmp_path: Path, mode: str, case_name: str) -> None:
    """Verify that all ablation modes handle adversarial edge cases cleanly."""
    case_map = {c.name: c for c in ALL_FUZZ_CASES}
    case = case_map[case_name]

    in_path = tmp_path / f"{case.name}.csv"
    out_path = tmp_path / f"{case.name}_{mode}_out.txt"

    in_path.write_bytes(case.content)

    # Test standard ablation run
    cmd = [str(CPP_BINARY), str(in_path), str(out_path), f"--ablation={mode}"]
    proc = subprocess.run(cmd, capture_output=True, timeout=5.0)

    stderr_lower = proc.stderr.decode("utf-8", errors="replace").lower()
    assert "sanitizer" not in stderr_lower, f"Sanitizer error in mode {mode} on {case_name}: {proc.stderr}"
    assert "runtime error:" not in stderr_lower, f"UBSan error in mode {mode} on {case_name}: {proc.stderr}"
    assert proc.returncode in (0, 1), f"Mode {mode} on case {case_name} crashed: returncode {proc.returncode}"

    # For full and basic ESKF pipelines, verify gap/outlier rejection guarantees no NaNs or Infs
    if mode in ("eskf_full", "eskf_basic"):
        if out_path.exists() and out_path.stat().st_size > 0:
            content = out_path.read_text(encoding="utf-8", errors="replace").lower()
            assert "nan" not in content, f"Mode {mode} on {case_name} produced NaN"
            assert "inf" not in content, f"Mode {mode} on {case_name} produced Inf"


@pytest.mark.parametrize("mode", ["eskf_full", "eskf_basic", "mahony", "naive"])
def test_fuzz_append_blind(tmp_path: Path, mode: str) -> None:
    """Verify --append-blind on edge cases terminates cleanly without crash or NaNs."""
    case_map = {c.name: c for c in ALL_FUZZ_CASES}
    case = case_map["single_row"]

    in_path = tmp_path / "single_row.csv"
    out_path = tmp_path / f"single_row_{mode}_blind_out.txt"

    in_path.write_bytes(case.content)

    cmd = [str(CPP_BINARY), str(in_path), str(out_path), f"--ablation={mode}", "--append-blind"]
    proc = subprocess.run(cmd, capture_output=True, timeout=5.0)

    stderr_lower = proc.stderr.decode("utf-8", errors="replace").lower()
    assert "sanitizer" not in stderr_lower, f"Sanitizer error in append-blind mode {mode}: {proc.stderr}"
    assert "runtime error:" not in stderr_lower, f"UBSan error in append-blind mode {mode}: {proc.stderr}"
    assert proc.returncode in (0, 1), f"--append-blind in mode {mode} crashed: returncode {proc.returncode}"

    if out_path.exists() and out_path.stat().st_size > 0:
        content = out_path.read_text(encoding="utf-8", errors="replace").lower()
        assert "nan" not in content, f"--append-blind in mode {mode} produced NaN"
        assert "inf" not in content, f"--append-blind in mode {mode} produced Inf"


def test_fuzz_cli_rejection(tmp_path: Path) -> None:
    """Verify CLI gracefully rejects invalid invocations and missing files without crashing."""
    out_path = tmp_path / "out.txt"

    # Missing arguments
    res1 = subprocess.run([str(CPP_BINARY)], capture_output=True, timeout=5.0)
    assert res1.returncode == 1, f"Expected 1, got {res1.returncode}"

    # Non-existent input file
    res2 = subprocess.run(
        [str(CPP_BINARY), "/tmp/this_file_does_not_exist_at_all.csv", str(out_path)],
        capture_output=True,
        timeout=5.0,
    )
    assert res2.returncode == 1, f"Expected 1, got {res2.returncode}"

    # Invalid ablation argument (missing argument value)
    res3 = subprocess.run(
        [str(CPP_BINARY), "/dev/null", str(out_path), "--ablation"],
        capture_output=True,
        timeout=5.0,
    )
    assert res3.returncode == 1, f"Expected 1, got {res3.returncode}"


def test_fuzz_generator_cli(tmp_path: Path) -> None:
    """Verify tools/gen_fuzz_cases.py CLI generator writes all files."""
    cases_dir = tmp_path / "cases"
    written = generate_fuzz_cases(cases_dir)
    assert len(written) == len(ALL_FUZZ_CASES)

    for p in written:
        assert p.exists()
        if p.name != "empty_file.csv":
            assert p.stat().st_size > 0
