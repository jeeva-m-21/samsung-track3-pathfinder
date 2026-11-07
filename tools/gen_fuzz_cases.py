#!/usr/bin/env python3
"""Adversarial Fuzz Inputs Generator for IMU data.

Generates extreme, malformed, corrupted, and adversarial IMU CSV inputs
to rigorously test the robustness of the C++ estimator pipeline.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class FuzzCase:
    name: str
    description: str
    content: bytes
    expected_exit_codes: Tuple[int, ...] = (0, 1)
    expect_validator_pass: bool = False
    expected_rows: Optional[int] = None


def _format_imu_row(
    t_ns: int,
    wx: float = 0.0,
    wy: float = 0.0,
    wz: float = 0.0,
    ax: float = 0.0,
    ay: float = 0.0,
    az: float = 9.80665,
) -> str:
    """Format a standard EuRoC IMU CSV line."""
    return f"{t_ns},{wx:.6f},{wy:.6f},{wz:.6f},{ax:.6f},{ay:.6f},{az:.6f}\n"


def get_all_fuzz_cases() -> List[FuzzCase]:
    """Return a list of all defined adversarial fuzz cases."""
    header = "#timestamp [ns],w_RS_x,w_RS_y,w_RS_z,a_RS_x,a_RS_y,a_RS_z\n"
    t_start = 1000000000000  # 1000 s in nanoseconds
    dt_nom_ns = 5000000  # 5 ms (200 Hz)

    cases: List[FuzzCase] = []

    # 1. 0-byte file (empty)
    cases.append(
        FuzzCase(
            name="empty_file",
            description="0-byte completely empty file",
            content=b"",
            expected_exit_codes=(0, 1),
            expect_validator_pass=False,
            expected_rows=0,
        )
    )

    # 2. Header-only file
    cases.append(
        FuzzCase(
            name="header_only",
            description="Header-only CSV file without any data rows",
            content=header.encode("utf-8"),
            expected_exit_codes=(0, 1),
            expect_validator_pass=False,
            expected_rows=0,
        )
    )

    # 3. Single valid IMU line
    single_line_content = header + _format_imu_row(t_start, 0.001, -0.002, 0.003, 0.05, -0.04, 9.80665)
    cases.append(
        FuzzCase(
            name="single_row",
            description="Single valid IMU row with standard header",
            content=single_line_content.encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=1,
        )
    )

    # 4. Backwards timestamps (t2 < t1)
    backwards_rows = [header]
    for i in range(250):
        t = t_start - i * dt_nom_ns
        backwards_rows.append(_format_imu_row(t, 0.001, -0.001, 0.0005, 0.02, -0.03, 9.80665))
    cases.append(
        FuzzCase(
            name="backwards_timestamps",
            description="Timestamps decreasing backwards in time",
            content="".join(backwards_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=False,  # validator enforces monotonically increasing timestamps
            expected_rows=250,
        )
    )

    # 5. Duplicate timestamps (t2 == t1)
    duplicate_rows = [header]
    for _ in range(250):
        duplicate_rows.append(_format_imu_row(t_start, 0.001, -0.001, 0.0005, 0.02, -0.03, 9.80665))
    cases.append(
        FuzzCase(
            name="duplicate_timestamps",
            description="All rows share the exact same duplicate timestamp",
            content="".join(duplicate_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=False,  # validator enforces strictly increasing timestamps
            expected_rows=250,
        )
    )

    # 6. Huge timestamp jump (100-second gap)
    gap_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        gap_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    gap_ns = 100 * 1000000000  # 100 seconds
    t_after_gap = t_start + 250 * dt_nom_ns + gap_ns
    for i in range(100):
        t = t_after_gap + i * dt_nom_ns
        gap_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="huge_timestamp_gap",
            description="Valid data interrupted by a 100-second timestamp gap",
            content="".join(gap_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=350,
        )
    )

    # 7. NaN bursts in gyro and accel channels
    nan_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        nan_rows.append(_format_imu_row(t, 0.001, 0.0, 0.0, 0.0, 0.0, 9.80665))
    for i in range(250, 300):
        t = t_start + i * dt_nom_ns
        nan_rows.append(f"{t},nan,NaN,NAN,nan,NaN,NAN\n")
    for i in range(300, 400):
        t = t_start + i * dt_nom_ns
        nan_rows.append(_format_imu_row(t, 0.001, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="nan_burst",
            description="50-row burst of NaN values across all 6 IMU channels",
            content="".join(nan_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=400,
        )
    )

    # 8. Inf bursts (+inf and -inf)
    inf_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        inf_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    for i in range(250, 300):
        t = t_start + i * dt_nom_ns
        inf_rows.append(f"{t},inf,-inf,+inf,inf,-inf,+inf\n")
    for i in range(300, 400):
        t = t_start + i * dt_nom_ns
        inf_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="inf_burst",
            description="50-row burst of positive and negative infinities",
            content="".join(inf_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=400,
        )
    )

    # 9. 1000g acceleration spike (norm > 9800 m/s^2)
    spike_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        spike_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    for i in range(250, 260):
        t = t_start + i * dt_nom_ns
        spike_rows.append(f"{t},0.0,0.0,0.0,5000.0,5000.0,10000.0\n")
    for i in range(260, 320):
        t = t_start + i * dt_nom_ns
        spike_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="accel_1000g_spike",
            description="Extreme 1000g acceleration spike (>9800 m/s^2)",
            content="".join(spike_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=320,
        )
    )

    # 10. 1000 rad/s angular spin
    spin_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        spin_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    for i in range(250, 260):
        t = t_start + i * dt_nom_ns
        spin_rows.append(f"{t},1000.0,1000.0,1000.0,0.0,0.0,9.80665\n")
    for i in range(260, 320):
        t = t_start + i * dt_nom_ns
        spin_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="gyro_1000rad_spin",
            description="Extreme 1000 rad/s angular velocity spin",
            content="".join(spin_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=320,
        )
    )

    # 11. Stuck sensor (identical values for 500 rows)
    stuck_rows = [header]
    for i in range(500):
        t = t_start + i * dt_nom_ns
        stuck_rows.append(f"{t},0.1234,0.5678,0.9101,1.2340,2.3450,9.8100\n")
    cases.append(
        FuzzCase(
            name="stuck_sensor",
            description="Stuck sensor emitting identical float values for 500 rows",
            content="".join(stuck_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=500,
        )
    )

    # 12. All-zero measurements (0,0,0, 0,0,0)
    zero_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        zero_rows.append(f"{t},0.0,0.0,0.0,0.0,0.0,0.0\n")
    cases.append(
        FuzzCase(
            name="all_zeros",
            description="All sensor channels reading exactly 0.0",
            content="".join(zero_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=250,
        )
    )

    # 13. Random binary noise
    # Deterministic pseudo-random bytes with NULLs and unprintable characters
    import random
    rng = random.Random(42)
    binary_noise = bytes(rng.getrandbits(8) for _ in range(2048))
    cases.append(
        FuzzCase(
            name="random_binary_noise",
            description="Random unprintable binary noise and garbage bytes",
            content=binary_noise,
            expected_exit_codes=(0, 1),
            expect_validator_pass=False,
        )
    )

    # 14. Malformed column counts
    malformed_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        malformed_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    malformed_rows.append(f"{t_start + 250 * dt_nom_ns},0.0,0.0\n")  # 3 cols
    malformed_rows.append(f"{t_start + 251 * dt_nom_ns},0.0,0.0,0.0,0.0,0.0,9.81,extra,cols,10\n")  # 10 cols
    malformed_rows.append(f"{t_start + 252 * dt_nom_ns},,,,,,,\n")  # empty commas
    malformed_rows.append(f"{t_start + 253 * dt_nom_ns},abc,def,ghi,jkl,mno,pqr\n")  # non-numeric strings
    for i in range(254, 300):
        t = t_start + i * dt_nom_ns
        malformed_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="malformed_columns",
            description="Malformed column counts, empty commas, and non-numeric fields",
            content="".join(malformed_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=300,
        )
    )

    # 15. Sub-microsecond dt (dt = 1 ns)
    tiny_dt_rows = [header]
    for i in range(250):
        t = t_start + i  # 1 ns increments
        tiny_dt_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="sub_microsecond_dt",
            description="Very high rate sub-microsecond dt (1 nanosecond step)",
            content="".join(tiny_dt_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=250,
        )
    )

    # 16. Timestamps near int64 max (near 2^63 - 1)
    near_max_t = 9223372035000000000
    near_max_rows = [header]
    for i in range(250):
        t = near_max_t + i * dt_nom_ns
        near_max_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="timestamps_near_int64_max",
            description="Timestamps near signed 64-bit maximum integer (2^63 - 1)",
            content="".join(near_max_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=250,
        )
    )

    # 17. Timestamp overflow (> 2^63 - 1)
    overflow_rows = [header]
    for i in range(100):
        t = 9999999999999999999 + i * 1000  # exceeds LLONG_MAX
        overflow_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="timestamp_overflow",
            description="Timestamps exceeding signed 64-bit int max (strtoll overflow)",
            content="".join(overflow_rows).encode("utf-8"),
            expected_exit_codes=(0, 1),
            expect_validator_pass=False,
        )
    )

    # 17. Huge sensor values (1e30)
    huge_val_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        huge_val_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    for i in range(250, 260):
        t = t_start + i * dt_nom_ns
        huge_val_rows.append(f"{t},1e30,1e30,1e30,1e30,1e30,1e30\n")
    for i in range(260, 300):
        t = t_start + i * dt_nom_ns
        huge_val_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="huge_values_1e30",
            description="Sensor values with extreme magnitudes (1e30)",
            content="".join(huge_val_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=300,
        )
    )

    # 18. CRLF line endings
    crlf_rows = ["#timestamp [ns],w_RS_x,w_RS_y,w_RS_z,a_RS_x,a_RS_y,a_RS_z\r\n"]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        crlf_rows.append(f"{t},0.000000,0.000000,0.000000,0.000000,0.000000,9.806650\r\n")
    cases.append(
        FuzzCase(
            name="crlf_endings",
            description="Windows CRLF (\\r\\n) line endings throughout file",
            content="".join(crlf_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=250,
        )
    )

    # 19. No header line
    no_header_rows = []
    for i in range(250):
        t = t_start + i * dt_nom_ns
        no_header_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="no_header",
            description="Valid data rows without any leading # comment header line",
            content="".join(no_header_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=250,
        )
    )

    # 20. Truncated last line
    trunc_content = header
    for i in range(250):
        t = t_start + i * dt_nom_ns
        trunc_content += _format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665)
    trunc_content += f"{t_start + 250 * dt_nom_ns},0.0,0.0,0."  # ends abruptly mid-float
    cases.append(
        FuzzCase(
            name="truncated_last_line",
            description="Final line truncated mid-number without trailing newline",
            content=trunc_content.encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=251,
        )
    )

    # 21. Gap continuing to end of file
    gap_end_rows = [header]
    for i in range(250):
        t = t_start + i * dt_nom_ns
        gap_end_rows.append(_format_imu_row(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    gap_end_rows.append(_format_imu_row(t_start + 250 * dt_nom_ns + 20 * 1000000000, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665))
    cases.append(
        FuzzCase(
            name="gap_to_end_of_file",
            description="20-second gap jumping directly to the final sample of the file",
            content="".join(gap_end_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=251,
        )
    )

    # 22. Large stream (10,000 samples, ~50 seconds)
    large_stream_rows = [header]
    for i in range(10000):
        t = t_start + i * dt_nom_ns
        large_stream_rows.append(_format_imu_row(t, 0.001, -0.001, 0.0005, 0.02, -0.03, 9.80665))
    cases.append(
        FuzzCase(
            name="large_stream",
            description="10,000 sample stream (~50s of data) checking speed & memory",
            content="".join(large_stream_rows).encode("utf-8"),
            expected_exit_codes=(0,),
            expect_validator_pass=True,
            expected_rows=10000,
        )
    )

    return cases


def generate_fuzz_cases(output_dir: Path) -> List[Path]:
    """Write all fuzz cases to output_dir and return list of written paths."""
    output_dir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []
    for case in get_all_fuzz_cases():
        file_path = output_dir / f"{case.name}.csv"
        file_path.write_bytes(case.content)
        written.append(file_path)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate adversarial IMU fuzz test cases.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory to write generated fuzz cases (.csv files)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all defined fuzz cases and exit",
    )
    args = parser.parse_args()

    cases = get_all_fuzz_cases()

    if args.list:
        print(f"Total fuzz cases: {len(cases)}")
        for idx, case in enumerate(cases, 1):
            print(f"{idx:2d}. {case.name:<25} - {case.description}")
        return

    if args.output_dir:
        written = generate_fuzz_cases(args.output_dir)
        print(f"Generated {len(written)} fuzz test cases in {args.output_dir}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
