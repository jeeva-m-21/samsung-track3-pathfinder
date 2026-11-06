"""Unit tests for tolerant streaming IMU CSV parser (SPEC §12.8).

Tests:
1. Standard valid EuRoC CSV lines
2. Comment lines (#...) and blank lines
3. Missing or unparseable timestamps (skipped row, counted)
4. Malformed floats and NaN values (converted to NaN, flagged invalid)
5. Truncated lines (missing columns filled with NaN, flagged invalid)
6. Out of range checks for gyro (|omega_i| > invalid_gyro_max) and acc (|acc_i| > invalid_acc_max)
7. All-zeros check (all 6 values exactly zero flagged invalid)
8. Streaming iterator/generator interface: parse_imu_stream
9. File, StringIO, list, and generator inputs
10. Context manager and stats recording
"""

import io
import math
from pathlib import Path
import numpy as np
import pytest

from src.ref.parser import (
    ParserFlags,
    FLAG_OK,
    FLAG_MISSING_TIMESTAMP,
    FLAG_NAN_OR_INF,
    FLAG_TRUNCATED,
    FLAG_OUT_OF_RANGE_GYRO,
    FLAG_OUT_OF_RANGE_ACC,
    FLAG_ALL_ZEROS,
    FLAG_COMMENT_OR_BLANK,
    is_comment_or_blank,
    parse_imu_line,
    parse_imu_stream,
    load_params,
)


def test_valid_euroc_line():
    """Test standard EuRoC CSV line parsing."""
    line = "1403636579758555392,0.01,-0.02,0.03,0.1,0.2,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line)

    assert t_ns == 1403636579758555392
    assert isinstance(t_ns, int)
    np.testing.assert_allclose(omega, [0.01, -0.02, 0.03], atol=1e-12)
    np.testing.assert_allclose(acc, [0.1, 0.2, 9.81], atol=1e-12)
    assert is_valid is True
    assert flags == FLAG_OK
    assert flags == 0


def test_line_whitespace_and_crlf_tolerance():
    """Test tolerance to whitespace around commas and CRLF line endings."""
    line = "  1403636579758555392 ,  0.01 , -0.02 , 0.03 , 0.1 , 0.2 , 9.81 \r\n"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line)

    assert t_ns == 1403636579758555392
    assert is_valid is True
    assert flags == FLAG_OK
    np.testing.assert_allclose(omega, [0.01, -0.02, 0.03], atol=1e-12)
    np.testing.assert_allclose(acc, [0.1, 0.2, 9.81], atol=1e-12)


def test_comment_and_blank_lines():
    """Test that comment lines (#...) and blank lines are identified."""
    assert is_comment_or_blank("#timestamp [ns],w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z") is True
    assert is_comment_or_blank("   # Comment with leading whitespace") is True
    assert is_comment_or_blank("") is True
    assert is_comment_or_blank("   \t  \r\n") is True
    assert is_comment_or_blank("1403636579758555392,0,0,0,0,0,9.81") is False

    # Direct line parse on comment returns t_ns = None and COMMENT_OR_BLANK flag
    t_ns, omega, acc, is_valid, flags = parse_imu_line("# header comment")
    assert t_ns is None
    assert is_valid is False
    assert (flags & FLAG_COMMENT_OR_BLANK) != 0


def test_missing_or_unparseable_timestamp():
    """Test that missing or malformed timestamps return t_ns = None and are flagged."""
    # Corrupt string timestamp
    line_corrupt = "not_a_time,0.01,-0.02,0.03,0.1,0.2,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_corrupt)
    assert t_ns is None
    assert is_valid is False
    assert (flags & FLAG_MISSING_TIMESTAMP) != 0

    # Empty timestamp
    line_empty_t = ",0.01,-0.02,0.03,0.1,0.2,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_empty_t)
    assert t_ns is None
    assert is_valid is False
    assert (flags & FLAG_MISSING_TIMESTAMP) != 0

    # NaN timestamp
    line_nan_t = "NaN,0.01,-0.02,0.03,0.1,0.2,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_nan_t)
    assert t_ns is None
    assert is_valid is False
    assert (flags & FLAG_MISSING_TIMESTAMP) != 0


def test_malformed_floats_and_nan():
    """Test that malformed floats or NaN values become NaN and are flagged invalid."""
    # NaN in gyro
    line_nan_gyro = "1403636579758555392,0.01,NaN,0.03,0.1,0.2,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_nan_gyro)
    assert t_ns == 1403636579758555392
    assert is_valid is False
    assert (flags & FLAG_NAN_OR_INF) != 0
    assert omega[0] == pytest.approx(0.01)
    assert np.isnan(omega[1])
    assert omega[2] == pytest.approx(0.03)
    np.testing.assert_allclose(acc, [0.1, 0.2, 9.81])

    # Text corrupt in acc
    line_corrupt_acc = "1403636579758555392,0.01,-0.02,0.03,0.1,corrupt_val,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_corrupt_acc)
    assert t_ns == 1403636579758555392
    assert is_valid is False
    assert (flags & FLAG_NAN_OR_INF) != 0
    np.testing.assert_allclose(omega, [0.01, -0.02, 0.03])
    assert acc[0] == pytest.approx(0.1)
    assert np.isnan(acc[1])
    assert acc[2] == pytest.approx(9.81)

    # Inf in measurement
    line_inf = "1403636579758555392,0.01,-0.02,0.03,0.1,inf,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_inf)
    assert is_valid is False
    assert (flags & FLAG_NAN_OR_INF) != 0
    assert np.isnan(acc[1])


def test_truncated_lines():
    """Test that truncated lines fill missing fields with NaN and flag invalid."""
    # Only timestamp and 2 gyro fields
    line_trunc = "1403636579758555392,0.01,-0.02"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_trunc)

    assert t_ns == 1403636579758555392
    assert is_valid is False
    assert (flags & FLAG_TRUNCATED) != 0
    assert (flags & FLAG_NAN_OR_INF) != 0
    assert omega[0] == pytest.approx(0.01)
    assert omega[1] == pytest.approx(-0.02)
    assert np.isnan(omega[2])
    assert np.all(np.isnan(acc))

    # Only timestamp
    line_only_t = "1403636579758555392"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_only_t)
    assert t_ns == 1403636579758555392
    assert is_valid is False
    assert (flags & FLAG_TRUNCATED) != 0
    assert np.all(np.isnan(omega))
    assert np.all(np.isnan(acc))


def test_out_of_range_checks():
    """Test range bounds: |omega_i| > invalid_gyro_max (35.0) or |acc_i| > invalid_acc_max (160.0)."""
    # Gyro out of range
    line_gyro_high = "1403636579758555392,35.5,0.0,0.0,0.0,0.0,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_gyro_high)
    assert is_valid is False
    assert (flags & FLAG_OUT_OF_RANGE_GYRO) != 0

    # Negative gyro out of range
    line_gyro_neg = "1403636579758555392,0.0,-36.0,0.0,0.0,0.0,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_gyro_neg)
    assert is_valid is False
    assert (flags & FLAG_OUT_OF_RANGE_GYRO) != 0

    # Acc out of range
    line_acc_high = "1403636579758555392,0.0,0.0,0.0,0.0,0.0,165.0"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_acc_high)
    assert is_valid is False
    assert (flags & FLAG_OUT_OF_RANGE_ACC) != 0

    # Negative acc out of range
    line_acc_neg = "1403636579758555392,0.0,0.0,0.0,-161.0,0.0,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_acc_neg)
    assert is_valid is False
    assert (flags & FLAG_OUT_OF_RANGE_ACC) != 0

    # Custom thresholds via params
    custom_params = {"invalid_gyro_max": 10.0, "invalid_acc_max": 50.0}
    line_custom = "1403636579758555392,12.0,0.0,0.0,0.0,0.0,9.81"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_custom, params=custom_params)
    assert is_valid is False
    assert (flags & FLAG_OUT_OF_RANGE_GYRO) != 0


def test_all_zeros_check():
    """Test that all six values exactly zero is flagged invalid (SPEC §12.7)."""
    line_zeros = "1403636579758555392,0.0,0.0,0.0,0.0,0.0,0.0"
    t_ns, omega, acc, is_valid, flags = parse_imu_line(line_zeros)
    assert is_valid is False
    assert (flags & FLAG_ALL_ZEROS) != 0


def test_streaming_iterator_mixed_stream():
    """Test streaming iterator with mixed comments, blanks, missing timestamps, and invalid rows."""
    csv_data = [
        "# EuRoC IMU Data Stream\n",
        "# timestamp,w_x,w_y,w_z,a_x,a_y,a_z\n",
        "\n",
        "1000000000, 0.01, 0.02, 0.03, 0.1, 0.2, 9.81\n",     # Valid 1
        "bad_timestamp, 0.01, 0.02, 0.03, 0.1, 0.2, 9.81\n",   # Missing timestamp -> skipped & counted
        "1005000000, 0.02, 0.01, 0.03, 0.1, 0.2, 9.80\n",     # Valid 2
        "1010000000, NaN, 0.02, 0.03, 0.1, 0.2, 9.80\n",      # Invalid: NaN -> yielded
        "   \n",
        "# Mid-file comment\n",
        "1015000000, 40.0, 0.02, 0.03, 0.1, 0.2, 9.80\n",     # Invalid: gyro > 35 -> yielded
        "1020000000, 0.01, 0.02\n",                            # Invalid: truncated -> yielded
        "1025000000, 0.01, 0.02, 0.03, 0.1, 0.2, 9.82\n",     # Valid 3
    ]

    stats = {}
    stream = parse_imu_stream(csv_data, stats=stats)

    rows = list(stream)

    # 6 rows yielded total (the 1 missing timestamp row was skipped)
    assert len(rows) == 6
    assert stream.skipped_timestamp_count == 1
    assert stream.skipped_count == 1
    assert stats["skipped_timestamps"] == 1

    # Check yielded rows
    timestamps = [r[0] for r in rows]
    validities = [r[3] for r in rows]

    assert timestamps == [1000000000, 1005000000, 1010000000, 1015000000, 1020000000, 1025000000]
    assert validities == [True, True, False, False, False, True]

    assert stream.valid_count == 3
    assert stream.invalid_count == 3
    assert stats["valid_samples"] == 3
    assert stats["invalid_samples"] == 3


def test_streaming_from_file_and_context_manager(tmp_path):
    """Test parse_imu_stream reading from a file path using context manager."""
    csv_file = tmp_path / "imu_test.csv"
    content = (
        "# timestamp,w_x,w_y,w_z,a_x,a_y,a_z\n"
        "1000000000,0.0,0.0,0.0,0.0,0.0,9.81\n"
        "corrupt_t,0.0,0.0,0.0,0.0,0.0,9.81\n"
        "1005000000,0.1,0.2,0.3,0.4,0.5,9.80\n"
    )
    csv_file.write_text(content, encoding="utf-8")

    with parse_imu_stream(csv_file) as stream:
        items = [item for item in stream]

    assert len(items) == 2
    assert stream.skipped_count == 1
    assert items[0][0] == 1000000000
    assert items[1][0] == 1005000000


def test_streaming_from_stringio():
    """Test streaming from an io.StringIO buffer."""
    buf = io.StringIO(
        "1000000000, 0.01, -0.01, 0.0, 0.0, 0.0, 9.81\n"
        "1005000000, 0.01, -0.01, 0.0, 0.0, 0.0, 9.81\n"
    )
    stream = parse_imu_stream(buf)
    rows = list(stream)
    assert len(rows) == 2
    assert rows[0][0] == 1000000000
    assert rows[1][0] == 1005000000
    assert rows[0][3] is True


def test_pure_function_reproducibility():
    """Test that parse_imu_line is a pure function without side effects."""
    line = "1403636579758555392,0.01,-0.02,0.03,0.1,0.2,9.81"
    res1 = parse_imu_line(line)
    res2 = parse_imu_line(line)

    assert res1[0] == res2[0]
    np.testing.assert_array_equal(res1[1], res2[1])
    np.testing.assert_array_equal(res1[2], res2[2])
    assert res1[3] == res2[3]
    assert res1[4] == res2[4]


def test_load_params():
    """Test parameter loading from configs/params.json."""
    params = load_params()
    assert "invalid_gyro_max" in params
    assert "invalid_acc_max" in params
    assert params["invalid_gyro_max"] == pytest.approx(35.0)
    assert params["invalid_acc_max"] == pytest.approx(160.0)
