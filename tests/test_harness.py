"""Unit tests for tools.eval_degraded_harness."""

import tempfile
from pathlib import Path
import pytest

from tools.eval_degraded_harness import (
    REAL_SEQUENCES,
    DEFAULT_SEEDS,
    DEFAULT_TRAIN_SEEDS,
    DEFAULT_TEST_SEEDS,
    DegradedHarness,
    compute_3way_loss,
)


def test_real_sequences_exist():
    """Verify all 5 real sequence files exist on disk."""
    assert len(REAL_SEQUENCES) == 5
    for seq in REAL_SEQUENCES:
        imu_p = Path(seq["imu"])
        gt_p = Path(seq["gt"])
        assert imu_p.exists(), f"IMU file missing: {imu_p}"
        assert gt_p.exists(), f"GT file missing: {gt_p}"


def test_compute_3way_loss():
    """Test 3-way target loss calculation and normalization."""
    cand = {"ate_rmse_aligned": 2.0, "blind5_rmse": 4.0, "gap_2s_rmse": 1.0}
    naive = {"ate_rmse_aligned": 20.0, "blind5_rmse": 40.0, "gap_2s_rmse": 10.0}
    # Each ratio is 0.1
    # Loss = 0.50 * 0.1 + 0.33 * 0.1 + 0.17 * 0.1 = 0.10
    loss = compute_3way_loss(cand, naive)
    assert loss == pytest.approx(0.10, rel=1e-3)

    # Identical candidate and naive should yield exactly 1.0
    loss_self = compute_3way_loss(naive, naive)
    assert loss_self == pytest.approx(1.0, rel=1e-3)


def test_harness_smoke(tmp_path):
    """Run smoke test on 1 sequence and 1 seed using DegradedHarness."""
    harness = DegradedHarness(
        binary_path="./main",
        config_path="configs/degrade_default.json",
        append_blind=True,
        verbose=False,
    )

    test_seq = REAL_SEQUENCES[1]  # V1_01_easy
    rec = harness.evaluate_case(test_seq, seed=42, work_dir=tmp_path)

    assert rec["sequence"] == "V1_01_easy"
    assert rec["seed"] == 42
    assert "ate_rmse_aligned" in rec
    assert "blind5_rmse" in rec
    assert "gap_2s_rmse" in rec
    assert "J_standard" in rec
    assert "J_3way" in rec

    assert rec["ate_rmse_aligned"] > 0.0
    assert rec["blind5_rmse"] > 0.0
    assert rec["gap_2s_rmse"] > 0.0
    assert rec["J_standard"] > 0.0
    assert rec["J_standard"] < 1.0  # Must beat naive baseline


def test_loocv_partitioning():
    """Verify LOOCV partitioning leaves out 1 sequence per fold and splits seeds."""
    assert len(REAL_SEQUENCES) == 5
    assert len(set(DEFAULT_TRAIN_SEEDS).intersection(set(DEFAULT_TEST_SEEDS))) == 0

    for fold_idx in range(5):
        test_seq = REAL_SEQUENCES[fold_idx]
        train_seqs = [s for j, s in enumerate(REAL_SEQUENCES) if j != fold_idx]

        assert len(train_seqs) == 4
        assert test_seq["name"] not in [s["name"] for s in train_seqs]
