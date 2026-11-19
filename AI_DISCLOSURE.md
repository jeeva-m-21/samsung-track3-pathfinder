# Official AI Disclosure Statement

**Samsung Hackathon — Track 3: Path Finder (IMU-Only Dead-Reckoning)**  
**Team:** `VITC_SingleThread`  
**Date:** November 2026

---

## 1. Executive Summary

In adherence to the Samsung Hackathon competition guidelines and ethics policy, this disclosure provides a transparent declaration regarding the use of Artificial Intelligence (AI) tools during the design, development, optimization, and documentation of the **VITC_SingleThread** navigation system.

---

## 2. Model Architecture & Core Algorithm: Deterministic Physics (0% Neural Networks)

The navigation engine submitted in `VITC_SingleThread` is **100% deterministic, classical physics-based estimation software**.
- **Core Algorithm:** 15-State Error-State Kalman Filter (ESKF) formulated on Lie group manifolds ($\text{SO}(3) \times \mathbb{R}^{12}$).
- **Kinematics:** Midpoint strapdown integration of 6-axis raw IMU telemetry (accelerometer and gyroscope).
- **Measurement Updates:** Linear and non-linear Kalman measurement updates (Zero Velocity Updates - ZUPT, Non-Holonomic Constraints - NHC, Centripetal-Compensated Gravity Tilt Alignment, and Student-$t$ noise-gated velocity bounds).
- **Neural Networks / Black-Box Models:** **ZERO**. There are no neural networks, transformers, reinforcement learning policies, or learned black-box weights used in the estimation loop. All state transitions and covariance updates follow first-principles classical mechanics.

---

## 3. Scope of AI Assistance

Generative AI coding assistants (Google Antigravity / Gemini) were utilized strictly as software engineering tools in the following defined scopes:

| Activity | AI Utilized? | Description & Human Oversight |
| :--- | :---: | :--- |
| **Mathematical Derivation** |  No | Filter kinematics, error-state Jacobians, covariance discretization, and centripetal compensation were derived from first-principles inertial navigation theory (Farrell, Titterton, Sola). |
| **Code Refactoring & Porting** |  Yes | Assisted in optimizing inner loop matrix operations into scalar operations to minimize Python runtime overhead. |
| **Test Suite Generation** |  Yes | Assisted in generating adversarial fuzzing test cases and parameter permutations. |
| **Harness Scripting & Benchmarks** |  Yes | Assisted in authoring multi-seed batch execution scripts (`tools/eval_multiseed_matrix.py`). |
| **Documentation & Markdown Layout** |  Yes | Assisted in formatting tables, checklists, and README presentation. |

---

## 4. Integrity and Compliance Verification

1. **Human Authorship & Rigorous Verification**: All algorithmic formulas, state dimensions, gating thresholds, and noise schedules were thoroughly inspected, verified against unit tests, and cross-validated across 50 multi-seed runs.
2. **Reproducibility**: The complete source code is self-contained in a single file (`src/main.py`) requiring only the standard Python library, ensuring 100% reproducible execution on any Linux machine without internet access or proprietary APIs.
3. **Intellectual Property**: The code contains no proprietary third-party intellectual property or copied code from commercial navigation suites.
