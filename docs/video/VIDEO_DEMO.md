# Video Demonstration & Walkthrough Guide

**Samsung Hackathon — Track 3: Path Finder (IMU-Only Dead-Reckoning)**  
**Team:** `VITC_SingleThread`  
**Presenter:** Jeeva M (23MIS1059), Vellore Institute of Technology, Chennai

---

## 1. Video Overview & Direct Links

- **Title:** VITC_SingleThread: High-Accuracy Inertial Dead-Reckoning Under Severe Sensor Degradation
- **Duration:** 3 Minutes 10 Seconds (190 Seconds)
- **YouTube Link:** [https://youtu.be/rLN_ayVEV6g](https://youtu.be/rLN_ayVEV6g)
- **Local Master Video File:** [`docs/video/demo_walkthrough.mp4`](demo_walkthrough.mp4) (H.264/AAC, 8.1 MB, web-optimized faststart)
- **Slide Deck:** [Official 9-Slide Presentation (Google Slides)](https://docs.google.com/presentation/d/1RJSXaoCRnsPSUkJOGbNYNX21EPCdQZjaV4vCF-gvLiY/edit?usp=sharing)

<div align="center">

[![Path Finder Video Demonstration](https://img.youtube.com/vi/rLN_ayVEV6g/maxresdefault.jpg)](https://youtu.be/rLN_ayVEV6g "Click to Watch Video Demonstration on YouTube")

*Click the thumbnail above to stream on YouTube, or open the local web-optimized MP4 directly: [demo_walkthrough.mp4](demo_walkthrough.mp4)*

</div>

---

## 2. Video Storyboard & Technical Script Breakdown

### [0:00 - 0:35] Title & The Core Problem
- **Slide Reference:** Slides 1 & 2
- **Visual:** Hackathon Track 03 Title, Key KPIs, and the physical breakdown of pure IMU dead-reckoning.
- **Key Concepts:** Why double integration ($\iint (a - b_a) dt^2$) diverges quadratically, uncalibrated bias drift ($> 225\text{ m}$ drift in 100s), and the 5 synthetic degradation modes (noise ramps, bias walk, dropouts, 2s blackout gaps, and terminal 5s blind cutoff).

### [0:35 - 1:25] Technological Solution & Filter Architecture
- **Slide Reference:** Slides 3 & 4
- **Visual:** End-to-end architecture pipeline diagram ([`docs/plots/architecture.png`](../plots/architecture.png)).
- **Key Concepts:**
  1. *Signal Pre-Cleaning*: MAD-based second-difference noise multiplier estimation ($\hat{\sigma}$).
  2. *Student-t Robust Fusion*: Down-weighting shock impulses smoothly via residual covariance scaling.
  3. *Regime-Adaptive Kinematic Priors*: Centripetal-compensated pedestrian tilt alignment, Weinberg stride bounding, and drone soft $v_z$ prior.
  4. *Decoupled Blackout Extrapolation*: Curvature-preserving Simpson decay bridging the 2s gap and final 5s blind cutoff.

### [1:25 - 2:10] Live Terminal Execution & Clean-Room Verification
- **Slide Reference:** Slide 7
- **Visual:** High-speed single-threaded execution on degraded telemetry.
- **Commands Executed:**
  ```bash
  # 1. Zero external pip packages
  python3 -m pip list || echo "Zero external packages required"

  # 2. Pinned single-core execution with resource profiling
  /usr/bin/time -v taskset -c 0 ./VITC_SingleThread/main evaluator_input.zip estimated_trajectory.txt

  # 3. Output compliance verification
  wc -l estimated_trajectory.txt
  head -n 5 estimated_trajectory.txt

  # 4. Official evo evaluation against ground truth
  evo_ape tum groundtruth.txt estimated_trajectory.txt -a -v
  ```
- **Showcase:** Processing 27,526 samples finishes in **`1.61 s`** wall time (**`11.59 ms/s`**), consuming only **`23.9 MB`** peak RSS and **`2.36 MB`** heap. Exactly 27,526 output poses ($N_{\text{out}} \equiv N_{\text{in}}$).

### [2:10 - 2:50] 50-Run Benchmark Results & Physical Differentiation
- **Slide Reference:** Slides 5 & 6
- **Visual:** Benchmark Scorecard (Slide 5) and V2_02 Agile Flight progression (Slide 6).
- **Key Metrics Highlighted:**
  - Overall ATE: **`2.102 m`** (−63.7% vs baseline 5.79 m).
  - Terminal 5s Blind Cutoff: **`2.894 m`** (−84.0% vs baseline 18.06 m).
  - 2s Gap Blackout: **`1.770 m`** (−73.3% vs baseline 6.63 m).
  - Local 1-second RPE: **`0.743 m/s`** (beats stationary baseline 5/5).
  - Agile flight stabilization: `V2_02_medium` ATE from $10.59\text{ m} \to 1.83\text{ m}$ (−83%), preserving 84.9% physical path scale.

### [2:50 - 3:10] Ablation Gates, Engineering Rigor & Conclusion
- **Slide Reference:** Slides 8 & 9
- **Visual:** Stepwise ablation progression (Slide 8) and Contact / Submission summary (Slide 9).
- **Key Takeaways:** 272/272 passing tests, disciplined rejection of non-generalizing hypotheses (body rotor drag, RTS smoother), production-grade zero-dependency stdlib code.
