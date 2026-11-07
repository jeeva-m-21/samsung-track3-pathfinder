#include "src/cpp/updates.h"
#include "src/cpp/monitor.h"
#include "src/cpp/gap.h"
#include "src/cpp/blind.h"
#include "src/cpp/eskf.h"
#include "src/cpp/init.h"
#include "src/cpp/mat.h"
#include "src/cpp/quat.h"
#include "src/cpp/params.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace math;

static int g_tests_run = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (!(cond)) { \
        std::printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
        g_tests_failed++; \
    } \
} while(0)

#define TEST_ASSERT_NEAR(val, exp, tol, msg) do { \
    g_tests_run++; \
    double diff = std::abs((val) - (exp)); \
    if (diff > (tol)) { \
        std::printf("FAIL [%s:%d]: %s (got %.15e, expected %.15e, diff %.15e > tol %.15e)\n", \
                    __FILE__, __LINE__, msg, (double)(val), (double)(exp), diff, (double)(tol)); \
        g_tests_failed++; \
    } \
} while(0)

// ============================================================================
// 1. Tilt update and ZUPT tests
// ============================================================================

static void test_tilt_update() {
    // 1. Verify tilt update reduces pitch/roll error
    NominalState state;
    // Introduce 5 degree roll error: phi = [0.087266, 0, 0]
    double roll_angle = 5.0 * M_PI / 180.0;
    state.q = exp_map(make_vec3(roll_angle, 0.0, 0.0));
    state.v = make_vec3(0.0, 0.0, 0.0);
    state.p = make_vec3(0.0, 0.0, 0.0);
    state.b_g = make_vec3(0.0, 0.0, 0.0);
    state.b_a = make_vec3(0.0, 0.0, 0.0);

    Mat15 P = Mat15::identity() * 0.01;

    // In true upright orientation, body accelerometer measures specific force [0, 0, g]
    Vec3 acc = make_vec3(0.0, 0.0, 9.80665);
    Vec3 omega = make_vec3(0.0, 0.0, 0.0);
    Vec3 g_w = make_vec3(0.0, 0.0, -9.80665);

    double init_roll_err = std::abs(log_map(state.q).data[0]);
    TEST_ASSERT(init_roll_err > 0.08, "Initial roll error should be ~0.087 rad");

    double p_theta_init = P(0, 0);

    bool applied = apply_tilt_update(state, P, acc, omega, 1.0, g_w);
    TEST_ASSERT(applied, "Tilt update should pass gating and be applied");

    double post_roll_err = std::abs(log_map(state.q).data[0]);
    TEST_ASSERT(post_roll_err < init_roll_err * 0.5, "Tilt update should reduce roll error by > 50%");
    TEST_ASSERT(P(0, 0) < p_theta_init, "Attitude covariance should decrease after tilt update");

    // 2. Gating checks
    // Large acceleration (e.g. 15 m/s^2) should fail gating
    Vec3 bad_acc = make_vec3(0.0, 0.0, 15.0);
    bool rej_acc = apply_tilt_update(state, P, bad_acc, omega, 1.0, g_w);
    TEST_ASSERT(!rej_acc, "Tilt update should be rejected when acceleration deviates from g");

    // High angular velocity should fail gating (|omega| > 1.0 rad/s)
    Vec3 bad_omega = make_vec3(1.5, 0.0, 0.0);
    bool rej_gyro = apply_tilt_update(state, P, acc, bad_omega, 1.0, g_w);
    TEST_ASSERT(!rej_gyro, "Tilt update should be rejected when angular velocity exceeds omega_max");
}

static void test_zupt_update() {
    NominalState state;
    state.q = Quat::identity();
    state.v = make_vec3(0.2, -0.15, 0.1);
    state.p = make_vec3(1.0, 2.0, 3.0);
    state.b_g = make_vec3(0.0, 0.0, 0.0);
    state.b_a = make_vec3(0.0, 0.0, 0.0);

    Mat15 P = Mat15::identity() * 0.05;
    double p_v_init = P(3, 3);
    double v_norm_init = state.v.norm();

    bool applied = apply_zupt_update(state, P);
    TEST_ASSERT(applied, "ZUPT update should succeed");
    TEST_ASSERT(state.v.norm() < v_norm_init * 0.5, "ZUPT update should reduce velocity norm");
    TEST_ASSERT(P(3, 3) < p_v_init, "Velocity covariance should decrease after ZUPT update");
}

static void test_stationary_buffer() {
    StationaryBuffer buf;
    TEST_ASSERT(!buf.is_stationary(), "Empty buffer should not be stationary");

    // Feed 50 samples at 200 Hz (dt = 0.005s) covering 0.25s
    double t = 0.0;
    for (int i = 0; i < 50; ++i) {
        Vec3 omega = make_vec3(0.005, -0.005, 0.002);
        Vec3 acc = make_vec3(0.01, -0.01, 9.80665);
        buf.add_sample(t, omega, acc);
        t += 0.005;
    }

    TEST_ASSERT(buf.is_stationary(9.80665, 1.0), "Low noise static window should be stationary");
    Vec3 mean_w = buf.get_mean_gyro();
    TEST_ASSERT_NEAR(mean_w.data[0], 0.005, 1e-4, "Stationary mean gyro x should match");
    TEST_ASSERT_NEAR(mean_w.data[1], -0.005, 1e-4, "Stationary mean gyro y should match");

    // Low quality should reject stationary condition
    TEST_ASSERT(!buf.is_stationary(9.80665, 0.2), "Quality below 0.3 should reject stationary detection");

    // Dynamic buffer with motion
    StationaryBuffer dynamic_buf;
    t = 0.0;
    for (int i = 0; i < 50; ++i) {
        Vec3 omega = make_vec3(0.5, 0.0, 0.0); // 0.5 rad/s > 0.03 rad/s
        Vec3 acc = make_vec3(0.0, 0.0, 9.80665);
        dynamic_buf.add_sample(t, omega, acc);
        t += 0.005;
    }
    TEST_ASSERT(!dynamic_buf.is_stationary(9.80665, 1.0), "High gyro buffer should not be stationary");
}

// ============================================================================
// 2. Degradation Monitor tests
// ============================================================================

static void test_degradation_monitor() {
    DegradationMonitor mon;
    TEST_ASSERT(!mon.calibrated(), "Monitor initially uncalibrated");

    double t = 0.0;
    double dt = 0.005; // 200 Hz

    // 1. Initial 1.0 s calibration phase: low noise (std ~ 0.001)
    for (int i = 0; i < 201; ++i) {
        double noise_a = ((i % 2 == 0) ? 0.002 : -0.002);
        double noise_w = ((i % 3 == 0) ? 0.0005 : -0.0005);
        Vec3 acc = make_vec3(noise_a, 0.0, 9.80665 + noise_a);
        Vec3 omega = make_vec3(noise_w, 0.0, noise_w);
        mon.step(t, omega, acc);
        t += dt;
    }

    TEST_ASSERT(mon.calibrated(), "Monitor should be calibrated after 1.0 s");
    TEST_ASSERT_NEAR(mon.quality(), 1.0, 0.05, "Quality should be near 1.0 during nominal conditions");
    TEST_ASSERT_NEAR(mon.kappa(), 1.0, 0.05, "Kappa should be near 1.0 during nominal conditions");

    // 2. Introduce 15x noise degradation
    for (int i = 0; i < 300; ++i) {
        double noise_a = ((i % 2 == 0) ? 0.05 : -0.05); // 25x larger residuals
        double noise_w = ((i % 3 == 0) ? 0.02 : -0.02);
        Vec3 acc = make_vec3(noise_a, 0.0, 9.80665 + noise_a);
        Vec3 omega = make_vec3(noise_w, 0.0, noise_w);
        mon.step(t, omega, acc);
        t += dt;
    }

    TEST_ASSERT(mon.quality() < 0.2, "Quality should drop significantly under high noise");
    TEST_ASSERT(mon.kappa() > 5.0, "Kappa inflation factor should increase under high noise");
    TEST_ASSERT(mon.noise_ratio() > 5.0, "Noise ratio should be > 5.0 under high noise");
}

// ============================================================================
// 3. Gap and Dropout Handler tests
// ============================================================================

static void test_gap_classifier() {
    Vec3 normal_w = make_vec3(0.01, 0.02, -0.01);
    Vec3 normal_a = make_vec3(0.1, -0.1, 9.8);
    TEST_ASSERT(classify_sample(normal_w, normal_a) == SampleValidity::VALID, "Normal sample is valid");

    // NaN / Inf
    Vec3 nan_w = make_vec3(NAN, 0.0, 0.0);
    TEST_ASSERT(classify_sample(nan_w, normal_a) == SampleValidity::NAN_OR_INF, "NaN gyro classified as NAN_OR_INF");
    Vec3 inf_a = make_vec3(0.0, INFINITY, 9.8);
    TEST_ASSERT(classify_sample(normal_w, inf_a) == SampleValidity::NAN_OR_INF, "Inf acc classified as NAN_OR_INF");

    // All zero
    Vec3 zero_v = make_vec3(0.0, 0.0, 0.0);
    TEST_ASSERT(classify_sample(zero_v, zero_v) == SampleValidity::ALL_ZERO, "All zeros classified as ALL_ZERO");

    // Out of range
    Vec3 huge_w = make_vec3(40.0, 0.0, 0.0); // > 35 rad/s
    TEST_ASSERT(classify_sample(huge_w, normal_a) == SampleValidity::OUT_OF_RANGE, "Huge gyro classified as OUT_OF_RANGE");
    Vec3 huge_a = make_vec3(0.0, 170.0, 0.0); // > 160 m/s^2
    TEST_ASSERT(classify_sample(normal_w, huge_a) == SampleValidity::OUT_OF_RANGE, "Huge acc classified as OUT_OF_RANGE");

    // Stuck sensor (3 consecutive identical rows)
    Vec3 prev_w = normal_w;
    Vec3 prev_a = normal_a;
    TEST_ASSERT(classify_sample(normal_w, normal_a, &prev_w, &prev_a, 2) == SampleValidity::STUCK,
                "Identical sample with repeat count 2 (3rd sample) classified as STUCK");
}

static void test_gap_substepping_decay() {
    NominalState state;
    state.q = Quat::identity();
    state.v = make_vec3(2.0, 0.0, 0.0); // 2 m/s along x
    state.p = make_vec3(0.0, 0.0, 0.0);
    state.b_g = make_vec3(0.0, 0.0, 0.0);
    state.b_a = make_vec3(0.0, 0.0, 0.0);

    Mat15 P = Mat15::identity() * 0.01;
    Vec3 omega_s = make_vec3(0.0, 0.0, 0.0);

    // Propagate over a 2.0 s gap
    double dt = 2.0;
    double dt_nom = 0.005; // 400 sub-steps
    int substeps = propagate_gap(state, P, omega_s, dt, dt_nom);

    TEST_ASSERT(substeps == 400, "Should execute 400 sub-steps for 2s gap at 200 Hz");

    // Velocity should decay: v = v0 * exp(-2.0 / 2.0) = 2.0 * exp(-1.0) ~ 0.73575888
    double exp_v = 2.0 * std::exp(-2.0 / params::tau_gap);
    TEST_ASSERT_NEAR(state.v.data[0], exp_v, 1e-3, "Velocity after 2s gap should match exponential decay");

    // Position integral: p = v0 * tau_gap * (1 - exp(-dt/tau_gap)) = 2 * 2 * (1 - exp(-1)) ~ 2.52848
    double exp_p = 2.0 * params::tau_gap * (1.0 - std::exp(-2.0 / params::tau_gap));
    TEST_ASSERT_NEAR(state.p.data[0], exp_p, 1e-2, "Position after 2s gap should not explode");

    // Covariance should inflate due to kappa_gap
    TEST_ASSERT(P(0, 0) > 0.01, "Attitude covariance should inflate across gap");
    TEST_ASSERT(P(3, 3) > 0.01, "Velocity covariance should inflate across gap");
}

// ============================================================================
// 4. Blind Extrapolation and Failure Detector tests
// ============================================================================

static void test_blind_extrapolation_math() {
    Vec3 p_f = make_vec3(1.0, 2.0, 3.0);
    Vec3 v_f = make_vec3(2.0, -1.0, 0.5);
    Quat q_f = Quat::identity();
    Vec3 omega_f = make_vec3(0.1, 0.0, -0.2);

    double tau = 1.5;
    // Expected position: p = p_f + v_f * shrink_v * tau_v * (1 - exp(-tau/tau_v))
    double factor = params::shrink_v * params::tau_v * (1.0 - std::exp(-tau / params::tau_v));
    Vec3 exp_p = p_f + v_f * factor;

    Vec3 pred_p = predict_blind_position(p_f, v_f, tau);
    TEST_ASSERT_NEAR(pred_p.data[0], exp_p.data[0], 1e-12, "Blind position x matches closed form");
    TEST_ASSERT_NEAR(pred_p.data[1], exp_p.data[1], 1e-12, "Blind position y matches closed form");
    TEST_ASSERT_NEAR(pred_p.data[2], exp_p.data[2], 1e-12, "Blind position z matches closed form");

    // Clamp beyond 5.0 s
    Vec3 p_at_5 = predict_blind_position(p_f, v_f, 5.0);
    Vec3 p_at_8 = predict_blind_position(p_f, v_f, 8.0);
    TEST_ASSERT_NEAR(p_at_8.data[0], p_at_5.data[0], 1e-12, "Blind position beyond 5s clamps to 5s pose");

    // Stationary hold when |v_f| < 0.05
    Vec3 v_slow = make_vec3(0.01, 0.02, 0.01);
    Vec3 p_slow = predict_blind_position(p_f, v_slow, 2.0);
    TEST_ASSERT_NEAR(p_slow.data[0], p_f.data[0], 1e-12, "Slow velocity holds stationary position");

    // Orientation extrapolation
    Quat pred_q = predict_blind_orientation(q_f, omega_f, tau);
    TEST_ASSERT(std::abs(pred_q.norm() - 1.0) < 1e-12, "Predicted quaternion must be normalized");
}

static void test_failure_detector_rollback() {
    FailureDetector detector;
    TEST_ASSERT(!detector.is_failed(), "Initially not failed");

    int64_t t_ns = 1000000000LL;
    int64_t dt_ns = 5000000LL; // 5 ms (200 Hz)

    // Feed 100 valid samples (0.5 s)
    for (int i = 0; i < 100; ++i) {
        Vec3 p = make_vec3(0.01 * i, 0.0, 0.0);
        Vec3 v = make_vec3(1.0, 0.0, 0.0);
        Quat q = Quat::identity();
        Vec3 omega_s = make_vec3(0.0, 0.0, 0.0);

        bool failed = detector.update(t_ns, true, p, v, q, omega_s);
        TEST_ASSERT(!failed, "Should not fail on valid samples");
        t_ns += dt_ns;
    }

    int64_t last_valid_t = t_ns - dt_ns;

    // Feed invalid samples for 0.8 s (< T_fail = 1.0 s)
    for (int i = 0; i < 160; ++i) {
        Vec3 p = make_vec3(0.0, 0.0, 0.0);
        Vec3 v = make_vec3(0.0, 0.0, 0.0);
        Quat q = Quat::identity();
        Vec3 omega_s = make_vec3(0.0, 0.0, 0.0);

        bool failed = detector.update(t_ns, false, p, v, q, omega_s);
        TEST_ASSERT(!failed, "Should not fail when invalid duration < 1.0 s");
        t_ns += dt_ns;
    }

    // Feed invalid samples until duration reaches 1.0 s (another 0.25 s = 50 samples)
    bool declared = false;
    for (int i = 0; i < 50; ++i) {
        Vec3 p = make_vec3(0.0, 0.0, 0.0);
        Vec3 v = make_vec3(0.0, 0.0, 0.0);
        Quat q = Quat::identity();
        Vec3 omega_s = make_vec3(0.0, 0.0, 0.0);

        if (detector.update(t_ns, false, p, v, q, omega_s)) {
            declared = true;
            break;
        }
        t_ns += dt_ns;
    }

    TEST_ASSERT(declared, "Failure detector should declare failure after 1.0 s of invalid samples");
    TEST_ASSERT(detector.is_failed(), "Detector state should be failed");
    TEST_ASSERT(detector.frozen_state().t_ns <= last_valid_t, "Rolled-back state must be pre-failure");
}

int main() {
    std::printf("Running WP-33 module unit tests...\n");

    test_tilt_update();
    test_zupt_update();
    test_stationary_buffer();
    test_degradation_monitor();
    test_gap_classifier();
    test_gap_substepping_decay();
    test_blind_extrapolation_math();
    test_failure_detector_rollback();

    std::printf("Tests run: %d, Tests failed: %d\n", g_tests_run, g_tests_failed);
    if (g_tests_failed > 0) {
        std::printf("SOME TESTS FAILED!\n");
        return 1;
    }

    std::printf("ALL MODULE TESTS PASSED!\n");
    return 0;
}
