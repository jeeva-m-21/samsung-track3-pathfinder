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

// Check if a 15x15 symmetric matrix is positive-definite via Cholesky decomposition
static bool is_positive_definite_cholesky(const Mat15& P) {
    Mat15 L = Mat15::zeros();
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }
            if (i == j) {
                double val = P(i, i) - sum;
                if (val <= 0.0) {
                    return false;
                }
                L(i, j) = std::sqrt(val);
            } else {
                if (L(j, j) < 1e-15) {
                    return false;
                }
                L(i, j) = (P(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

// 1. Initial tilt on canonical axes and arbitrary vectors
void test_initial_tilt() {
    std::printf("Running test_initial_tilt...\n");

    // Case 1: Upright [0, 0, 9.81]
    Vec3 a_up = make_vec3(0.0, 0.0, 9.81);
    Quat q_up = compute_initial_tilt(a_up);
    TEST_ASSERT_NEAR(q_up.w, 1.0, 1e-12, "q_up.w == 1");
    TEST_ASSERT_NEAR(q_up.x, 0.0, 1e-12, "q_up.x == 0");
    TEST_ASSERT_NEAR(q_up.y, 0.0, 1e-12, "q_up.y == 0");
    TEST_ASSERT_NEAR(q_up.z, 0.0, 1e-12, "q_up.z == 0");

    Vec3 a_rot = q_up.to_rot_mat() * (a_up / 9.81);
    TEST_ASSERT_NEAR(a_rot.data[0], 0.0, 1e-12, "aligned x == 0");
    TEST_ASSERT_NEAR(a_rot.data[1], 0.0, 1e-12, "aligned y == 0");
    TEST_ASSERT_NEAR(a_rot.data[2], 1.0, 1e-12, "aligned z == 1");

    // Case 2: Inverted [0, 0, -9.81] (180 deg pitch)
    Vec3 a_down = make_vec3(0.0, 0.0, -9.81);
    Quat q_down = compute_initial_tilt(a_down);
    Vec3 a_rot_down = q_down.to_rot_mat() * (a_down / 9.81);
    TEST_ASSERT_NEAR(a_rot_down.data[0], 0.0, 1e-12, "down aligned x == 0");
    TEST_ASSERT_NEAR(a_rot_down.data[1], 0.0, 1e-12, "down aligned y == 0");
    TEST_ASSERT_NEAR(a_rot_down.data[2], 1.0, 1e-12, "down aligned z == 1");

    // Case 3: Horizontal x [9.81, 0, 0]
    Vec3 a_x = make_vec3(9.81, 0.0, 0.0);
    Quat q_x = compute_initial_tilt(a_x);
    Vec3 a_rot_x = q_x.to_rot_mat() * (a_x / 9.81);
    TEST_ASSERT_NEAR(a_rot_x.data[0], 0.0, 1e-12, "x aligned x == 0");
    TEST_ASSERT_NEAR(a_rot_x.data[1], 0.0, 1e-12, "x aligned y == 0");
    TEST_ASSERT_NEAR(a_rot_x.data[2], 1.0, 1e-12, "x aligned z == 1");

    // Case 4: Horizontal y [0, 9.81, 0]
    Vec3 a_y = make_vec3(0.0, 9.81, 0.0);
    Quat q_y = compute_initial_tilt(a_y);
    Vec3 a_rot_y = q_y.to_rot_mat() * (a_y / 9.81);
    TEST_ASSERT_NEAR(a_rot_y.data[0], 0.0, 1e-12, "y aligned x == 0");
    TEST_ASSERT_NEAR(a_rot_y.data[1], 0.0, 1e-12, "y aligned y == 0");
    TEST_ASSERT_NEAR(a_rot_y.data[2], 1.0, 1e-12, "y aligned z == 1");

    // Case 5: Arbitrary diagonal vector [1.5, -2.3, 9.2]
    Vec3 a_diag = make_vec3(1.5, -2.3, 9.2);
    Quat q_diag = compute_initial_tilt(a_diag);
    Vec3 a_rot_diag = q_diag.to_rot_mat() * (a_diag / a_diag.norm());
    TEST_ASSERT_NEAR(a_rot_diag.data[0], 0.0, 1e-12, "diag aligned x == 0");
    TEST_ASSERT_NEAR(a_rot_diag.data[1], 0.0, 1e-12, "diag aligned y == 0");
    TEST_ASSERT_NEAR(a_rot_diag.data[2], 1.0, 1e-12, "diag aligned z == 1");
    // Rotation about v x u has zero z-component, so q_z must be identically 0 (zero yaw)
    TEST_ASSERT_NEAR(q_diag.z, 0.0, 1e-12, "diag q.z == 0 (zero yaw)");
}

// 2. Static initialization window
void test_static_initialization() {
    std::printf("Running test_static_initialization...\n");

    Initializer init;
    TEST_ASSERT(!init.is_ready(), "Initializer starts not ready");

    int64_t t0 = 1000000000LL; // 1 second in ns
    int64_t dt_ns = 5000000LL;  // 5 ms (200 Hz)
    Vec3 true_bg = make_vec3(0.015, -0.02, 0.005);
    Vec3 true_acc = make_vec3(0.1, -0.2, 9.80);

    // Feed 201 samples (1.000 s)
    bool became_ready = false;
    for (int i = 0; i <= 200; ++i) {
        int64_t t = t0 + i * dt_ns;
        // Deterministic pseudo-noise
        double noise_w = 0.001 * std::sin(static_cast<double>(i));
        double noise_a = 0.002 * std::cos(static_cast<double>(i));
        Vec3 w = true_bg + make_vec3(noise_w, -noise_w, noise_w * 0.5);
        Vec3 a = true_acc + make_vec3(noise_a, noise_a * 0.5, -noise_a);

        bool ready = init.add_sample(t, w, a);
        if (ready && !became_ready) {
            became_ready = true;
            TEST_ASSERT(i == 200, "Should become ready at exactly i=200 (1.0 s)");
        }
    }

    TEST_ASSERT(init.is_ready(), "Initializer is ready after 1.0s");

    NominalState state;
    Mat15 P0;
    Vec3 g_w;
    init.finish(state, P0, g_w);

    TEST_ASSERT(!init.is_motion_detected(), "Motion should not be detected");
    TEST_ASSERT_NEAR(state.p.norm(), 0.0, 1e-15, "p0 == 0");
    TEST_ASSERT_NEAR(state.v.norm(), 0.0, 1e-15, "v0 == 0");
    TEST_ASSERT_NEAR(state.b_a.norm(), 0.0, 1e-15, "b_a0 == 0");

    // Estimated gyro bias should be close to true_bg
    TEST_ASSERT_NEAR(state.b_g.data[0], true_bg.data[0], 0.002, "b_g.x near true_bg.x");
    TEST_ASSERT_NEAR(state.b_g.data[1], true_bg.data[1], 0.002, "b_g.y near true_bg.y");
    TEST_ASSERT_NEAR(state.b_g.data[2], true_bg.data[2], 0.002, "b_g.z near true_bg.z");

    // Gravity magnitude and vector
    double expected_g = true_acc.norm();
    TEST_ASSERT_NEAR(init.g_mag(), expected_g, 0.01, "g_mag near true");
    TEST_ASSERT_NEAR(g_w.data[0], 0.0, 1e-15, "g_w.x == 0");
    TEST_ASSERT_NEAR(g_w.data[1], 0.0, 1e-15, "g_w.y == 0");
    TEST_ASSERT_NEAR(g_w.data[2], -init.g_mag(), 1e-15, "g_w.z == -g_mag");

    // Covariance diagonal
    double exp_theta = params::sigma_theta_0 * params::sigma_theta_0;
    double exp_v = params::sigma_v_0 * params::sigma_v_0;
    double exp_p = params::sigma_p_0 * params::sigma_p_0;
    double exp_bg = params::sigma_bg_0 * params::sigma_bg_0;
    double exp_ba = params::sigma_ba_0 * params::sigma_ba_0;

    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(P0(i, i), exp_theta, 1e-12, "P0 theta diag");
        TEST_ASSERT_NEAR(P0(3 + i, 3 + i), exp_v, 1e-12, "P0 v diag");
        TEST_ASSERT_NEAR(P0(6 + i, 6 + i), exp_p, 1e-12, "P0 p diag");
        TEST_ASSERT_NEAR(P0(9 + i, 9 + i), exp_bg, 1e-12, "P0 bg diag");
        TEST_ASSERT_NEAR(P0(12 + i, 12 + i), exp_ba, 1e-12, "P0 ba diag");
    }
}

// 3. Motion fallback initialization
void test_moving_start_fallback() {
    std::printf("Running test_moving_start_fallback...\n");

    Initializer init;
    int64_t t0 = 0;
    int64_t dt_ns = 5000000LL;

    // Feed samples with high gyro standard deviation (> 0.2 rad/s)
    for (int i = 0; i <= 200; ++i) {
        int64_t t = t0 + i * dt_ns;
        double w_dyn = 0.5 * std::sin(static_cast<double>(i) * 0.2); // std ~ 0.35 > 0.2
        Vec3 w = make_vec3(w_dyn, 0.0, 0.0);
        Vec3 a = make_vec3(0.0, 0.0, 9.81);
        init.add_sample(t, w, a);
    }

    NominalState state;
    Mat15 P0;
    Vec3 g_w;
    init.finish(state, P0, g_w);

    TEST_ASSERT(init.is_motion_detected(), "Motion detected flag must be true");
    // Fallback: zero gyro bias
    TEST_ASSERT_NEAR(state.b_g.norm(), 0.0, 1e-15, "Fallback b_g == 0");
    TEST_ASSERT_NEAR(state.b_a.norm(), 0.0, 1e-15, "Fallback b_a == 0");

    // Covariance: sigma_theta multiplied by motion_detect_sigma_theta_mult (5.0)
    double scaled_sigma_th = params::sigma_theta_0 * params::motion_detect_sigma_theta_mult;
    double exp_theta = scaled_sigma_th * scaled_sigma_th;
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(P0(i, i), exp_theta, 1e-12, "P0 theta diag 5x scaled");
    }
}

// 4. Strapdown step exact equations
void test_strapdown_step_exact() {
    std::printf("Running test_strapdown_step_exact...\n");

    NominalState s0;
    s0.p = make_vec3(10.0, -20.0, 5.0);
    s0.v = make_vec3(1.2, -0.5, 0.8);
    s0.q = normalize(Quat{0.70710678, 0.0, 0.70710678, 0.0});
    s0.b_g = make_vec3(0.01, -0.02, 0.03);
    s0.b_a = make_vec3(0.05, 0.02, -0.04);

    Vec3 omega = make_vec3(0.1, 0.2, -0.15);
    Vec3 acc = make_vec3(0.3, -0.1, 9.85);
    double dt = 0.01;
    Vec3 g_w = make_vec3(0.0, 0.0, -9.80665);

    // Hand-calculate expected next state
    Vec3 w_hat = omega - s0.b_g;
    Vec3 a_hat = acc - s0.b_a;

    Quat delta_q = exp_map(w_hat * dt);
    Quat expected_q = normalize(s0.q * delta_q);
    ensure_continuity(expected_q, s0.q);

    Mat3 R_old = to_rot_mat(s0.q);
    Vec3 a_w = R_old * a_hat + g_w;
    Vec3 expected_p = s0.p + s0.v * dt + a_w * (0.5 * dt * dt);
    Vec3 expected_v = s0.v + a_w * dt;

    NominalState s1 = strapdown_step(s0, omega, acc, dt, g_w);

    TEST_ASSERT_NEAR(s1.p.data[0], expected_p.data[0], 1e-14, "s1.p.x");
    TEST_ASSERT_NEAR(s1.p.data[1], expected_p.data[1], 1e-14, "s1.p.y");
    TEST_ASSERT_NEAR(s1.p.data[2], expected_p.data[2], 1e-14, "s1.p.z");

    TEST_ASSERT_NEAR(s1.v.data[0], expected_v.data[0], 1e-14, "s1.v.x");
    TEST_ASSERT_NEAR(s1.v.data[1], expected_v.data[1], 1e-14, "s1.v.y");
    TEST_ASSERT_NEAR(s1.v.data[2], expected_v.data[2], 1e-14, "s1.v.z");

    TEST_ASSERT_NEAR(s1.q.w, expected_q.w, 1e-14, "s1.q.w");
    TEST_ASSERT_NEAR(s1.q.x, expected_q.x, 1e-14, "s1.q.x");
    TEST_ASSERT_NEAR(s1.q.y, expected_q.y, 1e-14, "s1.q.y");
    TEST_ASSERT_NEAR(s1.q.z, expected_q.z, 1e-14, "s1.q.z");

    // Input state s0 must not be mutated
    TEST_ASSERT_NEAR(s0.p.data[0], 10.0, 1e-15, "s0 unchanged");
}

// 5. Zero-noise static IMU drift over 60s < 1e-6 m
void test_strapdown_static_drift_60s() {
    std::printf("Running test_strapdown_static_drift_60s...\n");

    const double rate = 200.0;
    const double dt = 1.0 / rate;
    const size_t n_steps = 12000; // 60 seconds
    const double g = 9.80665;
    const Vec3 g_w = make_vec3(0.0, 0.0, -g);

    // Test 1: Upright orientation
    {
        NominalState state = NominalState::zeros();
        Vec3 acc = make_vec3(0.0, 0.0, g);
        Vec3 omega = make_vec3(0.0, 0.0, 0.0);

        for (size_t i = 0; i < n_steps; ++i) {
            state = strapdown_step(state, omega, acc, dt, g_w);
        }

        double drift = state.p.norm();
        TEST_ASSERT(drift < 1e-6, "Upright drift over 60s < 1e-6m");
        TEST_ASSERT_NEAR(state.v.norm(), 0.0, 1e-12, "Upright velocity == 0");
    }

    // Test 2: Arbitrarily tilted orientation
    {
        // 45 deg roll, 30 deg pitch
        Quat q_roll = exp_map(make_vec3(0.7853981633974483, 0.0, 0.0));
        Quat q_pitch = exp_map(make_vec3(0.0, 0.5235987755982988, 0.0));
        Quat q_init = normalize(q_roll * q_pitch);

        // Body accelerometer reading exactly cancels g_w in world frame:
        // a_w = R(q) * a_body + g_w = 0 => a_body = R(q)^T * (-g_w)
        Vec3 a_body = q_init.to_rot_mat().transpose() * (-g_w);
        Vec3 omega = make_vec3(0.0, 0.0, 0.0);

        NominalState state;
        state.p = make_vec3(0.0, 0.0, 0.0);
        state.v = make_vec3(0.0, 0.0, 0.0);
        state.q = q_init;
        state.b_g = make_vec3(0.0, 0.0, 0.0);
        state.b_a = make_vec3(0.0, 0.0, 0.0);

        for (size_t i = 0; i < n_steps; ++i) {
            state = strapdown_step(state, omega, a_body, dt, g_w);
        }

        double drift = state.p.norm();
        TEST_ASSERT(drift < 1e-6, "Tilted drift over 60s < 1e-6m");
        TEST_ASSERT_NEAR(state.v.norm(), 0.0, 1e-12, "Tilted velocity == 0");
    }
}

// 6. Covariance propagation F and Q construction
void test_f_and_q_matrices() {
    std::printf("Running test_f_and_q_matrices...\n");

    NominalState state;
    state.p = make_vec3(0.0, 0.0, 0.0);
    state.v = make_vec3(0.0, 0.0, 0.0);
    state.q = Quat::identity();
    state.b_g = make_vec3(0.01, -0.02, 0.03);
    state.b_a = make_vec3(0.05, -0.05, 0.1);

    Vec3 omega = make_vec3(0.1, -0.2, 0.3);
    Vec3 acc = make_vec3(0.5, 0.2, 9.8);
    double dt = 0.005;

    Mat15 F = build_f_matrix(state, omega, acc, dt);

    // Diagonal elements of F must be near 1.0 (since R(omega*dt) ~ I for small dt)
    for (size_t i = 0; i < 15; ++i) {
        TEST_ASSERT_NEAR(F(i, i), 1.0, 0.01, "F diagonal near 1.0");
    }

    // F[0:3, 9:12] must be -I * dt
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(F(i, 9 + i), -dt, 1e-14, "F gyro bias block == -dt");
    }

    // F[6:9, 3:6] must be I * dt
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(F(6 + i, 3 + i), dt, 1e-14, "F velocity-to-position block == dt");
    }

    // Q matrix
    Mat15 Q = build_q_matrix(dt, params::noise_mult_gyro, params::noise_mult_acc,
                             params::walk_mult_bg, params::walk_mult_ba, 1.0);

    // Q must be symmetric
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = 0; j < 15; ++j) {
            TEST_ASSERT_NEAR(Q(i, j), Q(j, i), 1e-15, "Q symmetric");
        }
    }

    // Q position block (6..8) must be exactly zero
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(Q(6 + i, 6 + i), 0.0, 1e-15, "Q position block == 0");
    }

    // Other diagonal blocks must be strictly positive
    TEST_ASSERT(Q(0, 0) > 0.0, "Q theta block > 0");
    TEST_ASSERT(Q(3, 3) > 0.0, "Q v block > 0");
    TEST_ASSERT(Q(9, 9) > 0.0, "Q bg block > 0");
    TEST_ASSERT(Q(12, 12) > 0.0, "Q ba block > 0");
}

// 7. Covariance propagation over 10,000 steps without updates
void test_covariance_propagation_10000_steps() {
    std::printf("Running test_covariance_propagation_10000_steps...\n");

    Mat15 P = initialize_covariance(false);
    NominalState state = NominalState::zeros();
    double dt = 0.005;

    Vec3 omega = make_vec3(0.05, -0.08, 0.12);
    Vec3 acc = make_vec3(0.3, -0.4, 9.81);

    Mat15 F = build_f_matrix(state, omega, acc, dt);
    Mat15 Q = build_q_matrix(dt, params::noise_mult_gyro, params::noise_mult_acc,
                             params::walk_mult_bg, params::walk_mult_ba, 1.0);

    for (size_t step = 0; step < 10000; ++step) {
        P = propagate_covariance(P, F, Q);
    }

    // 1. Check symmetry
    double max_asym = 0.0;
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = i + 1; j < 15; ++j) {
            double diff = std::abs(P(i, j) - P(j, i));
            if (diff > max_asym) max_asym = diff;
        }
    }
    TEST_ASSERT(max_asym < 1e-12, "P max asymmetry after 10000 steps < 1e-12");

    // 2. Check diagonal bounds
    for (size_t i = 0; i < 15; ++i) {
        TEST_ASSERT(P(i, i) >= params::cov_diag_floor, "P diag >= floor");
        TEST_ASSERT(P(i, i) <= params::cov_diag_cap, "P diag <= cap");
    }

    // 3. Positive definiteness via Cholesky
    bool is_pd = is_positive_definite_cholesky(P);
    TEST_ASSERT(is_pd, "P remains strictly positive definite after 10000 steps");
}

// 8. Covariance conditioning with congruence transformation
void test_covariance_conditioning() {
    std::printf("Running test_covariance_conditioning...\n");

    Mat15 P = Mat15::identity();
    P(0, 1) = 0.5;
    P(1, 0) = 0.1;   // Asymmetric
    P(0, 0) = 1e-16; // Below floor
    P(1, 1) = -5.0;  // Negative
    P(2, 2) = 1e8;   // Above cap (1e6)

    condition_covariance(P, 1e-12, 1e6);

    // Check symmetrization
    TEST_ASSERT_NEAR(P(0, 1), 0.3, 1e-14, "P(0,1) symmetrised");
    TEST_ASSERT_NEAR(P(1, 0), 0.3, 1e-14, "P(1,0) symmetrised");

    // Check floor
    TEST_ASSERT_NEAR(P(0, 0), 1e-12, 1e-15, "P(0,0) floored");
    TEST_ASSERT_NEAR(P(1, 1), 1e-12, 1e-15, "P(1,1) floored");

    // Check cap
    TEST_ASSERT_NEAR(P(2, 2), 1e6, 1e-12, "P(2,2) capped");
}

// 9. Generic Joseph-form measurement update
void test_joseph_update_and_reset() {
    std::printf("Running test_joseph_update_and_reset...\n");

    NominalState state = NominalState::zeros();
    state.v = make_vec3(0.5, -0.3, 0.2);

    Mat15 P = initialize_covariance(false);

    // Measurement: observation of velocity y = -v, H has I3 at [0:3, 3:6]
    Mat3x15 H = Mat3x15::zeros();
    for (size_t i = 0; i < 3; ++i) {
        H(i, 3 + i) = 1.0;
    }

    Vec3 y = make_vec3(-0.5, 0.3, -0.2); // Innovate towards zero velocity
    double sigma_v = 0.05;
    Mat3 R = Mat3::identity() * (sigma_v * sigma_v);

    double prior_trace_v = P(3, 3) + P(4, 4) + P(5, 5);

    bool updated = joseph_update(state, P, H, y, R, 1e9);
    TEST_ASSERT(updated, "Joseph update should succeed with wide gate");

    // Velocity should be corrected towards zero
    TEST_ASSERT(state.v.norm() < 0.5, "Velocity error reduced after update");

    // Posterior velocity covariance should shrink
    double post_trace_v = P(3, 3) + P(4, 4) + P(5, 5);
    TEST_ASSERT(post_trace_v < prior_trace_v, "Velocity covariance trace shrunk");

    // Covariance must remain symmetric and positive definite
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = i + 1; j < 15; ++j) {
            TEST_ASSERT_NEAR(P(i, j), P(j, i), 1e-14, "Posterior P symmetric");
        }
    }
    TEST_ASSERT(is_positive_definite_cholesky(P), "Posterior P is positive definite");

    // Test innovation gate rejection: tiny gate threshold should reject large innovation
    Vec3 huge_y = make_vec3(100.0, 100.0, 100.0);
    NominalState state_before = state;
    Mat15 P_before = P;

    bool rejected = joseph_update(state, P, H, huge_y, R, 1.0); // gate_threshold = 1.0
    TEST_ASSERT(!rejected, "Outlier measurement must be rejected by gate");
    TEST_ASSERT_NEAR(state.v.data[0], state_before.v.data[0], 1e-15, "State unmutated on rejection");
    TEST_ASSERT_NEAR(P(3, 3), P_before(3, 3), 1e-15, "P unmutated on rejection");
}

int main() {
    std::printf("===========================================\n");
    std::printf("Running C++ ESKF & Initializer Unit Tests\n");
    std::printf("===========================================\n");

    test_initial_tilt();
    test_static_initialization();
    test_moving_start_fallback();
    test_strapdown_step_exact();
    test_strapdown_static_drift_60s();
    test_f_and_q_matrices();
    test_covariance_propagation_10000_steps();
    test_covariance_conditioning();
    test_joseph_update_and_reset();

    std::printf("===========================================\n");
    std::printf("Test results: %d run, %d failed\n", g_tests_run, g_tests_failed);
    std::printf("===========================================\n");

    return (g_tests_failed == 0) ? 0 : 1;
}
