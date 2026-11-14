#include <cassert>
#include <cmath>
#include <cstdio>
#include "src/noise_est.h"

void test_initial_and_clamp() {
    AdaptiveQEstimator est;
    assert(est.get_gyro_mult() == 1.0);
    assert(est.get_acc_mult() == 1.0);
    std::printf("[PASS] test_initial_and_clamp\n");
}

void test_clean_constant_signal() {
    AdaptiveQEstimator est;
    math::Vec3 w = math::make_vec3(0.01, -0.02, 0.03);
    math::Vec3 a = math::make_vec3(0.0, 0.0, 9.81);
    for (int i = 0; i < 500; ++i) {
        est.update(w, a);
    }
    // With zero noise, second difference is 0, mult clamps to min_mult = 1.0
    assert(est.get_gyro_mult() == 1.0);
    assert(est.get_acc_mult() == 1.0);
    std::printf("[PASS] test_clean_constant_signal\n");
}

void test_elevated_noise() {
    AdaptiveQEstimator est;
    est.min_mult = 1.0;
    est.max_mult = 25.0;

    // Simulate white noise with sigma = 0.06 (4x nominal gyro noise base of 0.015)
    // Simple deterministic pseudo-random generator (LCG)
    uint32_t state = 12345;
    auto rand_normal = [&]() {
        // Box-Muller transform from LCG
        state = state * 1664525u + 1013904223u;
        double u1 = (state + 1.0) / 4294967296.0;
        state = state * 1664525u + 1013904223u;
        double u2 = (state + 1.0) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
    };

    double target_sigma_g = 0.06; // 4x params::sigma_g_base (0.015)
    double target_sigma_a = 0.32; // 4x params::sigma_a_base (0.08)

    for (int i = 0; i < 2000; ++i) {
        math::Vec3 w = math::make_vec3(rand_normal() * target_sigma_g,
                                       rand_normal() * target_sigma_g,
                                       rand_normal() * target_sigma_g);
        math::Vec3 a = math::make_vec3(rand_normal() * target_sigma_a,
                                       rand_normal() * target_sigma_a,
                                       9.81 + rand_normal() * target_sigma_a);
        est.update(w, a);
    }

    double mult_g = est.get_gyro_mult();
    double mult_a = est.get_acc_mult();

    // Expected multiplier is ~4.0 (within 15% margin due to sample variance)
    assert(std::abs(mult_g - 4.0) < 0.6);
    assert(std::abs(mult_a - 4.0) < 0.6);
    std::printf("[PASS] test_elevated_noise: mult_g=%.2f, mult_a=%.2f (expected ~4.0)\n", mult_g, mult_a);
}

void test_nan_and_gap_handling() {
    AdaptiveQEstimator est;
    math::Vec3 w = math::make_vec3(0.1, 0.1, 0.1);
    math::Vec3 a = math::make_vec3(0.0, 0.0, 9.81);
    for (int i = 0; i < 100; ++i) {
        est.update(w, a);
    }

    // Inject NaN
    math::Vec3 w_nan = math::make_vec3(NAN, 0.0, 0.0);
    est.update(w_nan, a);
    assert(est.count == 0); // Reset window on invalid

    // Gap reset
    est.reset_window();
    assert(est.count == 0);

    // Filter should continue smoothly without NaNs
    for (int i = 0; i < 10; ++i) {
        est.update(w, a);
    }
    assert(std::isfinite(est.get_gyro_mult()));
    assert(std::isfinite(est.get_acc_mult()));
    std::printf("[PASS] test_nan_and_gap_handling\n");
}

int main() {
    std::printf("=== Running Adaptive Q Estimator Unit Tests ===\n");
    test_initial_and_clamp();
    test_clean_constant_signal();
    test_elevated_noise();
    test_nan_and_gap_handling();
    std::printf("=== All Adaptive Q Estimator Unit Tests Passed ===\n");
    return 0;
}
