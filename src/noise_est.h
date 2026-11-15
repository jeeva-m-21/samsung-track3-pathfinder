#ifndef NOISE_EST_H
#define NOISE_EST_H

#include "mat.h"
#include "params.h"
#include <algorithm>
#include <cmath>

struct AdaptiveQEstimator {
    double alpha = params::adaptive_q_alpha;
    double min_mult = params::adaptive_q_min;
    double max_mult = params::adaptive_q_max;

    // Unbiased spike-clamped MAD estimator state
    double sigma_hat_g = 0.0;
    double sigma_hat_a = 0.0;
    double sigma_hat_g_0 = 0.0;
    double sigma_hat_a_0 = 0.0;
    bool baseline_latched = false;
    double d_clamp_g = 0.5; // rad/s
    double d_clamp_a = 5.0; // m/s^2

    // Legacy EWMA variance state (for exact fallback when disabled)
    double ewma_var_g = 0.0;
    double ewma_var_a = 0.0;

    double x_prev[3] = {0.0, 0.0, 0.0};
    double x_prev2[3] = {0.0, 0.0, 0.0};
    double a_prev[3] = {0.0, 0.0, 0.0};
    double a_prev2[3] = {0.0, 0.0, 0.0};
    int count = 0;

    void reset_window() {
        // Reset 2nd difference buffer on gap boundaries without clearing smoothed sigma or latched baseline
        count = 0;
    }

    void reset() {
        ewma_var_g = 0.0;
        ewma_var_a = 0.0;
        sigma_hat_g = 0.0;
        sigma_hat_a = 0.0;
        sigma_hat_g_0 = 0.0;
        sigma_hat_a_0 = 0.0;
        baseline_latched = false;
        for (int i = 0; i < 3; i++) {
            x_prev[i] = 0.0;
            x_prev2[i] = 0.0;
            a_prev[i] = 0.0;
            a_prev2[i] = 0.0;
        }
        count = 0;
    }

    void update(const math::Vec3& omega, const math::Vec3& accel, double t_sec = -1.0) {
        for (int i = 0; i < 3; i++) {
            if (!std::isfinite(omega.data[i]) || !std::isfinite(accel.data[i])) {
                count = 0;
                return;
            }
        }

        if (count == 0) {
            for (int i = 0; i < 3; i++) {
                x_prev2[i] = omega.data[i];
                a_prev2[i] = accel.data[i];
            }
            count = 1;
            return;
        }
        if (count == 1) {
            for (int i = 0; i < 3; i++) {
                x_prev[i] = omega.data[i];
                a_prev[i] = accel.data[i];
            }
            count = 2;
            return;
        }

        // 2nd difference: d = x_k - 2*x_{k-1} + x_{k-2}
        double sum_abs_d_g = 0.0;
        double sum_abs_d_a = 0.0;
        double sd_g = 0.0;
        double sd_a = 0.0;
        for (int i = 0; i < 3; i++) {
            double d_g = omega.data[i] - 2.0 * x_prev[i] + x_prev2[i];
            double d_a = accel.data[i] - 2.0 * a_prev[i] + a_prev2[i];
            sum_abs_d_g += std::abs(d_g);
            sum_abs_d_a += std::abs(d_a);
            sd_g += d_g * d_g;
            sd_a += d_a * d_a;
        }

        // Legacy EWMA update
        if (std::isfinite(sd_g)) {
            ewma_var_g += alpha * (sd_g / 3.0 - ewma_var_g);
        }
        if (std::isfinite(sd_a)) {
            ewma_var_a += alpha * (sd_a / 3.0 - ewma_var_a);
        }

        // Spike-clamped Mean Absolute Deviation (MAD) EWMA
        // bar{d}_g = min((1/3)*sum |d_{g,i}|, d_clamp)
        double bar_d_g = std::min((1.0 / 3.0) * sum_abs_d_g, d_clamp_g);
        double bar_d_a = std::min((1.0 / 3.0) * sum_abs_d_a, d_clamp_a);

        // Theoretical scale factor for Gaussian noise:
        // E[|d|] = sqrt(12/pi) * sigma  =>  sigma = sqrt(pi/12) * E[|d|]
        // sqrt(pi/12) ~= 0.5116633539732442
        // Correction factor for 3-sample median attenuation on 2nd difference:
        // 1 / sqrt(0.485) ~= 1.4364023
        constexpr double SQRT_PI_OVER_12 = 0.5116633539732442;
        constexpr double MEDIAN_CORRECTION = 1.4364023;

        double inst_sigma_g = SQRT_PI_OVER_12 * bar_d_g * MEDIAN_CORRECTION;
        double inst_sigma_a = SQRT_PI_OVER_12 * bar_d_a * MEDIAN_CORRECTION;

        if (std::isfinite(inst_sigma_g)) {
            if (sigma_hat_g <= 1e-12) {
                sigma_hat_g = inst_sigma_g;
            } else {
                sigma_hat_g = (1.0 - alpha) * sigma_hat_g + alpha * inst_sigma_g;
            }
        }
        if (std::isfinite(inst_sigma_a)) {
            if (sigma_hat_a <= 1e-12) {
                sigma_hat_a = inst_sigma_a;
            } else {
                sigma_hat_a = (1.0 - alpha) * sigma_hat_a + alpha * inst_sigma_a;
            }
        }

        // Latch baseline sigma_hat_{g,0} in the first 2 seconds
        if (t_sec >= 0.0) {
            if (t_sec <= 2.0) {
                sigma_hat_g_0 = sigma_hat_g;
                sigma_hat_a_0 = sigma_hat_a;
            } else {
                baseline_latched = true;
            }
        } else {
            if (count <= 400) {
                sigma_hat_g_0 = sigma_hat_g;
                sigma_hat_a_0 = sigma_hat_a;
            } else {
                baseline_latched = true;
            }
        }

        for (int i = 0; i < 3; i++) {
            x_prev2[i] = x_prev[i];
            x_prev[i] = omega.data[i];
            a_prev2[i] = a_prev[i];
            a_prev[i] = accel.data[i];
        }
        count++;
    }

    double get_noise_excess_ratio() const {
        if (!params::enable_unbiased_adaptive_q) {
            return 1.0;
        }
        double base_g = std::max((sigma_hat_g_0 > 1e-6) ? sigma_hat_g_0 : params::sigma_g_base, params::adaptive_q_ref);
        double rho = sigma_hat_g / base_g;
        return (rho > 1.0) ? rho : 1.0;
    }

    double get_gyro_mult() const {
        if (!params::enable_unbiased_adaptive_q) {
            double sigma_hat = std::sqrt(std::max(ewma_var_g / 6.0, 1e-12));
            double kappa = sigma_hat / params::sigma_g_base;
            return std::clamp(kappa, min_mult, max_mult);
        }

        double base_g = std::max((sigma_hat_g_0 > 1e-6) ? sigma_hat_g_0 : params::sigma_g_base, params::adaptive_q_ref);
        double kappa = sigma_hat_g / base_g;
        return std::clamp(kappa, min_mult, max_mult);
    }

    double get_acc_mult() const {
        if (!params::enable_unbiased_adaptive_q) {
            double sigma_hat = std::sqrt(std::max(ewma_var_a / 6.0, 1e-12));
            double kappa = sigma_hat / params::sigma_a_base;
            return std::clamp(kappa, min_mult, max_mult);
        }

        double base_a = (sigma_hat_a_0 > 1e-6) ? sigma_hat_a_0 : params::sigma_a_base;
        double kappa = sigma_hat_a / base_a;
        return std::clamp(kappa, min_mult, max_mult);
    }
};

#endif // NOISE_EST_H
