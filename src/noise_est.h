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

    double ewma_var_g = 0.0;
    double ewma_var_a = 0.0;
    double x_prev[3] = {0.0, 0.0, 0.0};
    double x_prev2[3] = {0.0, 0.0, 0.0};
    double a_prev[3] = {0.0, 0.0, 0.0};
    double a_prev2[3] = {0.0, 0.0, 0.0};
    int count = 0;

    void reset_window() {
        count = 0;
    }

    void reset() {
        ewma_var_g = 0.0;
        ewma_var_a = 0.0;
        for (int i = 0; i < 3; i++) {
            x_prev[i] = 0.0;
            x_prev2[i] = 0.0;
            a_prev[i] = 0.0;
            a_prev2[i] = 0.0;
        }
        count = 0;
    }

    void update(const math::Vec3& omega, const math::Vec3& accel) {
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

        double sd_g = 0.0;
        double sd_a = 0.0;
        for (int i = 0; i < 3; i++) {
            double d_g = omega.data[i] - 2.0 * x_prev[i] + x_prev2[i];
            double d_a = accel.data[i] - 2.0 * a_prev[i] + a_prev2[i];
            sd_g += d_g * d_g;
            sd_a += d_a * d_a;
        }
        if (std::isfinite(sd_g)) {
            ewma_var_g += alpha * (sd_g / 3.0 - ewma_var_g);
        }
        if (std::isfinite(sd_a)) {
            ewma_var_a += alpha * (sd_a / 3.0 - ewma_var_a);
        }

        for (int i = 0; i < 3; i++) {
            x_prev2[i] = x_prev[i];
            x_prev[i] = omega.data[i];
            a_prev2[i] = a_prev[i];
            a_prev[i] = accel.data[i];
        }
        count++;
    }

    double get_gyro_mult() const {
        double sigma_hat = std::sqrt(std::max(ewma_var_g / 6.0, 1e-12));
        double kappa = sigma_hat / params::sigma_g_base;
        return std::clamp(kappa, min_mult, max_mult);
    }

    double get_acc_mult() const {
        double sigma_hat = std::sqrt(std::max(ewma_var_a / 6.0, 1e-12));
        double kappa = sigma_hat / params::sigma_a_base;
        return std::clamp(kappa, min_mult, max_mult);
    }
};

#endif // NOISE_EST_H
