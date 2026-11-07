#include "monitor.h"
#include <algorithm>
#include <cmath>

double compute_residual(const Vec3& curr, const Vec3& prev) {
    return (curr - prev).norm();
}

double update_moving_variance(double prev_var, double residual, double alpha) {
    double res_sq = residual * residual;
    return (1.0 - alpha) * prev_var + alpha * res_sq;
}

double compute_noise_ratio(double sigma_now, double sigma_ref) {
    if (sigma_ref <= 1e-12) {
        return 1.0;
    }
    double ratio = sigma_now / sigma_ref;
    return (ratio > 1.0) ? ratio : 1.0;
}

double compute_quality_score(double noise_ratio, double min_quality, double max_quality) {
    double n = (noise_ratio < 1e-12) ? 1e-12 : noise_ratio;
    double val = 1.0 / n;
    if (val < min_quality) return min_quality;
    if (val > max_quality) return max_quality;
    return val;
}

double compute_kappa(double noise_ratio, double kappa_max) {
    if (noise_ratio < 1.0) return 1.0;
    if (noise_ratio > kappa_max) return kappa_max;
    return noise_ratio;
}

Mat3x15 build_gyro_bias_h_matrix() {
    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 9, Mat3::identity());
    return H;
}

Mat3 build_gyro_bias_r_matrix(double r_scale) {
    double var_bgm = (params::sigma_bgm * params::sigma_bgm) * r_scale;
    return Mat3::identity() * var_bgm;
}

bool apply_gyro_bias_update(NominalState& state,
                            Mat15& P,
                            const Vec3& omega_meas,
                            double gate_threshold,
                            double r_scale) {
    Vec3 y = omega_meas - state.b_g;
    Mat3x15 H = build_gyro_bias_h_matrix();
    Mat3 R = build_gyro_bias_r_matrix(r_scale);

    return joseph_update(state, P, H, y, R, gate_threshold);
}

// ---------------------------------------------------------------------------
// DegradationMonitor implementation
// ---------------------------------------------------------------------------

DegradationMonitor::DegradationMonitor() {
    reset();
}

void DegradationMonitor::reset() {
    t_start_ = 0.0;
    t_prev_ = 0.0;
    sample_count_ = 0;
    prev_acc_ = math::make_vec3(0.0, 0.0, 0.0);
    prev_omega_ = math::make_vec3(0.0, 0.0, 0.0);
    var_acc_ = 0.0;
    var_gyro_ = 0.0;
    var_acc_ref_ = 0.0;
    var_gyro_ref_ = 0.0;
    calibrated_ = false;
    n_acc_ = 1.0;
    n_gyro_ = 1.0;
    noise_ratio_ = 1.0;
    quality_ = 1.0;
    kappa_ = 1.0;
    residual_acc_ = 0.0;
    residual_gyro_ = 0.0;
    bias_trend_ = math::make_vec3(0.0, 0.0, 0.0);
    bias_trend_count_ = 0;
}

void DegradationMonitor::step(double t,
                              const Vec3& omega,
                              const Vec3& acc,
                              bool is_stationary,
                              const Vec3* omega_static_mean) {
    if (sample_count_ == 0) {
        t_start_ = t;
        t_prev_ = t;
        prev_acc_ = acc;
        prev_omega_ = omega;
        sample_count_ = 1;
        residual_acc_ = 0.0;
        residual_gyro_ = 0.0;
        quality_ = 1.0;
        kappa_ = 1.0;
        noise_ratio_ = 1.0;
        calibrated_ = false;
        return;
    }

    // 1. High-frequency residuals
    double r_k = compute_residual(acc, prev_acc_);
    double rho_k = compute_residual(omega, prev_omega_);
    residual_acc_ = r_k;
    residual_gyro_ = rho_k;

    // 2. Exponential moving variance
    double alpha = params::monitor_alpha;
    if (sample_count_ == 1) {
        var_acc_ = r_k * r_k;
        var_gyro_ = rho_k * rho_k;
    } else {
        var_acc_ = update_moving_variance(var_acc_, r_k, alpha);
        var_gyro_ = update_moving_variance(var_gyro_, rho_k, alpha);
    }

    // 3. Calibration over initial static window (T_init = 1.0 s)
    double dt_elapsed = t - t_start_;
    if (dt_elapsed < params::T_init) {
        calibrated_ = false;
        var_acc_ref_ = var_acc_;
        var_gyro_ref_ = var_gyro_;
        n_acc_ = 1.0;
        n_gyro_ = 1.0;
        noise_ratio_ = 1.0;
        quality_ = 1.0;
        kappa_ = 1.0;
    } else {
        if (!calibrated_) {
            calibrated_ = true;
            if (var_acc_ref_ <= 0.0) var_acc_ref_ = var_acc_;
            if (var_gyro_ref_ <= 0.0) var_gyro_ref_ = var_gyro_;
        }

        double sigma_a_now = std::sqrt((var_acc_ > 0.0) ? var_acc_ : 0.0);
        double sigma_g_now = std::sqrt((var_gyro_ > 0.0) ? var_gyro_ : 0.0);
        double sigma_a_ref = std::sqrt((var_acc_ref_ > 0.0) ? var_acc_ref_ : 0.0);
        double sigma_g_ref = std::sqrt((var_gyro_ref_ > 0.0) ? var_gyro_ref_ : 0.0);

        n_acc_ = compute_noise_ratio(sigma_a_now, sigma_a_ref);
        n_gyro_ = compute_noise_ratio(sigma_g_now, sigma_g_ref);
        noise_ratio_ = (n_acc_ > n_gyro_) ? n_acc_ : n_gyro_;
        if (noise_ratio_ < 1.0) noise_ratio_ = 1.0;

        quality_ = compute_quality_score(noise_ratio_);
        kappa_ = compute_kappa(noise_ratio_, params::kappa_max);
    }

    // 4. Stationary gyro bias trend tracking
    if (is_stationary && omega_static_mean != nullptr) {
        if (bias_trend_count_ == 0) {
            bias_trend_ = *omega_static_mean;
        } else {
            bias_trend_ = (1.0 - alpha) * bias_trend_ + alpha * (*omega_static_mean);
        }
        bias_trend_count_++;
    }

    prev_acc_ = acc;
    prev_omega_ = omega;
    t_prev_ = t;
    sample_count_++;
}
