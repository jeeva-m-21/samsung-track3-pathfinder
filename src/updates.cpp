#include "updates.h"
#include <cmath>

bool tilt_gating(const Vec3& acc,
                 const Vec3& omega,
                 double g_mag,
                 double quality) {
    double omega_norm = omega.norm();
    if (omega_norm >= params::tilt_omega_max) {
        return false;
    }

    double q_clamped = (quality < 0.0) ? 0.0 : ((quality > 1.0) ? 1.0 : quality);
    double tau_eff = params::tilt_tau_a * (1.0 + 5.0 * (1.0 - q_clamped));

    double acc_norm = acc.norm();
    double acc_dev = std::abs(acc_norm - g_mag);
    return (acc_dev < tau_eff);
}

void compute_tilt_innovation(const NominalState& state,
                             const Vec3& acc,
                             const Vec3& g_w,
                             Vec3& y,
                             Vec3& h_hat) {
    Vec3 a_unbiased = acc - state.b_a;
    double norm_a = a_unbiased.norm();
    if (norm_a < 1e-12) {
        norm_a = 1e-12;
    }
    Vec3 z = a_unbiased / norm_a;

    Vec3 up = math::make_vec3(0.0, 0.0, 1.0);
    double g_norm = g_w.norm();
    if (g_norm > 1e-12) {
        up = -g_w / g_norm;
    }

    // R_bw maps body to world; R_bw.transpose() maps world to body
    Mat3 R_bw = state.q.to_rot_mat();
    h_hat = R_bw.transpose() * up;

    y = z - h_hat;
}

Mat3x15 build_tilt_h_matrix(const Vec3& h_hat) {
    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 0, math::skew(h_hat));
    return H;
}

Mat3 build_tilt_r_matrix(double quality, double r_scale) {
    double sigma_base = params::tilt_sigma;
    double q_clamped = (quality < 0.1) ? 0.1 : quality;
    double sigma_tilt = sigma_base / q_clamped;
    double var_tilt = (sigma_tilt * sigma_tilt) * r_scale;
    return Mat3::identity() * var_tilt;
}

bool apply_tilt_update(NominalState& state,
                       Mat15& P,
                       const Vec3& acc,
                       const Vec3& omega,
                       double quality,
                       const Vec3& g_w,
                       double gate_threshold,
                       double r_scale) {
    double g_mag = g_w.norm();
    if (g_mag < 1e-12) {
        g_mag = 9.80665;
    }

    if (!tilt_gating(acc, omega, g_mag, quality)) {
        return false;
    }

    Vec3 y, h_hat;
    compute_tilt_innovation(state, acc, g_w, y, h_hat);
    Mat3x15 H = build_tilt_h_matrix(h_hat);
    Mat3 R = build_tilt_r_matrix(quality, r_scale);

    return joseph_update(state, P, H, y, R, gate_threshold);
}

bool apply_tilt_update(NominalState& state,
                       Mat15& P,
                       const Vec3& acc,
                       double quality,
                       const Vec3& g_w,
                       double gate_threshold) {
    Vec3 zero_omega = math::make_vec3(0.0, 0.0, 0.0);
    return apply_tilt_update(state, P, acc, zero_omega, quality, g_w, gate_threshold, 1.0);
}

// ---------------------------------------------------------------------------
// Butterworth2LowPass implementation
// ---------------------------------------------------------------------------

Butterworth2LowPass::Butterworth2LowPass()
    : d1(math::make_vec3(0.0, 0.0, 0.0)),
      d2(math::make_vec3(0.0, 0.0, 0.0)),
      initialized(false) {}

void Butterworth2LowPass::reset(const Vec3& x0) {
    d1 = x0 * (b1 + b2 - a1 - a2);
    d2 = x0 * (b2 - a2);
    initialized = true;
}

Vec3 Butterworth2LowPass::step(const Vec3& x) {
    if (!initialized) {
        reset(x);
        return x;
    }
    Vec3 y = x * b0 + d1;
    d1 = x * b1 - y * a1 + d2;
    d2 = x * b2 - y * a2;
    return y;
}

// ---------------------------------------------------------------------------
// StationaryBuffer implementation
// ---------------------------------------------------------------------------

StationaryBuffer::StationaryBuffer()
    : timestamps_{}, gyro_{}, acc_{}, head_(0), count_(0), acc_filter_() {}

void StationaryBuffer::clear() {
    head_ = 0;
    count_ = 0;
    acc_filter_.initialized = false;
}

void StationaryBuffer::add_sample(double t, const Vec3& omega, const Vec3& acc) {
    timestamps_[head_] = t;
    gyro_[head_] = omega;
    acc_[head_] = acc;

    head_ = (head_ + 1) % CAPACITY;
    if (count_ < CAPACITY) {
        count_++;
    }
}

bool StationaryBuffer::is_stationary(double g_mag,
                                     double quality,
                                     double window_s) const {
    if (count_ < 2) {
        return false;
    }

    if (quality < params::zupt_min_quality) {
        return false;
    }

    size_t latest_idx = (head_ - 1 + CAPACITY) % CAPACITY;
    double latest_t = timestamps_[latest_idx];

    size_t oldest_idx = (head_ - count_ + CAPACITY) % CAPACITY;
    double oldest_t = timestamps_[oldest_idx];

    // Check accumulated history duration >= 90% of window
    if ((latest_t - oldest_t) < (window_s * 0.9)) {
        return false;
    }

    double t_thresh = latest_t - window_s;
    size_t k = 0;
    double sum_gyro_norm = 0.0;
    double sum_acc_norm = 0.0;

    for (size_t i = 0; i < count_; ++i) {
        size_t idx = (head_ - 1 - i + CAPACITY) % CAPACITY;
        if (timestamps_[idx] < t_thresh) {
            break;
        }
        sum_gyro_norm += gyro_[idx].norm();
        sum_acc_norm += acc_[idx].norm();
        k++;
    }

    if (k < 2) {
        return false;
    }

    double mean_gyro_norm = sum_gyro_norm / static_cast<double>(k);
    if (mean_gyro_norm >= params::zupt_gyro_mean) {
        return false;
    }

    double mean_acc_norm = sum_acc_norm / static_cast<double>(k);
    if (std::abs(mean_acc_norm - g_mag) >= params::zupt_acc_dev) {
        return false;
    }

    double sum_var = 0.0;
    for (size_t i = 0; i < k; ++i) {
        size_t idx = (head_ - 1 - i + CAPACITY) % CAPACITY;
        double diff = acc_[idx].norm() - mean_acc_norm;
        sum_var += diff * diff;
    }

    double acc_var = sum_var / static_cast<double>(k);
    double var_thresh = params::zupt_acc_dev * params::zupt_acc_dev;
    if (acc_var >= var_thresh) {
        return false;
    }

    return true;
}

Vec3 StationaryBuffer::get_mean_gyro(double window_s) const {
    if (count_ == 0) {
        return math::make_vec3(0.0, 0.0, 0.0);
    }

    size_t latest_idx = (head_ - 1 + CAPACITY) % CAPACITY;
    double latest_t = timestamps_[latest_idx];
    double t_thresh = latest_t - window_s;

    Vec3 sum = math::make_vec3(0.0, 0.0, 0.0);
    size_t k = 0;

    for (size_t i = 0; i < count_; ++i) {
        size_t idx = (head_ - 1 - i + CAPACITY) % CAPACITY;
        if (timestamps_[idx] < t_thresh) {
            break;
        }
        sum += gyro_[idx];
        k++;
    }

    if (k == 0) {
        return math::make_vec3(0.0, 0.0, 0.0);
    }

    return sum / static_cast<double>(k);
}

// ---------------------------------------------------------------------------
// ZUPT measurement update implementation
// ---------------------------------------------------------------------------

Mat3x15 build_zupt_h_matrix() {
    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 3, Mat3::identity());
    return H;
}

Mat3 build_zupt_r_matrix(double r_scale) {
    double var_z = (params::zupt_sigma * params::zupt_sigma) * r_scale;
    return Mat3::identity() * var_z;
}

bool apply_zupt_update(NominalState& state,
                       Mat15& P,
                       double gate_threshold,
                       double r_scale) {
    Vec3 y = -state.v;
    Mat3x15 H = build_zupt_h_matrix();
    Mat3 R = build_zupt_r_matrix(r_scale);

    return joseph_update(state, P, H, y, R, gate_threshold);
}

bool apply_velocity_bounding(NominalState& state,
                             Mat15& P,
                             double max_speed,
                             double sigma_v) {
    if (!params::enable_velocity_bounding) {
        return false;
    }
    double v_mag = state.v.norm();
    if (v_mag <= max_speed || v_mag < 1e-12) {
        return false;
    }

    Vec3 target_v = state.v * (max_speed / v_mag);
    Vec3 y = target_v - state.v;

    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 3, Mat3::identity());

    double var_v = sigma_v * sigma_v;
    Mat3 R = Mat3::identity() * var_v;

    return joseph_update(state, P, H, y, R, 1e9);
}
