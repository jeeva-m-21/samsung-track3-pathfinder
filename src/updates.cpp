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
    compute_tilt_innovation(state, acc, math::make_vec3(0.0, 0.0, 0.0), g_w, y, h_hat);
}

void compute_tilt_innovation(const NominalState& state,
                             const Vec3& acc,
                             const Vec3& omega,
                             const Vec3& g_w,
                             Vec3& y,
                             Vec3& h_hat,
                             double comp_gain) {
    Mat3 R_bw = state.q.to_rot_mat();
    Vec3 a_eff = acc - state.b_a;
    if (params::enable_centripetal_comp && comp_gain > 0.0) {
        Vec3 v_body = R_bw.transpose() * state.v;
        Vec3 a_centripetal = math::cross(omega, v_body) * comp_gain;
        double a_c_norm = a_centripetal.norm();
        if (a_c_norm > 15.0) {
            a_centripetal *= (15.0 / a_c_norm);
        }
        a_eff = a_eff - a_centripetal;
    }

    double norm_a = a_eff.norm();
    if (norm_a < 1e-12) {
        norm_a = 1e-12;
    }
    Vec3 z = a_eff / norm_a;

    Vec3 up = math::make_vec3(0.0, 0.0, 1.0);
    double g_norm = g_w.norm();
    if (g_norm > 1e-12) {
        up = -g_w / g_norm;
    }

    h_hat = R_bw.transpose() * up;
    y = z - h_hat;
}

void compute_tilt_innovation(const NominalState& state,
                             const Vec3& acc,
                             const Vec3& omega,
                             const Vec3& g_w,
                             Vec3& y,
                             Vec3& h_hat) {
    compute_tilt_innovation(state, acc, omega, g_w, y, h_hat, 0.2);
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

    double q_clamped = (quality < 0.0) ? 0.0 : ((quality > 1.0) ? 1.0 : quality);
    double comp_gain = 0.2 * q_clamped * q_clamped;
    Vec3 a_gating = acc;
    if (params::enable_centripetal_comp && comp_gain > 0.0) {
        Mat3 R_bw = state.q.to_rot_mat();
        Vec3 v_body = R_bw.transpose() * state.v;
        Vec3 a_centripetal = math::cross(omega, v_body) * comp_gain;
        double a_c_norm = a_centripetal.norm();
        if (a_c_norm > 15.0) {
            a_centripetal *= (15.0 / a_c_norm);
        }
        a_gating = acc - a_centripetal;
    }

    if (!tilt_gating(a_gating, omega, g_mag, quality)) {
        return false;
    }

    Vec3 y, h_hat;
    compute_tilt_innovation(state, acc, omega, g_w, y, h_hat, comp_gain);
    Mat3x15 H = build_tilt_h_matrix(h_hat);

    // Dynamic noise scaling based on linear acceleration residual and gyro rate
    double acc_norm = a_gating.norm();
    double delta_a = std::abs(acc_norm - g_mag);
    double a_scale = (params::tilt_a_scale > 1e-6) ? params::tilt_a_scale : params::tilt_tau_a;
    double a_ratio = delta_a / a_scale;

    double omega_norm = omega.norm();
    double omega_scale = (params::tilt_omega_scale > 1e-6) ? params::tilt_omega_scale : params::tilt_omega_max;
    double omega_ratio = omega_norm / omega_scale;

    double dynamic_scale = 1.0 + params::tilt_alpha_a * (a_ratio * a_ratio)
                               + params::tilt_alpha_omega * (omega_ratio * omega_ratio);
    if (dynamic_scale > 1e6) {
        dynamic_scale = 1e6;
    }
    Mat3 R = build_tilt_r_matrix(quality, r_scale * dynamic_scale);

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

void StationaryBuffer::add_sample(double t, const Vec3& omega, const Vec3& acc, const Vec3& b_g) {
    timestamps_[head_] = t;
    gyro_[head_] = omega - b_g;
    acc_[head_] = acc_filter_.step(acc);

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
    double zupt_gyro_limit = (params::zupt_omega_thresh > 0.0) ? params::zupt_omega_thresh : params::zupt_gyro_mean;
    if (mean_gyro_norm >= zupt_gyro_limit) {
        return false;
    }

    if (gyro_[latest_idx].norm() > zupt_gyro_limit * 1.5) {
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

bool apply_zaru_update(NominalState& state,
                       Mat15& P,
                       const Vec3& omega_curr,
                       double r_scale) {
    Vec3 y = omega_curr - state.b_g;
    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 9, Mat3::identity());
    double sigma_zaru = 0.005;
    double var_z = (sigma_zaru * sigma_zaru) * r_scale;
    Mat3 R = Mat3::identity() * var_z;

    return joseph_update(state, P, H, y, R, 1e9);
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

    // Decouple P_pv and P_vp cross-covariance blocks so velocity bounding
    // pseudo-measurement does not inject unphysical position jumps/kicks
    P.set_block<3, 3>(6, 3, Mat3::zeros());
    P.set_block<3, 3>(3, 6, Mat3::zeros());

    Vec3 target_v = state.v * (max_speed / v_mag);
    Vec3 y = target_v - state.v;

    Mat3x15 H = Mat3x15::zeros();
    H.set_block<3, 3>(0, 3, Mat3::identity());

    double var_v = sigma_v * sigma_v;
    Mat3 R = Mat3::identity() * var_v;

    return joseph_update(state, P, H, y, R, 1e9);
}

bool apply_rotor_drag_update(NominalState& state,
                             Mat15& P,
                             const Vec3& acc,
                             const Vec3& omega,
                             const Vec3& /*g_w*/,
                             double kd,
                             double sigma_drag,
                             double gate_threshold) {
    if (!params::enable_rotor_drag || kd <= 0.0) {
        return false;
    }

    if (omega.norm() > 2.5) {
        return false;
    }

    Vec3 a_b = acc - state.b_a;
    double a_norm = a_b.norm();
    if (a_norm < 3.0 || a_norm > 35.0) {
        return false;
    }

    Mat3 R = state.q.to_rot_mat();
    Mat3 R_T = R.transpose();
    Vec3 v_b = R_T * state.v;

    // Physical rotor drag in body horizontal axes:
    // a_xy^b ≈ - kd * v_xy^b  ==>  y = a_xy^b + kd * v_xy^b
    Vec3 y = math::make_vec3(a_b(0) + kd * v_b(0),
                             a_b(1) + kd * v_b(1),
                             0.0);

    // Decouple P_pv and P_vp cross-covariance blocks so velocity pseudo-measurement
    // does not inject unphysical position jumps/kicks
    P.set_block<3, 3>(6, 3, Mat3::zeros());
    P.set_block<3, 3>(3, 6, Mat3::zeros());

    Mat3x15 H = Mat3x15::zeros();

    // H_v: dy / d(delta_v) = - kd * (R^T)_{0:2, :}
    // Rotor drag directly constrains velocity without corrupting gravity-anchored attitude
    Mat3 H_v = Mat3::zeros();
    for (size_t r = 0; r < 2; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            H_v(r, c) = -kd * R_T(r, c);
        }
    }
    H.set_block<3, 3>(0, 3, H_v);

    double var_drag = sigma_drag * sigma_drag;
    Mat3 R_cov = Mat3::identity() * var_drag;
    R_cov(2, 2) = 1e8;

    return joseph_update(state, P, H, y, R_cov, gate_threshold);
}

bool apply_pedestrian_prior(NominalState& state,
                            Mat15& P,
                            double sigma_lat,
                            double sigma_vert,
                            double gate_threshold) {
    if (!params::enable_pedestrian_prior) {
        return false;
    }

    // Decouple P_pv and P_vp cross-covariance blocks
    P.set_block<3, 3>(6, 3, Mat3::zeros());
    P.set_block<3, 3>(3, 6, Mat3::zeros());

    Mat3 R = state.q.to_rot_mat();
    Mat3 R_T = R.transpose();
    Vec3 v_b = R_T * state.v;

    // Pedestrian motion prior:
    // 1. Lateral body velocity is near zero (v_y^b ≈ 0)
    // 2. World vertical velocity is near zero (v_z^w ≈ 0)
    Vec3 y = math::make_vec3(-v_b(1), -state.v(2), 0.0);

    Mat3x15 H = Mat3x15::zeros();

    Mat3 H_v = Mat3::zeros();
    for (size_t c = 0; c < 3; ++c) {
        H_v(0, c) = R_T(1, c);
    }
    // Row 1: world vertical velocity
    H_v(1, 2) = 1.0;
    H.set_block<3, 3>(0, 3, H_v);

    Mat3 R_cov = Mat3::identity();
    R_cov(0, 0) = sigma_lat > 0.0 ? (sigma_lat * sigma_lat) : 1e8;
    R_cov(1, 1) = sigma_vert * sigma_vert;
    R_cov(2, 2) = 1e8;

    return joseph_update(state, P, H, y, R_cov, gate_threshold);
}
