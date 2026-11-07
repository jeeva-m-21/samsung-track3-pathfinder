#include "init.h"
#include <cmath>

using namespace math;

Quat compute_initial_tilt(const Vec3& a_mean) {
    double norm_a = a_mean.norm();
    if (norm_a < 1e-8) {
        return Quat{1.0, 0.0, 0.0, 0.0};
    }

    Vec3 v = a_mean / norm_a;
    double cos_theta = v.data[2]; // dot(v, [0, 0, 1])

    if (cos_theta > 0.999999999) {
        return Quat{1.0, 0.0, 0.0, 0.0};
    }
    if (cos_theta < -0.999999999) {
        // 180 degree rotation about x: R * [0, 0, -1] = [0, 0, 1]
        return Quat{0.0, 1.0, 0.0, 0.0};
    }

    // Cross product v x u with target u = [0, 0, 1]:
    // (v x u)_x = v[1]
    // (v x u)_y = -v[0]
    // (v x u)_z = 0.0 (ensures zero yaw)
    Quat q_raw{1.0 + cos_theta, v.data[1], -v.data[0], 0.0};
    Quat q = normalize(q_raw);

    // Canonical positive scalar part
    if (q.w < 0.0) {
        q = -q;
    }
    return q;
}

bool detect_motion(const Vec3* omega_buf, const Vec3* acc_buf, size_t count) {
    if (count < 2) {
        return false;
    }

    double gyro_thresh = params::motion_detect_gyro_std;
    double acc_thresh = params::motion_detect_acc_std;

    // Means
    Vec3 w_mean = make_vec3(0.0, 0.0, 0.0);
    double w_norm_mean = 0.0;
    double a_norm_mean = 0.0;

    for (size_t i = 0; i < count; ++i) {
        w_mean += omega_buf[i];
        w_norm_mean += omega_buf[i].norm();
        a_norm_mean += acc_buf[i].norm();
    }

    double inv_n = 1.0 / static_cast<double>(count);
    w_mean *= inv_n;
    w_norm_mean *= inv_n;
    a_norm_mean *= inv_n;

    // Variances (ddof=0 matching numpy std)
    Vec3 w_var = make_vec3(0.0, 0.0, 0.0);
    double w_norm_var = 0.0;
    double a_norm_var = 0.0;

    for (size_t i = 0; i < count; ++i) {
        Vec3 dw = omega_buf[i] - w_mean;
        w_var.data[0] += dw.data[0] * dw.data[0];
        w_var.data[1] += dw.data[1] * dw.data[1];
        w_var.data[2] += dw.data[2] * dw.data[2];

        double d_wn = omega_buf[i].norm() - w_norm_mean;
        w_norm_var += d_wn * d_wn;

        double d_an = acc_buf[i].norm() - a_norm_mean;
        a_norm_var += d_an * d_an;
    }

    w_var *= inv_n;
    w_norm_var *= inv_n;
    a_norm_var *= inv_n;

    double gyro_std_x = std::sqrt(w_var.data[0]);
    double gyro_std_y = std::sqrt(w_var.data[1]);
    double gyro_std_z = std::sqrt(w_var.data[2]);
    double gyro_std_norm = std::sqrt(w_norm_var);
    double acc_norm_std = std::sqrt(a_norm_var);

    return (gyro_std_x > gyro_thresh ||
            gyro_std_y > gyro_thresh ||
            gyro_std_z > gyro_thresh ||
            gyro_std_norm > gyro_thresh ||
            acc_norm_std > acc_thresh);
}

Mat15 initialize_covariance(bool motion_detected) {
    Mat15 P0 = Mat15::zeros();

    double sigma_theta = params::sigma_theta_0;
    if (motion_detected) {
        sigma_theta *= params::motion_detect_sigma_theta_mult;
    }

    double sigma_v = params::sigma_v_0;
    double sigma_p = params::sigma_p_0;
    double sigma_bg = params::sigma_bg_0;
    double sigma_ba = params::sigma_ba_0;

    double q_theta = sigma_theta * sigma_theta;
    double q_v = sigma_v * sigma_v;
    double q_p = sigma_p * sigma_p;
    double q_bg = sigma_bg * sigma_bg;
    double q_ba = sigma_ba * sigma_ba;

    for (size_t i = 0; i < 3; ++i) {
        P0(i, i) = q_theta;
        P0(3 + i, 3 + i) = q_v;
        P0(6 + i, 6 + i) = q_p;
        P0(9 + i, 9 + i) = q_bg;
        P0(12 + i, 12 + i) = q_ba;
    }

    return P0;
}

Initializer::Initializer()
    : count_(0),
      is_ready_(false),
      motion_detected_(false),
      g_mag_(9.80665) {
    reset();
}

void Initializer::reset() {
    count_ = 0;
    is_ready_ = false;
    motion_detected_ = false;
    g_mag_ = 9.80665;
    for (size_t i = 0; i < MAX_SAMPLES; ++i) {
        t_buf_[i] = 0;
        w_buf_[i] = make_vec3(0.0, 0.0, 0.0);
        a_buf_[i] = make_vec3(0.0, 0.0, 0.0);
    }
}

bool Initializer::add_sample(int64_t t_ns, const Vec3& w, const Vec3& a) {
    if (is_ready_) {
        return true;
    }

    if (count_ < MAX_SAMPLES) {
        t_buf_[count_] = t_ns;
        w_buf_[count_] = w;
        a_buf_[count_] = a;
        count_++;
    }

    if (count_ >= 2) {
        double dt_span_s = static_cast<double>(t_buf_[count_ - 1] - t_buf_[0]) * 1e-9;
        if (dt_span_s >= params::T_init) {
            is_ready_ = true;
            return true;
        }
    }

    return false;
}

bool Initializer::is_ready() const {
    return is_ready_;
}

void Initializer::finish(NominalState& state, Mat15& P0, Vec3& g_w) const {
    if (count_ == 0) {
        state = NominalState::zeros();
        P0 = initialize_covariance(false);
        g_mag_ = 9.80665;
        g_w = make_vec3(0.0, 0.0, -g_mag_);
        motion_detected_ = false;
        return;
    }

    if (count_ >= 2) {
        motion_detected_ = detect_motion(w_buf_, a_buf_, count_);
    } else {
        motion_detected_ = false;
    }

    // Window sample accumulation
    Vec3 w_sum = make_vec3(0.0, 0.0, 0.0);
    Vec3 a_sum = make_vec3(0.0, 0.0, 0.0);
    for (size_t i = 0; i < count_; ++i) {
        w_sum += w_buf_[i];
        a_sum += a_buf_[i];
    }
    double inv_n = 1.0 / static_cast<double>(count_);
    Vec3 w_mean = w_sum * inv_n;
    Vec3 a_mean = a_sum * inv_n;

    Vec3 a_ref;
    if (motion_detected_) {
        // Dynamic motion fallback: use first sample's tilt and zero bias
        a_ref = a_buf_[0];
        state.b_g = make_vec3(0.0, 0.0, 0.0);
        state.b_a = make_vec3(0.0, 0.0, 0.0);
    } else {
        // Static condition: mean gyro as bias and mean accel for tilt
        a_ref = a_mean;
        state.b_g = w_mean;
        state.b_a = make_vec3(0.0, 0.0, 0.0);
    }

    // Zero initial position and velocity
    state.p = make_vec3(0.0, 0.0, 0.0);
    state.v = make_vec3(0.0, 0.0, 0.0);

    // Initial tilt quaternion
    state.q = compute_initial_tilt(a_ref);

    // Gravity magnitude clamping and world gravity vector
    double norm_a = a_ref.norm();
    double g_mag = norm_a;
    if (g_mag < params::g_mag_clamp_min) {
        g_mag = params::g_mag_clamp_min;
    } else if (g_mag > params::g_mag_clamp_max) {
        g_mag = params::g_mag_clamp_max;
    }
    g_mag_ = g_mag;
    g_w = make_vec3(0.0, 0.0, -g_mag);

    // Initial covariance
    P0 = initialize_covariance(motion_detected_);
}
