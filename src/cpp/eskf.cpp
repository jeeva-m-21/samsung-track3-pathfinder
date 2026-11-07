#include "eskf.h"
#include <cmath>

using namespace math;

NominalState strapdown_step(const NominalState& state,
                            const Vec3& omega,
                            const Vec3& acc,
                            double dt,
                            const Vec3& g_w) {
    if (dt <= 0.0) {
        return state;
    }

    // 1. Unbiased angular velocity and specific force (SPEC §12.3)
    Vec3 omega_hat = omega - state.b_g;
    Vec3 a_hat = acc - state.b_a;

    // 2. Propagate orientation quaternion
    Quat delta_q = exp_map(omega_hat * dt);
    Quat q_new = normalize(state.q * delta_q);
    ensure_continuity(q_new, state.q);

    // 3. Accelerometer transformation using q_old
    Mat3 R_old = to_rot_mat(state.q);
    Vec3 a_w = R_old * a_hat + g_w;

    // 4. Integrate position and velocity
    double dt_sq_half = 0.5 * (dt * dt);
    Vec3 p_new = state.p + state.v * dt + a_w * dt_sq_half;
    Vec3 v_new = state.v + a_w * dt;

    // 5. Biases unchanged
    NominalState new_state;
    new_state.p = p_new;
    new_state.v = v_new;
    new_state.q = q_new;
    new_state.b_g = state.b_g;
    new_state.b_a = state.b_a;

    return new_state;
}

Mat15 build_f_matrix(const NominalState& state,
                     const Vec3& omega,
                     const Vec3& acc,
                     double dt) {
    Mat15 F = Mat15::identity();
    if (dt <= 0.0) {
        return F;
    }

    Vec3 omega_hat = omega - state.b_g;
    Vec3 a_hat = acc - state.b_a;

    // Orientation error block: R(omega_hat * dt)^T
    Quat delta_q = exp_map(omega_hat * dt);
    Mat3 R_delta = to_rot_mat(delta_q);
    F.set_block<3, 3>(0, 0, R_delta.transpose());

    // Gyro bias effect on orientation: -I * dt
    for (size_t i = 0; i < 3; ++i) {
        F(i, 9 + i) = -dt;
    }

    // Velocity error blocks
    Mat3 R_old = to_rot_mat(state.q);
    Mat3 S_a = skew(a_hat);
    Mat3 block_v_th = (R_old * S_a) * (-dt);
    F.set_block<3, 3>(3, 0, block_v_th);

    Mat3 block_v_ba = R_old * (-dt);
    F.set_block<3, 3>(3, 12, block_v_ba);

    // Position error block: I * dt
    for (size_t i = 0; i < 3; ++i) {
        F(6 + i, 3 + i) = dt;
    }

    return F;
}

Mat15 build_q_matrix(double dt,
                     double noise_mult_gyro,
                     double noise_mult_acc,
                     double walk_mult_bg,
                     double walk_mult_ba,
                     double kappa) {
    Mat15 Q = Mat15::zeros();
    if (dt <= 0.0) {
        return Q;
    }

    double sigma_g = params::sigma_g_base * noise_mult_gyro * kappa;
    double sigma_a = params::sigma_a_base * noise_mult_acc * kappa;
    double sigma_bg = params::sigma_bg_base * walk_mult_bg * kappa;
    double sigma_ba = params::sigma_ba_base * walk_mult_ba * kappa;

    double q_theta = (sigma_g * sigma_g) * dt;
    double q_v = (sigma_a * sigma_a) * dt;
    double q_bg = (sigma_bg * sigma_bg) * dt;
    double q_ba = (sigma_ba * sigma_ba) * dt;

    for (size_t i = 0; i < 3; ++i) {
        Q(i, i) = q_theta;
        Q(3 + i, 3 + i) = q_v;
        // Position block (6..8) remains 0
        Q(9 + i, 9 + i) = q_bg;
        Q(12 + i, 12 + i) = q_ba;
    }

    return Q;
}

void condition_covariance(Mat15& P, double diag_floor, double diag_cap) {
    // 1. Symmetrisation: P = 0.5 * (P + P^T)
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = i + 1; j < 15; ++j) {
            double avg = 0.5 * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }

    // 2. Floor handling
    for (size_t i = 0; i < 15; ++i) {
        if (P(i, i) < diag_floor) {
            P(i, i) = diag_floor;
        }
    }

    // 3. Cap handling with positive-definiteness-preserving congruence transformation
    for (size_t i = 0; i < 15; ++i) {
        if (P(i, i) > diag_cap) {
            double scale = std::sqrt(diag_cap / P(i, i));
            for (size_t j = 0; j < 15; ++j) {
                if (j != i) {
                    P(i, j) *= scale;
                    P(j, i) *= scale;
                }
            }
            P(i, i) = diag_cap;
        }
    }
}

Mat15 propagate_covariance(const Mat15& P, const Mat15& F, const Mat15& Q) {
    Mat15 P_pred = (F * P) * F.transpose() + Q;
    condition_covariance(P_pred, params::cov_diag_floor, params::cov_diag_cap);
    return P_pred;
}

NominalState eskf_inject_state(const NominalState& state, const Vec15& delta_x) {
    Vec3 delta_theta = make_vec3(delta_x(0), delta_x(1), delta_x(2));
    Vec3 delta_v = make_vec3(delta_x(3), delta_x(4), delta_x(5));
    Vec3 delta_p = make_vec3(delta_x(6), delta_x(7), delta_x(8));
    Vec3 delta_bg = make_vec3(delta_x(9), delta_x(10), delta_x(11));
    Vec3 delta_ba = make_vec3(delta_x(12), delta_x(13), delta_x(14));

    // Orientation injection with normalization and sign continuity
    Quat delta_q = exp_map(delta_theta);
    Quat q_new = normalize(state.q * delta_q);
    ensure_continuity(q_new, state.q);

    NominalState new_state;
    new_state.p = state.p + delta_p;
    new_state.v = state.v + delta_v;
    new_state.q = q_new;
    new_state.b_g = state.b_g + delta_bg;
    new_state.b_a = state.b_a + delta_ba;

    return new_state;
}

Mat15 eskf_reset_covariance(const Mat15& P,
                            const Vec3& delta_theta,
                            double diag_floor,
                            double diag_cap) {
    Mat15 G = Mat15::identity();
    Mat3 S_dth = skew(delta_theta * 0.5);
    Mat3 G_th = Mat3::identity() - S_dth;
    G.set_block<3, 3>(0, 0, G_th);

    Mat15 P_reset = (G * P) * G.transpose();
    condition_covariance(P_reset, diag_floor, diag_cap);
    return P_reset;
}

bool joseph_update(NominalState& state,
                   Mat15& P,
                   const Mat3x15& H,
                   const Vec3& y,
                   const Mat3& R,
                   double gate_threshold) {
    // S = H * P * H^T + R
    Mat3x15 HP = H * P;
    Mat<15, 3> H_T = H.transpose();
    Mat3 S = (HP * H_T) + R;

    // Closed-form 3x3 inverse with floor
    Mat3 S_inv = inv3(S, params::cov_diag_floor);

    // Chi-squared innovation gate: y^T * S^{-1} * y <= gate_threshold
    Vec3 S_inv_y = S_inv * y;
    double chi2 = y.dot(S_inv_y);
    if (chi2 > gate_threshold) {
        return false;
    }

    // Kalman gain K = P * H^T * S^{-1}  (15x3)
    Mat15x3 PH_T = P * H_T;
    Mat15x3 K = PH_T * S_inv;

    // Error state correction delta_x = K * y  (15x1)
    Vec15 delta_x = K * y;

    // Joseph-form covariance update: (I - K*H) * P * (I - K*H)^T + K * R * K^T
    Mat15 KH = K * H;
    Mat15 I_KH = Mat15::identity() - KH;
    Mat15 P_post = (I_KH * P) * I_KH.transpose() + (K * R) * K.transpose();
    condition_covariance(P_post, params::cov_diag_floor, params::cov_diag_cap);

    // Error state injection into nominal state
    Vec3 delta_theta = make_vec3(delta_x(0), delta_x(1), delta_x(2));
    state = eskf_inject_state(state, delta_x);

    // Error state covariance reset
    P = eskf_reset_covariance(P_post, delta_theta, params::cov_diag_floor, params::cov_diag_cap);

    return true;
}
