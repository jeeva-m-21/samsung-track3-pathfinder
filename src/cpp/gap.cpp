#include "gap.h"
#include <algorithm>
#include <cmath>

SampleValidity classify_sample(const Vec3& omega,
                               const Vec3& acc,
                               const Vec3* prev_omega,
                               const Vec3* prev_acc,
                               int repeat_count) {
    // 1. Check for NaN or Inf
    for (size_t i = 0; i < 3; ++i) {
        if (std::isnan(omega.data[i]) || std::isinf(omega.data[i]) ||
            std::isnan(acc.data[i]) || std::isinf(acc.data[i])) {
            return SampleValidity::NAN_OR_INF;
        }
    }

    // 2. Check for all six zeros
    if (omega.data[0] == 0.0 && omega.data[1] == 0.0 && omega.data[2] == 0.0 &&
        acc.data[0] == 0.0 && acc.data[1] == 0.0 && acc.data[2] == 0.0) {
        return SampleValidity::ALL_ZERO;
    }

    // 3. Out-of-range bounds
    double gyro_max = params::invalid_gyro_max;
    double acc_max = params::invalid_acc_max;

    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(omega.data[i]) > gyro_max || std::abs(acc.data[i]) > acc_max) {
            return SampleValidity::OUT_OF_RANGE;
        }
    }
    if (omega.norm() > gyro_max || acc.norm() > acc_max) {
        return SampleValidity::OUT_OF_RANGE;
    }

    // 4. Stuck sensor (>= 3 consecutive identical rows)
    if (prev_omega != nullptr && prev_acc != nullptr) {
        bool identical = (omega.data[0] == prev_omega->data[0] &&
                          omega.data[1] == prev_omega->data[1] &&
                          omega.data[2] == prev_omega->data[2] &&
                          acc.data[0] == prev_acc->data[0] &&
                          acc.data[1] == prev_acc->data[1] &&
                          acc.data[2] == prev_acc->data[2]);
        if (identical && (repeat_count + 1 >= 3)) {
            return SampleValidity::STUCK;
        }
    }

    return SampleValidity::VALID;
}

double compute_dt_nom(const double* dt_samples, size_t count, double default_dt) {
    if (count == 0 || dt_samples == nullptr) {
        return default_dt;
    }

    double valid_buf[200];
    size_t valid_count = 0;

    for (size_t i = 0; i < count && valid_count < 200; ++i) {
        double val = dt_samples[i];
        if (val > 0.0 && std::isfinite(val)) {
            valid_buf[valid_count++] = val;
        }
    }

    if (valid_count == 0) {
        return default_dt;
    }

    // In-place sort on the stack buffer
    std::sort(valid_buf, valid_buf + valid_count);

    if (valid_count % 2 == 1) {
        return valid_buf[valid_count / 2];
    } else {
        return 0.5 * (valid_buf[valid_count / 2 - 1] + valid_buf[valid_count / 2]);
    }
}

Vec3 update_smoothed_gyro(const Vec3& omega_s,
                          const Vec3& omega,
                          const Vec3& b_g,
                          double dt,
                          double tau) {
    if (dt <= 0.0) {
        return omega_s;
    }
    Vec3 omega_corr = omega - b_g;
    double alpha = 1.0 - std::exp(-dt / tau);
    return (1.0 - alpha) * omega_s + alpha * omega_corr;
}

double get_recovery_multiplier(int64_t recover, int64_t recover_steps) {
    if (recover <= 0) {
        return 1.0;
    }
    double steps = (recover_steps < 1) ? 1.0 : static_cast<double>(recover_steps);
    return 1.0 + 10.0 * (static_cast<double>(recover) / steps);
}

void gap_substep(NominalState& state,
                 Mat15& P,
                 const Vec3& omega_s,
                 double dt_sub) {
    double decay = std::exp(-dt_sub / params::tau_gap);
    Vec3 v_next = state.v * decay;
    Vec3 p_next = state.p + 0.5 * (state.v + v_next) * dt_sub;

    Quat delta_q = math::exp_map(omega_s * dt_sub);
    Quat q_rot = state.q * delta_q;
    Quat q_next = q_rot.normalized();
    math::ensure_continuity(q_next, state.q);

    state.q = q_next;
    state.v = v_next;
    state.p = p_next;

    Mat15 F = build_f_matrix(state, omega_s + state.b_g, state.b_a, dt_sub);
    Mat15 Q = build_q_matrix(dt_sub,
                             params::noise_mult_gyro,
                             params::noise_mult_acc,
                             params::walk_mult_bg,
                             params::walk_mult_ba,
                             1.0) * params::kappa_gap;

    P = propagate_covariance(P, F, Q);
}

int propagate_gap(NominalState& state,
                  Mat15& P,
                  const Vec3& omega_s,
                  double dt,
                  double dt_nom) {
    double nom_dt = (dt_nom < 1e-6) ? 1e-6 : dt_nom;
    int n = static_cast<int>(std::ceil((dt / nom_dt) - 1e-9));
    if (n < 1) n = 1;
    if (n > params::gap_substep_max) n = static_cast<int>(params::gap_substep_max);

    double dt_sub = dt / static_cast<double>(n);
    for (int i = 0; i < n; ++i) {
        gap_substep(state, P, omega_s, dt_sub);
    }
    return n;
}

// ---------------------------------------------------------------------------
// GapHandler implementation
// ---------------------------------------------------------------------------

GapHandler::GapHandler() {
    reset();
}

void GapHandler::reset() {
    recover_ = 0;
    dt_nom_ = 0.005;
    dt_nom_frozen_ = false;
    dt_sample_count_ = 0;
    omega_s_ = math::make_vec3(0.0, 0.0, 0.0);
    prev_t_ns_ = 0;
    has_prev_sample_ = false;
    prev_omega_ = math::make_vec3(0.0, 0.0, 0.0);
    prev_acc_ = math::make_vec3(0.0, 0.0, 0.0);
    repeat_count_ = 0;
}

GapStepResult GapHandler::process_sample(int64_t t_ns,
                                         const Vec3& omega,
                                         const Vec3& acc,
                                         const NominalState& state,
                                         const Mat15& P) {
    // 1. First sample
    if (!has_prev_sample_) {
        SampleValidity validity = classify_sample(omega, acc);
        bool is_valid = is_valid_sample(validity);
        prev_t_ns_ = t_ns;
        prev_omega_ = omega;
        prev_acc_ = acc;
        has_prev_sample_ = true;
        repeat_count_ = 1;

        if (is_valid) {
            omega_s_ = omega - state.b_g;
        } else {
            omega_s_ = math::make_vec3(0.0, 0.0, 0.0);
        }

        return GapStepResult{
            state,
            P,
            is_valid ? GapCase::NORMAL : GapCase::INVALID_SAMPLE,
            1,
            is_valid,
            1.0,
            is_valid,
            validity
        };
    }

    // 2. Check dt
    int64_t dt_ns = t_ns - prev_t_ns_;
    double dt = static_cast<double>(dt_ns) * 1e-9;

    if (dt <= 0.0) {
        double r_scale = get_recovery_multiplier(recover_, params::recover_steps);
        return GapStepResult{
            state,
            P,
            GapCase::ZERO_OR_NEGATIVE_DT,
            0,
            false,
            r_scale,
            false,
            SampleValidity::VALID
        };
    }

    // 3. Classify sample validity
    SampleValidity validity = classify_sample(omega, acc, &prev_omega_, &prev_acc_, repeat_count_);
    bool is_valid = is_valid_sample(validity);

    bool identical = (omega.data[0] == prev_omega_.data[0] &&
                      omega.data[1] == prev_omega_.data[1] &&
                      omega.data[2] == prev_omega_.data[2] &&
                      acc.data[0] == prev_acc_.data[0] &&
                      acc.data[1] == prev_acc_.data[1] &&
                      acc.data[2] == prev_acc_.data[2]);
    if (identical) {
        repeat_count_++;
    } else {
        repeat_count_ = 1;
        prev_omega_ = omega;
        prev_acc_ = acc;
    }

    // 4. Invalid sample handling
    if (!is_valid) {
        recover_ = params::recover_steps;
        double r_scale = get_recovery_multiplier(recover_, params::recover_steps);

        NominalState next_state = state;
        Mat15 next_P = P;
        gap_substep(next_state, next_P, omega_s_, dt_nom_);
        prev_t_ns_ = t_ns;

        return GapStepResult{
            next_state,
            next_P,
            GapCase::INVALID_SAMPLE,
            1,
            false,
            r_scale,
            false,
            validity
        };
    }

    // 5. Valid sample: calibrate dt_nom if not frozen
    if (!dt_nom_frozen_) {
        if (dt_sample_count_ < DT_CALIB_MAX) {
            dt_samples_[dt_sample_count_++] = dt;
            dt_nom_ = compute_dt_nom(dt_samples_, dt_sample_count_);
            if (dt_sample_count_ >= DT_CALIB_MAX) {
                dt_nom_frozen_ = true;
            }
        }
    }

    omega_s_ = update_smoothed_gyro(omega_s_, omega, state.b_g, dt, params::smooth_omega_tau);

    // 6. Check for gap (dt > 2.5 * dt_nom)
    if (dt > 2.5 * dt_nom_) {
        NominalState next_state = state;
        Mat15 next_P = P;
        int n_steps = propagate_gap(next_state, next_P, omega_s_, dt, dt_nom_);

        recover_ = params::recover_steps;
        double r_scale = get_recovery_multiplier(recover_, params::recover_steps);
        prev_t_ns_ = t_ns;

        return GapStepResult{
            next_state,
            next_P,
            GapCase::GAP,
            n_steps,
            false,
            r_scale,
            true,
            SampleValidity::VALID
        };
    }

    // 7. Normal propagation
    if (recover_ > 0) {
        recover_--;
    }

    double r_scale = get_recovery_multiplier(recover_, params::recover_steps);
    prev_t_ns_ = t_ns;

    return GapStepResult{
        state,
        P,
        GapCase::NORMAL,
        1,
        true,
        r_scale,
        true,
        SampleValidity::VALID
    };
}
