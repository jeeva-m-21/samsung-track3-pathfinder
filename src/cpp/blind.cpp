#include "blind.h"
#include <cmath>

Vec3 predict_blind_position(const Vec3& p_f,
                            const Vec3& v_f,
                            double tau,
                            double tau_v,
                            double shrink_v,
                            double stationary_v_thresh) {
    if (tau <= 0.0) {
        return p_f;
    }

    double t = (tau > 5.0) ? 5.0 : tau;

    if (v_f.norm() < stationary_v_thresh) {
        return p_f;
    }

    double tau_v_val = (tau_v < 1e-6) ? 1e-6 : tau_v;
    double decay_factor = 1.0 - std::exp(-t / tau_v_val);
    Vec3 delta_p = v_f * (shrink_v * tau_v_val * decay_factor);

    return p_f + delta_p;
}

Quat predict_blind_orientation(const Quat& q_f,
                               const Vec3& omega_f,
                               double tau,
                               double tau_omega) {
    if (tau <= 0.0) {
        return q_f.normalized();
    }

    double t = (tau > 5.0) ? 5.0 : tau;

    double tau_w_val = (tau_omega < 1e-6) ? 1e-6 : tau_omega;
    double decay_factor = 1.0 - std::exp(-t / tau_w_val);
    Vec3 rot_vec = omega_f * (tau_w_val * decay_factor);

    Quat delta_q = math::exp_map(rot_vec);
    Quat q_rot = (q_f * delta_q).normalized();
    math::ensure_continuity(q_rot, q_f);

    return q_rot;
}

Vec3 predict_blind_velocity(const Vec3& v_f,
                            double tau,
                            double tau_v,
                            double shrink_v,
                            double stationary_v_thresh) {
    if (tau < 0.0) {
        tau = 0.0;
    }
    if (tau >= 5.0) {
        return math::make_vec3(0.0, 0.0, 0.0);
    }
    if (v_f.norm() < stationary_v_thresh) {
        return math::make_vec3(0.0, 0.0, 0.0);
    }

    double tau_v_val = (tau_v < 1e-6) ? 1e-6 : tau_v;
    return v_f * (shrink_v * std::exp(-tau / tau_v_val));
}

void predict_blind_pose(const Vec3& p_f,
                        const Vec3& v_f,
                        const Quat& q_f,
                        const Vec3& omega_f,
                        double tau,
                        Vec3& out_p,
                        Quat& out_q) {
    out_p = predict_blind_position(p_f, v_f, tau);
    out_q = predict_blind_orientation(q_f, omega_f, tau);
}

// ---------------------------------------------------------------------------
// ReliableRingBuffer implementation
// ---------------------------------------------------------------------------

ReliableRingBuffer::ReliableRingBuffer()
    : t_ns_{}, p_{}, v_{}, q_{}, omega_s_{}, head_(0), count_(0) {}

void ReliableRingBuffer::clear() {
    head_ = 0;
    count_ = 0;
}

void ReliableRingBuffer::push(int64_t t_ns,
                              const Vec3& p,
                              const Vec3& v,
                              const Quat& q,
                              const Vec3& omega_s) {
    t_ns_[head_] = t_ns;
    p_[head_] = p;
    v_[head_] = v;
    q_[head_] = q;
    omega_s_[head_] = omega_s;

    head_ = (head_ + 1) % CAPACITY;
    if (count_ < CAPACITY) {
        count_++;
    }
}

bool ReliableRingBuffer::get_latest(ReliableSnapshot& out) const {
    if (count_ == 0) {
        return false;
    }
    size_t latest_idx = (head_ - 1 + CAPACITY) % CAPACITY;
    return get_at_index(latest_idx, out);
}

bool ReliableRingBuffer::get_oldest(ReliableSnapshot& out) const {
    if (count_ == 0) {
        return false;
    }
    size_t oldest_idx = (head_ - count_ + CAPACITY) % CAPACITY;
    return get_at_index(oldest_idx, out);
}

bool ReliableRingBuffer::get_at_index(size_t ring_idx, ReliableSnapshot& out) const {
    if (ring_idx >= CAPACITY) {
        return false;
    }
    out.t_ns = t_ns_[ring_idx];
    out.p = p_[ring_idx];
    out.v = v_[ring_idx];
    out.q = q_[ring_idx];
    out.omega_s = omega_s_[ring_idx];
    return true;
}

ReliableSnapshot rollback_to_reliable(const ReliableRingBuffer& ring_buffer, int64_t t_onset_ns) {
    ReliableSnapshot fallback{
        t_onset_ns,
        math::make_vec3(0.0, 0.0, 0.0),
        math::make_vec3(0.0, 0.0, 0.0),
        Quat::identity(),
        math::make_vec3(0.0, 0.0, 0.0)
    };

    if (ring_buffer.empty()) {
        return fallback;
    }

    size_t cap = ReliableRingBuffer::CAPACITY;
    size_t head = ring_buffer.head();
    size_t count = ring_buffer.count();

    // Scan backwards from newest to oldest for latest entry with t_ns < t_onset_ns
    for (size_t i = 0; i < count; ++i) {
        size_t idx = (head - 1 - i + cap) % cap;
        if (ring_buffer.t_ns_at(idx) < t_onset_ns) {
            ReliableSnapshot snap;
            if (ring_buffer.get_at_index(idx, snap)) {
                return snap;
            }
        }
    }

    // Fallback to oldest available entry
    ReliableSnapshot oldest;
    if (ring_buffer.get_oldest(oldest)) {
        return oldest;
    }

    return fallback;
}

// ---------------------------------------------------------------------------
// FailureDetector implementation
// ---------------------------------------------------------------------------

FailureDetector::FailureDetector() {
    reset();
}

void FailureDetector::reset() {
    ring_buffer_.clear();
    is_failed_ = false;
    failure_time_ns_ = 0;
    invalid_start_ns_ = -1;
    last_valid_t_ns_ = 0;
    frozen_state_ = ReliableSnapshot{
        0,
        math::make_vec3(0.0, 0.0, 0.0),
        math::make_vec3(0.0, 0.0, 0.0),
        Quat::identity(),
        math::make_vec3(0.0, 0.0, 0.0)
    };
}

void FailureDetector::record_reliable_state(int64_t t_ns,
                                            const Vec3& p,
                                            const Vec3& v,
                                            const Quat& q,
                                            const Vec3& omega_s) {
    ring_buffer_.push(t_ns, p, v, q, omega_s);
    last_valid_t_ns_ = t_ns;
    invalid_start_ns_ = -1;
}

bool FailureDetector::update(int64_t t_ns,
                             bool is_valid,
                             const Vec3& p,
                             const Vec3& v,
                             const Quat& q,
                             const Vec3& omega_s) {
    if (is_failed_) {
        return true;
    }

    if (is_valid) {
        record_reliable_state(t_ns, p, v, q, omega_s);
        invalid_start_ns_ = -1;
        last_valid_t_ns_ = t_ns;
        return false;
    }

    // Invalid sample
    if (invalid_start_ns_ < 0) {
        invalid_start_ns_ = t_ns;
    }

    double invalid_duration_s = static_cast<double>(t_ns - invalid_start_ns_) * 1e-9;
    if (invalid_duration_s >= params::T_fail) {
        declare_failure(t_ns, invalid_start_ns_);
        return true;
    }

    return false;
}

void FailureDetector::declare_failure(int64_t failure_time_ns, int64_t failure_onset_ns) {
    if (is_failed_) {
        return;
    }
    int64_t onset = (failure_onset_ns >= 0) ? failure_onset_ns : failure_time_ns;
    frozen_state_ = rollback_to_reliable(ring_buffer_, onset);
    failure_time_ns_ = failure_time_ns;
    is_failed_ = true;
}

void FailureDetector::notify_stream_end(int64_t final_t_ns) {
    if (is_failed_) {
        return;
    }
    if (!ring_buffer_.empty()) {
        ReliableSnapshot latest;
        ring_buffer_.get_latest(latest);
        int64_t onset = (final_t_ns > 0) ? final_t_ns : latest.t_ns;
        frozen_state_ = rollback_to_reliable(ring_buffer_, onset + 1);
    } else {
        frozen_state_ = ReliableSnapshot{
            final_t_ns,
            math::make_vec3(0.0, 0.0, 0.0),
            math::make_vec3(0.0, 0.0, 0.0),
            Quat::identity(),
            math::make_vec3(0.0, 0.0, 0.0)
        };
    }
    failure_time_ns_ = (final_t_ns > 0) ? final_t_ns : frozen_state_.t_ns;
    is_failed_ = true;
}

void FailureDetector::predict(int64_t t_ns, Vec3& out_p, Quat& out_q) const {
    double tau = static_cast<double>(t_ns - frozen_state_.t_ns) * 1e-9;
    predict_blind_pose(frozen_state_.p,
                       frozen_state_.v,
                       frozen_state_.q,
                       frozen_state_.omega_s,
                       tau,
                       out_p,
                       out_q);
}

void FailureDetector::predict_tau(double tau, Vec3& out_p, Quat& out_q) const {
    predict_blind_pose(frozen_state_.p,
                       frozen_state_.v,
                       frozen_state_.q,
                       frozen_state_.omega_s,
                       tau,
                       out_p,
                       out_q);
}
