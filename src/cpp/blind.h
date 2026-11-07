#ifndef BLIND_H
#define BLIND_H

#include "mat.h"
#include "quat.h"
#include "params.h"
#include <cstdint>
#include <cstddef>

// Pure extrapolation functions (SPEC §12.9)
Vec3 predict_blind_position(const Vec3& p_f,
                            const Vec3& v_f,
                            double tau,
                            double tau_v = params::tau_v,
                            double shrink_v = params::shrink_v,
                            double stationary_v_thresh = params::stationary_v_thresh);

Quat predict_blind_orientation(const Quat& q_f,
                               const Vec3& omega_f,
                               double tau,
                               double tau_omega = params::tau_omega);

Vec3 predict_blind_velocity(const Vec3& v_f,
                            double tau,
                            double tau_v = params::tau_v,
                            double shrink_v = params::shrink_v,
                            double stationary_v_thresh = params::stationary_v_thresh);

void predict_blind_pose(const Vec3& p_f,
                        const Vec3& v_f,
                        const Quat& q_f,
                        const Vec3& omega_f,
                        double tau,
                        Vec3& out_p,
                        Quat& out_q);

// Reliable state snapshot for pre-failure rollback (SPEC §12.7)
struct ReliableSnapshot {
    int64_t t_ns;
    Vec3 p;
    Vec3 v;
    Quat q;
    Vec3 omega_s;
};

// Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7)
class ReliableRingBuffer {
public:
    static constexpr size_t CAPACITY = 64;

    ReliableRingBuffer();
    void clear();

    void push(int64_t t_ns,
              const Vec3& p,
              const Vec3& v,
              const Quat& q,
              const Vec3& omega_s);

    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

    bool get_latest(ReliableSnapshot& out) const;
    bool get_oldest(ReliableSnapshot& out) const;
    bool get_at_index(size_t ring_idx, ReliableSnapshot& out) const;

    int64_t t_ns_at(size_t idx) const { return t_ns_[idx]; }
    size_t head() const { return head_; }
    size_t count() const { return count_; }

private:
    int64_t t_ns_[CAPACITY];
    Vec3 p_[CAPACITY];
    Vec3 v_[CAPACITY];
    Quat q_[CAPACITY];
    Vec3 omega_s_[CAPACITY];
    size_t head_;
    size_t count_;
};

// Retrieve last reliable state prior to failure onset (SPEC §12.7)
ReliableSnapshot rollback_to_reliable(const ReliableRingBuffer& ring_buffer, int64_t t_onset_ns);

// Streaming FailureDetector and BlindPredictor class (SPEC §12.7, §12.9)
class FailureDetector {
public:
    FailureDetector();
    void reset();

    void record_reliable_state(int64_t t_ns,
                               const Vec3& p,
                               const Vec3& v,
                               const Quat& q,
                               const Vec3& omega_s);

    bool update(int64_t t_ns,
                bool is_valid,
                const Vec3& p,
                const Vec3& v,
                const Quat& q,
                const Vec3& omega_s);

    void declare_failure(int64_t failure_time_ns, int64_t failure_onset_ns);
    void notify_stream_end(int64_t final_t_ns);

    bool is_failed() const { return is_failed_; }
    int64_t failure_time_ns() const { return failure_time_ns_; }
    const ReliableSnapshot& frozen_state() const { return frozen_state_; }

    void predict(int64_t t_ns, Vec3& out_p, Quat& out_q) const;
    void predict_tau(double tau, Vec3& out_p, Quat& out_q) const;

    const ReliableRingBuffer& ring_buffer() const { return ring_buffer_; }

private:
    ReliableRingBuffer ring_buffer_;
    bool is_failed_;
    int64_t failure_time_ns_;
    int64_t invalid_start_ns_;
    int64_t last_valid_t_ns_;
    ReliableSnapshot frozen_state_;
};

#endif // BLIND_H
