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

Vec3 predict_blind_position(const Vec3& p_f,
                            const Vec3& v_f,
                            const Quat& q_f,
                            const Vec3& omega_f,
                            double tau,
                            double tau_v = params::tau_v,
                            double shrink_v = params::shrink_v,
                            double tau_omega = params::tau_omega_blind,
                            double stationary_v_thresh = params::stationary_v_thresh,
                            double w_walk = 0.0);

Quat predict_blind_orientation(const Quat& q_f,
                               const Vec3& omega_f,
                               double tau,
                               double tau_omega = params::tau_omega_blind);

Vec3 predict_blind_velocity(const Vec3& v_f,
                            double tau,
                            double tau_v = params::tau_v,
                            double shrink_v = params::shrink_v,
                            double stationary_v_thresh = params::stationary_v_thresh);

Vec3 predict_blind_velocity(const Vec3& v_f,
                            const Quat& q_f,
                            const Vec3& omega_f,
                            double tau,
                            double tau_v = params::tau_v,
                            double shrink_v = params::shrink_v,
                            double tau_omega = params::tau_omega_blind,
                            double stationary_v_thresh = params::stationary_v_thresh,
                            double w_walk = 0.0);

void predict_blind_pose(const Vec3& p_f,
                        const Vec3& v_f,
                        const Quat& q_f,
                        const Vec3& omega_f,
                        double tau,
                        Vec3& out_p,
                        Quat& out_q,
                        double w_walk = 0.0);

// Walking regime detector (SPEC §12.9, EXP-E64)
// Computes walking confidence score w_walk in [0, 1] based on
// vibration proxy (consecutive accel jitter), speed, and step energy.
struct WalkingDetector {
    double ewma_vibe;    // EWMA of consecutive accel differences (vibration proxy)
    double ewma_speed;   // EWMA of velocity magnitude
    double ewma_ped;     // EWMA of |a| deviation squared (step impulses)
    Vec3 prev_acc;
    bool has_prev_acc;
    int count;

    WalkingDetector()
        : ewma_vibe(0.0),
          ewma_speed(0.0),
          ewma_ped(0.0),
          prev_acc(math::make_vec3(0.0, 0.0, 0.0)),
          has_prev_acc(false),
          count(0) {}

    void reset() {
        ewma_vibe = 0.0;
        ewma_speed = 0.0;
        ewma_ped = 0.0;
        prev_acc = math::make_vec3(0.0, 0.0, 0.0);
        has_prev_acc = false;
        count = 0;
    }

    void update(const Vec3& accel, const Vec3& vel, double dt) {
        (void)dt;
        if (!has_prev_acc) {
            prev_acc = accel;
            has_prev_acc = true;
            return;
        }
        double d_a = (accel - prev_acc).norm();
        prev_acc = accel;

        if (!std::isfinite(d_a)) {
            return;
        }

        double a_norm = accel.norm();
        double g_dev = std::abs(a_norm - 9.80665);
        double speed = vel.norm();

        constexpr double alpha_vibe = 0.02;
        constexpr double alpha_ped = 0.01;
        constexpr double alpha_speed = 0.02;

        ewma_vibe += alpha_vibe * (d_a - ewma_vibe);
        ewma_ped += alpha_ped * (g_dev * g_dev - ewma_ped);
        ewma_speed += alpha_speed * (speed - ewma_speed);
        count++;
    }

    double get_w_walk() const {
        if (count < 50) {
            return 0.0;
        }
        // High vibration (motor jitter) -> drone -> vibe_score -> 0
        double vibe_score = 1.0;
        if (ewma_vibe > params::flight_vibe_thresh) {
            vibe_score = std::exp(-(ewma_vibe - params::flight_vibe_thresh) / 0.2);
        }
        // Speed score: normal walking is ~1 m/s, vehicle/drone is fast
        double speed_score = 1.0;
        if (ewma_speed > 2.0) {
            speed_score = std::exp(-(ewma_speed - 2.0) / (params::max_velocity_mps * 0.5));
        }
        // Pedestrian step score
        double ped_score = (ewma_ped > params::ped_energy_thresh) ? 1.0 : (ewma_ped / params::ped_energy_thresh);

        double w = vibe_score * speed_score * (0.2 + 0.8 * ped_score);
        if (w < 0.0) w = 0.0;
        if (w > 1.0) w = 1.0;
        return w;
    }
};

// Reliable state snapshot for pre-failure rollback (SPEC §12.7)
struct ReliableSnapshot {
    int64_t t_ns;
    Vec3 p;
    Vec3 v;
    Quat q;
    Vec3 omega_s;
    double w_walk;
};

// Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7)
class ReliableRingBuffer {
public:
    static constexpr size_t CAPACITY = 256;

    ReliableRingBuffer();
    void clear();

    void push(int64_t t_ns,
              const Vec3& p,
              const Vec3& v,
              const Quat& q,
              const Vec3& omega_s,
              double w_walk = 0.0);

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
    double w_walk_[CAPACITY];
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
                               const Vec3& omega_s,
                               double w_walk = 0.0);

    bool update(int64_t t_ns,
                bool is_valid,
                const Vec3& p,
                const Vec3& v,
                const Quat& q,
                const Vec3& omega_s,
                double w_walk = 0.0);

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
