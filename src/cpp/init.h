#ifndef INIT_H
#define INIT_H

#include "mat.h"
#include "quat.h"
#include "params.h"
#include <cstddef>
#include <cstdint>

// 16-parameter nominal kinematic state (SPEC §12.2, §12.3)
struct NominalState {
    Vec3 p;
    Vec3 v;
    Quat q;
    Vec3 b_g;
    Vec3 b_a;

    static NominalState zeros() {
        NominalState s;
        s.p = math::make_vec3(0.0, 0.0, 0.0);
        s.v = math::make_vec3(0.0, 0.0, 0.0);
        s.q = Quat::identity();
        s.b_g = math::make_vec3(0.0, 0.0, 0.0);
        s.b_a = math::make_vec3(0.0, 0.0, 0.0);
        return s;
    }
};

// Standalone tilt and motion helper functions
Quat compute_initial_tilt(const Vec3& a_mean);
bool detect_motion(const Vec3* omega_buf, const Vec3* acc_buf, size_t count);
Mat15 initialize_covariance(bool motion_detected = false);

// Online streaming initializer with fixed-size static buffer (SPEC §12.2)
class Initializer {
public:
    static constexpr size_t MAX_SAMPLES = 400;

    Initializer();
    void reset();

    // Feeds an IMU sample. Returns true when initialisation window is complete.
    bool add_sample(int64_t t_ns, const Vec3& w, const Vec3& a);

    // Returns true when enough samples spanning T_init (or fallback) have been gathered.
    bool is_ready() const;

    // Computes initial NominalState, P0 covariance, and world gravity vector g_w.
    void finish(NominalState& state, Mat15& P0, Vec3& g_w) const;

    size_t count() const { return count_; }
    bool is_motion_detected() const { return motion_detected_; }
    double g_mag() const { return g_mag_; }

private:
    int64_t t_buf_[MAX_SAMPLES];
    Vec3 w_buf_[MAX_SAMPLES];
    Vec3 a_buf_[MAX_SAMPLES];
    size_t count_;
    bool is_ready_;
    mutable bool motion_detected_;
    mutable double g_mag_;
};

#endif // INIT_H
