#ifndef UPDATES_H
#define UPDATES_H

#include "eskf.h"
#include "init.h"
#include "mat.h"
#include "quat.h"
#include "params.h"
#include <cstddef>

// Tilt update gating check (SPEC §12.5)
bool tilt_gating(const Vec3& acc,
                 const Vec3& omega,
                 double g_mag = 9.80665,
                 double quality = 1.0);

// Compute tilt innovation y = z - h_hat and predicted gravity direction in body frame (SPEC §12.5)
void compute_tilt_innovation(const NominalState& state,
                             const Vec3& acc,
                             const Vec3& g_w,
                             Vec3& y,
                             Vec3& h_hat);

// Build 3x15 tilt measurement Jacobian matrix H (SPEC §12.5)
Mat3x15 build_tilt_h_matrix(const Vec3& h_hat);

// Build 3x3 tilt measurement noise covariance matrix R (SPEC §12.5)
Mat3 build_tilt_r_matrix(double quality = 1.0, double r_scale = 1.0);

// Tilt update full step with gyro gating (SPEC §12.5)
bool apply_tilt_update(NominalState& state,
                       Mat15& P,
                       const Vec3& acc,
                       const Vec3& omega,
                       double quality,
                       const Vec3& g_w,
                       double gate_threshold = 1e9,
                       double r_scale = 1.0);

// Overload without explicit omega (assumes stationary/near-zero gyro)
bool apply_tilt_update(NominalState& state,
                       Mat15& P,
                       const Vec3& acc,
                       double quality,
                       const Vec3& g_w,
                       double gate_threshold = 1e9);

// Fixed-capacity circular ring buffer for streaming stationary detection (SPEC §12.5)
class StationaryBuffer {
public:
    static constexpr size_t CAPACITY = 64;

    StationaryBuffer();
    void clear();
    void add_sample(double t, const Vec3& omega, const Vec3& acc);
    bool is_stationary(double g_mag = 9.80665,
                       double quality = 1.0,
                       double window_s = params::zupt_window_s) const;
    Vec3 get_mean_gyro(double window_s = params::zupt_window_s) const;
    size_t count() const { return count_; }

private:
    double timestamps_[CAPACITY];
    Vec3 gyro_[CAPACITY];
    Vec3 acc_[CAPACITY];
    size_t head_;
    size_t count_;
};

// Build 3x15 ZUPT measurement Jacobian matrix H (SPEC §12.5)
Mat3x15 build_zupt_h_matrix();

// Build 3x3 ZUPT measurement noise covariance matrix R (SPEC §12.5)
Mat3 build_zupt_r_matrix(double r_scale = 1.0);

// Zero Velocity Update (ZUPT) measurement update (SPEC §12.5)
bool apply_zupt_update(NominalState& state,
                       Mat15& P,
                       double gate_threshold = 1e9,
                       double r_scale = 1.0);

#endif // UPDATES_H
