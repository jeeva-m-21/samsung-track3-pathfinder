#ifndef ESKF_H
#define ESKF_H

#include "init.h"
#include "mat.h"
#include "quat.h"
#include "params.h"

// 1. Single-step nominal strapdown state propagation (SPEC §12.3)
NominalState strapdown_step(const NominalState& state,
                            const Vec3& omega,
                            const Vec3& acc,
                            double dt,
                            const Vec3& g_w);

// 2. Analytical 15x15 error-state transition matrix F (SPEC §12.4)
Mat15 build_f_matrix(const NominalState& state,
                     const Vec3& omega,
                     const Vec3& acc,
                     double dt);

// 3. Discrete 15x15 process noise covariance matrix Q (SPEC §12.4)
Mat15 build_q_matrix(double dt,
                     double noise_mult_gyro = params::noise_mult_gyro,
                     double noise_mult_acc = params::noise_mult_acc,
                     double walk_mult_bg = params::walk_mult_bg,
                     double walk_mult_ba = params::walk_mult_ba,
                     double kappa = 1.0);

// 4. Safe covariance conditioning (symmetrization + Cauchy-Schwarz congruence clamping)
void condition_covariance(Mat15& P,
                          double diag_floor = params::cov_diag_floor,
                          double diag_cap = params::cov_diag_cap);

// 5. Error covariance propagation P <- F P F^T + Q with safe conditioning (SPEC §12.4)
Mat15 propagate_covariance(const Mat15& P,
                           const Mat15& F,
                           const Mat15& Q);

// 6. Error state injection into NominalState (SPEC §12.5)
NominalState eskf_inject_state(const NominalState& state, const Vec15& delta_x);

// 7. Error state covariance reset P <- G P G^T (SPEC §12.5)
Mat15 eskf_reset_covariance(const Mat15& P,
                            const Vec3& delta_theta,
                            double diag_floor = params::cov_diag_floor,
                            double diag_cap = params::cov_diag_cap);

// 8. Generic Joseph-form measurement update with gating, injection, and reset (SPEC §12.5)
bool joseph_update(NominalState& state,
                   Mat15& P,
                   const Mat3x15& H,
                   const Vec3& y,
                   const Mat3& R,
                   double gate_threshold = 1e9);

#endif // ESKF_H
