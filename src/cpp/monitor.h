#ifndef MONITOR_H
#define MONITOR_H

#include "eskf.h"
#include "mat.h"
#include "params.h"
#include <cstdint>

// Pure helper functions (SPEC §12.6)
double compute_residual(const Vec3& curr, const Vec3& prev);
double update_moving_variance(double prev_var, double residual, double alpha = params::monitor_alpha);
double compute_noise_ratio(double sigma_now, double sigma_ref);
double compute_quality_score(double noise_ratio, double min_quality = 0.05, double max_quality = 1.0);
double compute_kappa(double noise_ratio, double kappa_max = params::kappa_max);

// Stationary gyro bias pseudo-update (SPEC §12.6)
Mat3x15 build_gyro_bias_h_matrix();
Mat3 build_gyro_bias_r_matrix(double r_scale = 1.0);
bool apply_gyro_bias_update(NominalState& state,
                            Mat15& P,
                            const Vec3& omega_meas,
                            double gate_threshold = 1e9,
                            double r_scale = 1.0);

// Streaming DegradationMonitor class
class DegradationMonitor {
public:
    DegradationMonitor();
    void reset();

    void step(double t,
              const Vec3& omega,
              const Vec3& acc,
              bool is_stationary = false,
              const Vec3* omega_static_mean = nullptr);

    double quality() const { return quality_; }
    double kappa() const { return kappa_; }
    double noise_ratio() const { return noise_ratio_; }
    double n() const { return noise_ratio_; }
    bool calibrated() const { return calibrated_; }
    int64_t count() const { return sample_count_; }
    double var_acc() const { return var_acc_; }
    double var_gyro() const { return var_gyro_; }
    double var_acc_ref() const { return var_acc_ref_; }
    double var_gyro_ref() const { return var_gyro_ref_; }
    double residual_acc() const { return residual_acc_; }
    double residual_gyro() const { return residual_gyro_; }
    Vec3 bias_trend() const { return bias_trend_; }

private:
    double t_start_;
    double t_prev_;
    int64_t sample_count_;
    Vec3 prev_acc_;
    Vec3 prev_omega_;
    double var_acc_;
    double var_gyro_;
    double var_acc_ref_;
    double var_gyro_ref_;
    bool calibrated_;
    double n_acc_;
    double n_gyro_;
    double noise_ratio_;
    double quality_;
    double kappa_;
    double residual_acc_;
    double residual_gyro_;
    Vec3 bias_trend_;
    int64_t bias_trend_count_;
};

#endif // MONITOR_H
