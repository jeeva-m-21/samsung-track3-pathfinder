#ifndef GAP_H
#define GAP_H

#include "eskf.h"
#include "mat.h"
#include "quat.h"
#include "params.h"
#include <cstdint>
#include <cstddef>

enum class SampleValidity {
    VALID,
    NAN_OR_INF,
    ALL_ZERO,
    STUCK,
    OUT_OF_RANGE
};

enum class GapCase {
    NORMAL,
    ZERO_OR_NEGATIVE_DT,
    GAP,
    INVALID_SAMPLE
};

struct GapStepResult {
    NominalState state;
    Mat15 P;
    GapCase gap_case;
    int substeps;
    bool apply_updates;
    double r_scale;
    bool is_valid;
    SampleValidity validity;
};

// Classify sample validity (SPEC §12.7)
SampleValidity classify_sample(const Vec3& omega,
                               const Vec3& acc,
                               const Vec3* prev_omega = nullptr,
                               const Vec3* prev_acc = nullptr,
                               int repeat_count = 0);

inline bool is_valid_sample(SampleValidity v) {
    return v == SampleValidity::VALID;
}

// Compute median dt from sample sequence (SPEC §12.7)
double compute_dt_nom(const double* dt_samples, size_t count, double default_dt = 0.005);

// Motion smoothing: update smoothed body angular velocity omega_s (SPEC §12.7)
Vec3 update_smoothed_gyro(const Vec3& omega_s,
                          const Vec3& omega,
                          const Vec3& b_g,
                          double dt,
                          double tau = params::smooth_omega_tau);

// Post-gap recovery multiplier for measurement covariance (SPEC §12.7)
double get_recovery_multiplier(int64_t recover, int64_t recover_steps = params::recover_steps);

// Execute a single gap sub-step (velocity decay + orientation integration + Q inflation)
void gap_substep(NominalState& state,
                 Mat15& P,
                 const Vec3& omega_s,
                 double dt_sub);

// Propagate across a large time gap via sub-stepping (SPEC §12.7)
int propagate_gap(NominalState& state,
                  Mat15& P,
                  const Vec3& omega_s,
                  double dt,
                  double dt_nom);

// Streaming GapHandler class (SPEC §12.7)
class GapHandler {
public:
    static constexpr size_t DT_CALIB_MAX = 200;

    GapHandler();
    void reset();

    GapStepResult process_sample(int64_t t_ns,
                                 const Vec3& omega,
                                 const Vec3& acc,
                                 const NominalState& state,
                                 const Mat15& P);

    double dt_nom() const { return dt_nom_; }
    int64_t recover() const { return recover_; }
    const Vec3& omega_s() const { return omega_s_; }

private:
    int64_t recover_;
    double dt_nom_;
    bool dt_nom_frozen_;
    double dt_samples_[DT_CALIB_MAX];
    size_t dt_sample_count_;

    Vec3 omega_s_;
    int64_t prev_t_ns_;
    bool has_prev_sample_;
    Vec3 prev_omega_;
    Vec3 prev_acc_;
    int repeat_count_;
};

#endif // GAP_H
