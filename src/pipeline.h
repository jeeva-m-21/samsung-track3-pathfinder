#ifndef PIPELINE_H
#define PIPELINE_H

#include <cstdint>
#include <cstddef>
#include "init.h"
#include "eskf.h"
#include "updates.h"
#include "monitor.h"
#include "gap.h"
#include "blind.h"
#include "io.h"

enum class AblationMode {
    NAIVE,
    MAHONY,
    ESKF_BASIC,
    ESKF_FULL
};

struct PipelineOptions {
    AblationMode mode = AblationMode::ESKF_FULL;
    bool append_blind = false;
    bool disable_spike_filter = false;
    bool enable_adaptive_q = params::enable_adaptive_q;
    bool adaptive_q_scale_acc = params::adaptive_q_scale_acc;
    double adaptive_q_alpha = params::adaptive_q_alpha;
    double adaptive_q_min = params::adaptive_q_min;
    double adaptive_q_max = params::adaptive_q_max;
    bool enable_unbiased_adaptive_q = params::enable_unbiased_adaptive_q;
#ifdef ENABLE_ORACLES
    const char* telemetry_path = nullptr;
    const char* oracle_gt_path = nullptr;
    bool oracle_att = false;
    bool oracle_yaw = false;
    bool oracle_bias = false;
    bool oracle_v0 = false;
    bool disable_tau_ou = false;
    bool disable_zupt = false;
    bool disable_tilt = false;
    bool disable_vel_bound = false;
#endif
};

AblationMode parse_ablation_mode(const char* str);

int run_pipeline_with_options(const char* input_path,
                              const char* output_path,
                              const PipelineOptions& options);

int run_pipeline(const char* input_path,
                 const char* output_path,
                 AblationMode mode = AblationMode::ESKF_FULL,
                 bool append_blind = false);

int run_eskf_full(const char* input_path,
                  const char* output_path,
                  bool append_blind = false);

int run_eskf_basic(const char* input_path,
                   const char* output_path,
                   bool append_blind = false);

int run_naive(const char* input_path,
              const char* output_path,
              bool append_blind = false);

int run_mahony(const char* input_path,
               const char* output_path,
               bool append_blind = false);

#endif // PIPELINE_H
