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

AblationMode parse_ablation_mode(const char* str);

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
