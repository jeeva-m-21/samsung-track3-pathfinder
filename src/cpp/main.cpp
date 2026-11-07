#include <cinttypes>
#include <cstdio>
#include <cstring>
#include "pipeline.h"

int main(int argc, char* argv[]) {
    const char* input_path = nullptr;
    const char* output_path = nullptr;
    bool append_blind = false;
    const char* ablation_mode = "eskf_full";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--append-blind") == 0) {
            append_blind = true;
        } else if (std::strcmp(argv[i], "--ablation") == 0) {
            if (i + 1 < argc) {
                ablation_mode = argv[++i];
            } else {
                std::fprintf(stderr, "Error: --ablation requires a mode argument\n");
                return 1;
            }
        } else if (std::strncmp(argv[i], "--ablation=", 11) == 0) {
            ablation_mode = argv[i] + 11;
        } else if (std::strcmp(argv[i], "--params") == 0) {
            if (i + 1 < argc) {
                i++; // Skip params argument
            }
        } else if (argv[i][0] != '-') {
            if (!input_path) {
                input_path = argv[i];
            } else if (!output_path) {
                output_path = argv[i];
            }
        }
    }

    if (!input_path || !output_path) {
        std::fprintf(stderr,
                     "Usage: %s <imu_input.csv> <output_trajectory.txt> [--append-blind] [--ablation MODE]\n",
                     argv[0]);
        return 1;
    }

    AblationMode mode = parse_ablation_mode(ablation_mode);
    return run_pipeline(input_path, output_path, mode, append_blind);
}
