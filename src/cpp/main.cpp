#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include "pipeline.h"

int main(int argc, char* argv[]) {
    const char* raw_input_path = nullptr;
    const char* output_path = nullptr;
    bool append_blind = false;
    const char* ablation_mode = "eskf_full"; // Default is full ESKF production pipeline

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
            if (!raw_input_path) {
                raw_input_path = argv[i];
            } else if (!output_path) {
                output_path = argv[i];
            }
        }
    }

    if (!raw_input_path) {
        std::fprintf(stderr,
                     "Usage: %s <input_path_or_dir> [output_trajectory.txt] [--append-blind] [--ablation MODE]\n",
                     argv[0]);
        return 1;
    }

    // Default output path if omitted: estimated_trajectory.txt
    if (!output_path) {
        output_path = "estimated_trajectory.txt";
    }

    // Resolve directory inputs: look for imu0/data.csv, mav0/imu0/data.csv, or data.csv
    char resolved_input[1024];
    std::snprintf(resolved_input, sizeof(resolved_input), "%s", raw_input_path);

    struct stat st;
    if (stat(raw_input_path, &st) == 0 && S_ISDIR(st.st_mode)) {
        const char* subpaths[] = {
            "/imu0/data.csv",
            "/mav0/imu0/data.csv",
            "/data.csv"
        };
        for (const char* sub : subpaths) {
            char candidate[1024];
            std::snprintf(candidate, sizeof(candidate), "%s%s", raw_input_path, sub);
            FILE* fp = std::fopen(candidate, "r");
            if (fp) {
                std::fclose(fp);
                std::snprintf(resolved_input, sizeof(resolved_input), "%s", candidate);
                break;
            }
        }
    }

    AblationMode mode = parse_ablation_mode(ablation_mode);
    return run_pipeline(resolved_input, output_path, mode, append_blind);
}
