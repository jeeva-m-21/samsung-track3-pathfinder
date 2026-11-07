#include <cinttypes>
#include <cstdio>
#include <cstring>
#include "io.h"
#include "params.h"

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
                i++; // Skip params argument in skeleton
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

    // Suppress unused warning in skeleton
    (void)ablation_mode;

    io::IMUReader reader;
    if (!reader.open(input_path)) {
        std::fprintf(stderr, "Error: failed to open input '%s'\n", input_path);
        return 1;
    }

    io::TrajectoryWriter writer;
    if (!writer.open(output_path)) {
        std::fprintf(stderr, "Error: failed to open output '%s'\n", output_path);
        return 1;
    }

    io::IMUSample sample;
    int64_t row_count = 0;
    int64_t first_t_ns = 0;
    int64_t last_t_ns = 0;

    while (reader.next(sample)) {
        if (row_count == 0) {
            first_t_ns = sample.t_ns;
        }
        last_t_ns = sample.t_ns;
        row_count++;

        // Dummy initial pose for WP-30 skeleton: tx=0, ty=0, tz=0, qx=0, qy=0, qz=0, qw=1
        writer.write_pose(sample.t_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    }

    if (append_blind && row_count > 1) {
        int64_t dt_ns = (last_t_ns - first_t_ns) / (row_count - 1);
        if (dt_ns <= 0) {
            dt_ns = 5000000LL; // 5ms default (200 Hz)
        }
        int64_t extrap_duration_ns = 5000000000LL; // 5.0 s
        int64_t num_extrap = extrap_duration_ns / dt_ns;
        for (int64_t step = 1; step <= num_extrap; ++step) {
            int64_t t_extrap = last_t_ns + step * dt_ns;
            writer.write_pose(t_extrap, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        }
    }

    writer.close();
    reader.close();
    return 0;
}
