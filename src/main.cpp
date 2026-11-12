#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>
#include "pipeline.h"

int main(int argc, char* argv[]) {
    const char* raw_input_path = nullptr;
    const char* output_path = nullptr;
    bool append_blind = false;
    const char* ablation_mode = "eskf_full"; // Default is full ESKF production pipeline

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--append-blind") == 0) {
            append_blind = true;
        } else if (std::strcmp(argv[i], "--input") == 0 || std::strcmp(argv[i], "-i") == 0) {
            if (i + 1 < argc) {
                raw_input_path = argv[++i];
            } else {
                std::fprintf(stderr, "Error: %s requires a path argument\n", argv[i]);
                return 1;
            }
        } else if (std::strncmp(argv[i], "--input=", 8) == 0) {
            raw_input_path = argv[i] + 8;
        } else if (std::strcmp(argv[i], "--output") == 0 || std::strcmp(argv[i], "-o") == 0) {
            if (i + 1 < argc) {
                output_path = argv[++i];
            } else {
                std::fprintf(stderr, "Error: %s requires a path argument\n", argv[i]);
                return 1;
            }
        } else if (std::strncmp(argv[i], "--output=", 9) == 0) {
            output_path = argv[i] + 9;
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
        // Fallback: search for standard input files in current working directory
        const char* local_candidates[] = {
            "imu0/data.csv",
            "mav0/imu0/data.csv",
            "data.csv"
        };
        for (const char* cand : local_candidates) {
            struct stat st_local;
            if (stat(cand, &st_local) == 0 && S_ISREG(st_local.st_mode)) {
                raw_input_path = cand;
                break;
            }
        }
    }

    if (!raw_input_path) {
        std::fprintf(stderr,
                     "Usage: %s <input_csv_dir_or_zip> [output_trajectory.txt] [--append-blind] [--ablation MODE]\n",
                     argv[0]);
        return 1;
    }

    // Default output path if omitted: estimated_trajectory.txt
    if (!output_path) {
        output_path = "estimated_trajectory.txt";
    }

    char resolved_input[1024];
    std::snprintf(resolved_input, sizeof(resolved_input), "%s", raw_input_path);
    char temp_extracted_file[1024] = "";

    struct stat st;
    if (stat(raw_input_path, &st) == 0 && S_ISDIR(st.st_mode)) {
        // Directory input: locate imu0/data.csv, mav0/imu0/data.csv, or data.csv
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
    } else {
        // Check if input is a ZIP archive
        bool is_zip = false;
        size_t in_len = std::strlen(raw_input_path);
        if (in_len >= 4 && std::strcmp(raw_input_path + in_len - 4, ".zip") == 0) {
            is_zip = true;
        } else {
            FILE* test_fp = std::fopen(raw_input_path, "rb");
            if (test_fp) {
                uint8_t sig[4];
                if (std::fread(sig, 1, 4, test_fp) == 4) {
                    if (sig[0] == 0x50 && sig[1] == 0x4B && (sig[2] == 0x03 || sig[2] == 0x05)) {
                        is_zip = true;
                    }
                }
                std::fclose(test_fp);
            }
        }

        if (is_zip) {
            char tmp_path[] = "/tmp/imu_eval_XXXXXX.csv";
            int fd = mkstemps(tmp_path, 4);
            if (fd >= 0) {
                close(fd);
            } else {
                std::snprintf(tmp_path, sizeof(tmp_path), "/tmp/imu_eval_%d.csv", static_cast<int>(getpid()));
            }

            char cmd[2048];
            // Extract imu0/data.csv or *data.csv from the zip archive into temp CSV
            std::snprintf(cmd, sizeof(cmd), "unzip -p '%s' '*imu0/data.csv' > '%s' 2>/dev/null",
                          raw_input_path, tmp_path);
            int ret = std::system(cmd);

            struct stat st_zip;
            if (ret != 0 || stat(tmp_path, &st_zip) != 0 || st_zip.st_size == 0) {
                std::snprintf(cmd, sizeof(cmd), "unzip -p '%s' '*data.csv' > '%s' 2>/dev/null",
                              raw_input_path, tmp_path);
                int ret2 = std::system(cmd);
                (void)ret2;
            }

            if (stat(tmp_path, &st_zip) != 0 || st_zip.st_size == 0) {
                std::snprintf(cmd, sizeof(cmd), "unzip -p '%s' '*.csv' > '%s' 2>/dev/null",
                              raw_input_path, tmp_path);
                int ret3 = std::system(cmd);
                (void)ret3;
            }

            if (stat(tmp_path, &st_zip) == 0 && st_zip.st_size > 0) {
                std::snprintf(resolved_input, sizeof(resolved_input), "%s", tmp_path);
                std::snprintf(temp_extracted_file, sizeof(temp_extracted_file), "%s", tmp_path);
            }
        }
    }

    AblationMode mode = parse_ablation_mode(ablation_mode);
    int status = run_pipeline(resolved_input, output_path, mode, append_blind);

    if (temp_extracted_file[0] != '\0') {
        std::remove(temp_extracted_file);
    }
    return status;
}
