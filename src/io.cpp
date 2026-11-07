#include "io.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sys/stat.h>

namespace io {

namespace {

struct Token {
    const char* ptr;
    size_t len;
};

// Helper to create directories recursively if path has folders
void create_parent_dirs(const char* path) {
    char temp[1024];
    size_t len = 0;
    while (path[len] && len + 1 < sizeof(temp)) {
        temp[len] = path[len];
        len++;
    }
    temp[len] = '\0';
    for (size_t i = 1; i < len; ++i) {
        if (temp[i] == '/') {
            temp[i] = '\0';
            mkdir(temp, 0755);
            temp[i] = '/';
        }
    }
}

} // namespace

bool parse_imu_line(const char* line, IMUSample& sample) {
    sample.t_ns = 0;
    sample.wx = 0.0;
    sample.wy = 0.0;
    sample.wz = 0.0;
    sample.ax = 0.0;
    sample.ay = 0.0;
    sample.az = 0.0;
    sample.is_valid = false;
    sample.flags = FLAG_OK;

    // Skip leading whitespace
    const char* p = line;
    while (*p == ' ' || *p == '\t' || *p == '\r') {
        p++;
    }

    // Empty line or comment line starting with '#'
    if (*p == '\0' || *p == '\n') {
        sample.flags = FLAG_COMMENT_OR_BLANK;
        return false;
    }
    if (*p == '#') {
        sample.flags = FLAG_COMMENT_OR_BLANK;
        return false;
    }

    // Tokenize line (up to 16 tokens)
    Token tokens[16];
    size_t num_tokens = 0;

    bool has_comma = (std::strchr(p, ',') != nullptr);

    if (has_comma) {
        const char* token_start = p;
        while (*p != '\0' && *p != '\n') {
            if (*p == ',') {
                if (num_tokens < 16) {
                    const char* t_start = token_start;
                    const char* t_end = p;
                    while (t_start < t_end && (*t_start == ' ' || *t_start == '\t' || *t_start == '\r')) {
                        t_start++;
                    }
                    while (t_end > t_start && (*(t_end - 1) == ' ' || *(t_end - 1) == '\t' || *(t_end - 1) == '\r')) {
                        t_end--;
                    }
                    tokens[num_tokens].ptr = t_start;
                    tokens[num_tokens].len = static_cast<size_t>(t_end - t_start);
                    num_tokens++;
                }
                token_start = p + 1;
            }
            p++;
        }
        // Last token
        if (num_tokens < 16) {
            const char* t_start = token_start;
            const char* t_end = p;
            while (t_start < t_end && (*t_start == ' ' || *t_start == '\t' || *t_start == '\r')) {
                t_start++;
            }
            while (t_end > t_start && (*(t_end - 1) == ' ' || *(t_end - 1) == '\t' || *(t_end - 1) == '\r')) {
                t_end--;
            }
            tokens[num_tokens].ptr = t_start;
            tokens[num_tokens].len = static_cast<size_t>(t_end - t_start);
            num_tokens++;
        }
    } else {
        // Delimited by whitespace
        while (*p != '\0' && *p != '\n') {
            while (*p == ' ' || *p == '\t' || *p == '\r') {
                p++;
            }
            if (*p == '\0' || *p == '\n') break;
            const char* t_start = p;
            while (*p != '\0' && *p != '\n' && *p != ' ' && *p != '\t' && *p != '\r') {
                p++;
            }
            if (num_tokens < 16) {
                tokens[num_tokens].ptr = t_start;
                tokens[num_tokens].len = static_cast<size_t>(p - t_start);
                num_tokens++;
            }
        }
    }

    // 1. Parse timestamp (token 0)
    if (num_tokens == 0 || tokens[0].len == 0) {
        sample.flags |= FLAG_MISSING_TIMESTAMP;
        return false;
    }

    char ts_buf[64];
    size_t ts_len = tokens[0].len < sizeof(ts_buf) - 1 ? tokens[0].len : sizeof(ts_buf) - 1;
    std::memcpy(ts_buf, tokens[0].ptr, ts_len);
    ts_buf[ts_len] = '\0';

    char* endptr = nullptr;
    long long t_val = std::strtoll(ts_buf, &endptr, 10);
    if (endptr != ts_buf) {
        // If there's a decimal dot, check if trailing part is all '0's (e.g. 14036.0)
        if (*endptr == '.') {
            const char* d_check = endptr + 1;
            while (*d_check == '0') {
                d_check++;
            }
            if (*d_check == '\0') {
                endptr = const_cast<char*>(d_check);
            }
        }
        if (*endptr == '\0') {
            sample.t_ns = static_cast<int64_t>(t_val);
        } else {
            sample.flags |= FLAG_MISSING_TIMESTAMP;
            return false;
        }
    } else {
        sample.flags |= FLAG_MISSING_TIMESTAMP;
        return false;
    }

    // 2. Parse 6 channels: wx, wy, wz, ax, ay, az
    double vals[6];
    for (int k = 0; k < 6; ++k) {
        vals[k] = std::numeric_limits<double>::quiet_NaN();
    }

    if (num_tokens < 7) {
        sample.flags |= FLAG_TRUNCATED;
        sample.flags |= FLAG_NAN_OR_INF;
    }

    for (size_t k = 0; k < 6; ++k) {
        size_t col = k + 1;
        if (col < num_tokens && tokens[col].len > 0) {
            char fbuf[64];
            size_t flen = tokens[col].len < sizeof(fbuf) - 1 ? tokens[col].len : sizeof(fbuf) - 1;
            std::memcpy(fbuf, tokens[col].ptr, flen);
            fbuf[flen] = '\0';

            char* fend = nullptr;
            double v = std::strtod(fbuf, &fend);
            if (fend != fbuf && *fend == '\0' && std::isfinite(v)) {
                vals[k] = v;
            } else {
                vals[k] = std::numeric_limits<double>::quiet_NaN();
                sample.flags |= FLAG_NAN_OR_INF;
            }
        } else {
            vals[k] = std::numeric_limits<double>::quiet_NaN();
            sample.flags |= FLAG_NAN_OR_INF;
        }
    }

    sample.wx = vals[0];
    sample.wy = vals[1];
    sample.wz = vals[2];
    sample.ax = vals[3];
    sample.ay = vals[4];
    sample.az = vals[5];

    // 3. Check bounds and all zeros if all finite
    if (!(sample.flags & FLAG_NAN_OR_INF)) {
        if (std::abs(sample.wx) > params::invalid_gyro_max ||
            std::abs(sample.wy) > params::invalid_gyro_max ||
            std::abs(sample.wz) > params::invalid_gyro_max) {
            sample.flags |= FLAG_OUT_OF_RANGE_GYRO;
        }
        if (std::abs(sample.ax) > params::invalid_acc_max ||
            std::abs(sample.ay) > params::invalid_acc_max ||
            std::abs(sample.az) > params::invalid_acc_max) {
            sample.flags |= FLAG_OUT_OF_RANGE_ACC;
        }
        if (sample.wx == 0.0 && sample.wy == 0.0 && sample.wz == 0.0 &&
            sample.ax == 0.0 && sample.ay == 0.0 && sample.az == 0.0) {
            sample.flags |= FLAG_ALL_ZEROS;
        }
    }

    sample.is_valid = (sample.flags == FLAG_OK);
    return true;
}

IMUReader::IMUReader()
    : file_(nullptr),
      buf_pos_(0),
      buf_len_(0),
      eof_reached_(false),
      skipped_timestamps_(0),
      valid_count_(0),
      invalid_count_(0),
      total_lines_read_(0) {}

IMUReader::~IMUReader() {
    close();
}

bool IMUReader::open(const char* filepath) {
    close();
    skipped_timestamps_ = 0;
    valid_count_ = 0;
    invalid_count_ = 0;
    total_lines_read_ = 0;
    buf_pos_ = 0;
    buf_len_ = 0;
    eof_reached_ = false;

    if (!filepath || filepath[0] == '\0') {
        return false;
    }

    struct stat st;
    if (stat(filepath, &st) == 0 && S_ISDIR(st.st_mode)) {
        char subpath[1024];
        snprintf(subpath, sizeof(subpath), "%s/imu0/data.csv", filepath);
        if (stat(subpath, &st) == 0 && !S_ISDIR(st.st_mode)) {
            file_ = std::fopen(subpath, "r");
        } else {
            snprintf(subpath, sizeof(subpath), "%s/mav0/imu0/data.csv", filepath);
            if (stat(subpath, &st) == 0 && !S_ISDIR(st.st_mode)) {
                file_ = std::fopen(subpath, "r");
            } else {
                snprintf(subpath, sizeof(subpath), "%s/data.csv", filepath);
                file_ = std::fopen(subpath, "r");
            }
        }
    } else {
        file_ = std::fopen(filepath, "r");
    }

    return file_ != nullptr;
}

void IMUReader::close() {
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

bool IMUReader::read_line(char* out_line, size_t max_len) {
    if (!file_ || max_len == 0) {
        return false;
    }

    size_t out_idx = 0;
    while (true) {
        if (buf_pos_ >= buf_len_) {
            if (eof_reached_) {
                if (out_idx > 0) {
                    out_line[out_idx] = '\0';
                    return true;
                }
                return false;
            }
            buf_len_ = std::fread(buffer_, 1, BUFFER_SIZE, file_);
            buf_pos_ = 0;
            if (buf_len_ == 0) {
                eof_reached_ = true;
                if (out_idx > 0) {
                    out_line[out_idx] = '\0';
                    return true;
                }
                return false;
            }
        }

        char c = buffer_[buf_pos_++];
        if (c == '\n') {
            if (out_idx > 0 && out_line[out_idx - 1] == '\r') {
                out_idx--;
            }
            out_line[out_idx] = '\0';
            return true;
        }
        if (out_idx + 1 < max_len) {
            out_line[out_idx++] = c;
        }
    }
}

bool IMUReader::next(IMUSample& sample) {
    while (read_line(line_buf_, sizeof(line_buf_))) {
        total_lines_read_++;
        IMUSample s;
        bool has_timestamp = parse_imu_line(line_buf_, s);
        if (!has_timestamp) {
            if (s.flags & FLAG_MISSING_TIMESTAMP) {
                skipped_timestamps_++;
            }
            continue;
        }

        if (s.is_valid) {
            valid_count_++;
        } else {
            invalid_count_++;
        }
        sample = s;
        return true;
    }
    return false;
}

TrajectoryWriter::TrajectoryWriter() : file_(nullptr) {}

TrajectoryWriter::~TrajectoryWriter() {
    close();
}

bool TrajectoryWriter::open(const char* filepath) {
    close();
    if (!filepath || filepath[0] == '\0') {
        return false;
    }

    create_parent_dirs(filepath);
    file_ = std::fopen(filepath, "w");
    if (!file_) {
        return false;
    }

    setvbuf(file_, write_buf_, _IOFBF, sizeof(write_buf_));
    return true;
}

void TrajectoryWriter::close() {
    if (file_) {
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
    }
}

bool TrajectoryWriter::write_pose(int64_t t_ns, double tx, double ty, double tz,
                                  double qx, double qy, double qz, double qw) {
    if (!file_) {
        return false;
    }

    int64_t sec = t_ns / 1000000000LL;
    int64_t nsec = t_ns % 1000000000LL;
    if (nsec < 0) {
        nsec = -nsec;
    }

    int ret = std::fprintf(file_, "%" PRId64 ".%09" PRId64 " %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n",
                           sec, nsec, tx, ty, tz, qx, qy, qz, qw);
    return ret > 0;
}

} // namespace io
