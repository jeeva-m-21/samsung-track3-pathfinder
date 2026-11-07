#ifndef IO_H
#define IO_H

#include <cstdint>
#include <cstdio>
#include "params.h"

namespace io {

enum ParserFlags : uint32_t {
    FLAG_OK = 0,
    FLAG_MISSING_TIMESTAMP = 1 << 0,
    FLAG_NAN_OR_INF = 1 << 1,
    FLAG_TRUNCATED = 1 << 2,
    FLAG_OUT_OF_RANGE_GYRO = 1 << 3,
    FLAG_OUT_OF_RANGE_ACC = 1 << 4,
    FLAG_ALL_ZEROS = 1 << 5,
    FLAG_COMMENT_OR_BLANK = 1 << 6
};

struct IMUSample {
    int64_t t_ns;
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    bool is_valid;
    uint32_t flags;
};

// Pure function to parse a single line into an IMUSample.
// Returns true if a valid timestamp was found (even if fields have NaN/Inf/out-of-range).
// Returns false if line is a comment/blank or timestamp is missing/unparseable.
bool parse_imu_line(const char* line, IMUSample& sample);

class IMUReader {
public:
    IMUReader();
    ~IMUReader();

    IMUReader(const IMUReader&) = delete;
    IMUReader& operator=(const IMUReader&) = delete;

    bool open(const char* filepath);
    void close();

    // Streaming iterator: yields next row with a timestamp
    // Returns false on EOF or unrecoverable error
    bool next(IMUSample& sample);

    int64_t skipped_timestamps() const { return skipped_timestamps_; }
    int64_t valid_count() const { return valid_count_; }
    int64_t invalid_count() const { return invalid_count_; }
    int64_t total_lines_read() const { return total_lines_read_; }

private:
    FILE* file_;
    static constexpr size_t BUFFER_SIZE = 65536;
    static constexpr size_t MAX_LINE_LEN = 2048;

    char buffer_[BUFFER_SIZE];
    size_t buf_pos_;
    size_t buf_len_;
    bool eof_reached_;

    char line_buf_[MAX_LINE_LEN];

    int64_t skipped_timestamps_;
    int64_t valid_count_;
    int64_t invalid_count_;
    int64_t total_lines_read_;

    bool read_line(char* out_line, size_t max_len);
};

class TrajectoryWriter {
public:
    TrajectoryWriter();
    ~TrajectoryWriter();

    TrajectoryWriter(const TrajectoryWriter&) = delete;
    TrajectoryWriter& operator=(const TrajectoryWriter&) = delete;

    bool open(const char* filepath);
    void close();

    bool write_pose(int64_t t_ns, double tx, double ty, double tz,
                    double qx, double qy, double qz, double qw);

private:
    FILE* file_;
    static constexpr size_t WRITE_BUFFER_SIZE = 65536;
    char write_buf_[WRITE_BUFFER_SIZE];
};

} // namespace io

#endif // IO_H
