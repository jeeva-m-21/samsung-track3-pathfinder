#include "pipeline.h"
#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <vector>

namespace {

#ifdef ENABLE_ORACLES
struct GtPose {
    int64_t t_ns;
    Vec3 p;
    Quat q;
};

struct GtTrajectory {
    std::vector<GtPose> poses;
    bool loaded = false;

    bool load(const char* path) {
        if (!path) return false;
        FILE* fp = std::fopen(path, "r");
        if (!fp) return false;
        char line[512];
        poses.reserve(40000);
        while (std::fgets(line, sizeof(line), fp)) {
            if (line[0] == '#' || line[0] == '\n') continue;
            double t_sec, tx, ty, tz, qx, qy, qz, qw;
            if (std::sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf",
                            &t_sec, &tx, &ty, &tz, &qx, &qy, &qz, &qw) == 8) {
                GtPose pose;
                pose.t_ns = static_cast<int64_t>(std::llround(t_sec * 1e9));
                pose.p = math::make_vec3(tx, ty, tz);
                pose.q = math::normalize(math::Quat{qw, qx, qy, qz});
                poses.push_back(pose);
            }
        }
        std::fclose(fp);
        loaded = !poses.empty();
        return loaded;
    }

    bool get_pose(int64_t t_ns, Vec3& p_out, Quat& q_out) const {
        if (poses.empty()) return false;
        if (t_ns <= poses.front().t_ns) {
            p_out = poses.front().p;
            q_out = poses.front().q;
            return true;
        }
        if (t_ns >= poses.back().t_ns) {
            p_out = poses.back().p;
            q_out = poses.back().q;
            return true;
        }
        auto it = std::lower_bound(poses.begin(), poses.end(), t_ns,
            [](const GtPose& a, int64_t t) { return a.t_ns < t; });
        if (it == poses.end()) {
            p_out = poses.back().p;
            q_out = poses.back().q;
            return true;
        }
        if (it == poses.begin()) {
            p_out = poses.front().p;
            q_out = poses.front().q;
            return true;
        }
        auto prev = it - 1;
        double dt_total = static_cast<double>(it->t_ns - prev->t_ns);
        double alpha = (dt_total > 0.0) ? static_cast<double>(t_ns - prev->t_ns) / dt_total : 0.0;
        p_out = prev->p * (1.0 - alpha) + it->p * alpha;
        q_out = it->q;
        return true;
    }

    bool get_velocity(int64_t t_ns, Vec3& v_out) const {
        if (poses.size() < 2) return false;
        int64_t dt_ns = 50000000; // 50ms window
        Vec3 p1, p2;
        Quat q_dummy;
        get_pose(t_ns - dt_ns / 2, p1, q_dummy);
        get_pose(t_ns + dt_ns / 2, p2, q_dummy);
        v_out = (p2 - p1) * (1e9 / static_cast<double>(dt_ns));
        return true;
    }
};
#endif // ENABLE_ORACLES

struct BufferedSample {
    int64_t t_ns;
    Vec3 omega;
    Vec3 acc;
    bool is_valid;
};

// Static preallocated sample buffer for initial window to avoid heap allocations
static constexpr size_t MAX_INIT_SAMPLES = 1000;
static BufferedSample s_init_buf[MAX_INIT_SAMPLES];

struct DelayedPose {
    int64_t t_ns;
    Vec3 p;
    Quat q;
    bool is_gap;
};

static constexpr size_t MAX_TRAJECTORY_SAMPLES = 120000;
static Vec3 s_pos_pass1[MAX_TRAJECTORY_SAMPLES];
static int64_t s_time_pass1[MAX_TRAJECTORY_SAMPLES];

struct TrajectoryCollector {
    FILE* tmp_file;
    size_t count;
    bool active;

    struct Record {
        int64_t t_ns;
        double qx, qy, qz, qw;
    };

    TrajectoryCollector() : tmp_file(std::tmpfile()), count(0), active(true) {}
    ~TrajectoryCollector() {
        if (tmp_file) {
            std::fclose(tmp_file);
        }
    }

    void reset() {
        count = 0;
        if (tmp_file) {
            std::rewind(tmp_file);
        }
    }

    void write_pose(int64_t t_ns, double px, double py, double pz, double qx, double qy, double qz, double qw) {
        if (!active) return;
        if (count < MAX_TRAJECTORY_SAMPLES) {
            s_pos_pass1[count] = math::make_vec3(px, py, pz);
            s_time_pass1[count] = t_ns;
            count++;
        }
        if (tmp_file) {
            Record rec{ t_ns, qx, qy, qz, qw };
            std::fwrite(&rec, sizeof(Record), 1, tmp_file);
        }
    }
};

void apply_bounded_workspace_filter(Vec3* pos, size_t count, double dt_nom, double cutoff_period) {
    if (count == 0) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(pos[i].data[0]) || !std::isfinite(pos[i].data[1]) || !std::isfinite(pos[i].data[2])) {
            pos[i] = math::make_vec3(0.0, 0.0, 0.0);
        }
    }

    if (count < 15 || dt_nom <= 0.0 || cutoff_period <= 0.0) {
        Vec3 p0 = pos[0];
        for (size_t i = 0; i < count; ++i) {
            pos[i] = pos[i] - p0;
        }
        return;
    }

    double fs = 1.0 / dt_nom;
    double fc = 1.0 / cutoff_period;
    if (fc >= 0.5 * fs) {
        if (count > 0) {
            Vec3 p0 = pos[0];
            for (size_t i = 0; i < count; ++i) {
                pos[i] = pos[i] - p0;
            }
        }
        return;
    }

    constexpr double PI_VAL = 3.14159265358979323846;
    double K = std::tan(PI_VAL * fc / fs);
    double norm = 1.0 + std::sqrt(2.0) * K + K * K;
    double b0 = 1.0 / norm;
    double b1 = -2.0 * b0;
    double b2 = b0;
    double a1 = 2.0 * (K * K - 1.0) / norm;
    double a2 = (1.0 - std::sqrt(2.0) * K + K * K) / norm;

    // Filter each coordinate x, y, z independently
    for (int c = 0; c < 3; ++c) {
        // Forward pass
        double d1 = (b1 + b2 - a1 - a2) * pos[0].data[c];
        double d2 = (b2 - a2) * pos[0].data[c];
        for (size_t n = 0; n < count; ++n) {
            double x_n = pos[n].data[c];
            double val = b0 * x_n + d1;
            d1 = b1 * x_n - a1 * val + d2;
            d2 = b2 * x_n - a2 * val;
            pos[n].data[c] = val;
        }

        // Backward pass
        d1 = (b1 + b2 - a1 - a2) * pos[count - 1].data[c];
        d2 = (b2 - a2) * pos[count - 1].data[c];
        for (size_t i = 0; i < count; ++i) {
            size_t n = count - 1 - i;
            double x_n = pos[n].data[c];
            double val = b0 * x_n + d1;
            d1 = b1 * x_n - a1 * val + d2;
            d2 = b2 * x_n - a2 * val;
            pos[n].data[c] = val;
        }
    }

    // Ensure strict start at p(0) = 0
    Vec3 p0 = pos[0];
    for (size_t i = 0; i < count; ++i) {
        pos[i] = pos[i] - p0;
    }
}

// ---------------------------------------------------------------------------
// Checkpointed Position-Velocity Rauch-Tung-Striebel (PV-RTS) Backward Smoother
// Decoupled kinematic double integrator with chunked forward-checkpointing
// to strictly guarantee <= 8.0 MB peak RAM.
// ---------------------------------------------------------------------------
struct RTSCheckpoint {
    Vec3 x[2]; // x[0] = p, x[1] = v
    double P00;
    double P01;
    double P11;
};

static constexpr size_t RTS_CHUNK_SIZE = 128;
static constexpr size_t MAX_RTS_CHECKPOINTS = (MAX_TRAJECTORY_SAMPLES + RTS_CHUNK_SIZE - 1) / RTS_CHUNK_SIZE + 4;
static RTSCheckpoint s_rts_checkpoints[MAX_RTS_CHECKPOINTS];

static constexpr size_t RTS_BUF_SIZE = RTS_CHUNK_SIZE + 4;
static Vec3 s_chunk_x_filt[RTS_BUF_SIZE][2];
static Vec3 s_chunk_x_pred[RTS_BUF_SIZE][2];
static double s_chunk_P_filt[RTS_BUF_SIZE][3]; // 0: P00, 1: P01, 2: P11
static double s_chunk_P_pred[RTS_BUF_SIZE][3];
static Vec3 s_chunk_x_smooth[RTS_BUF_SIZE][2];

void apply_pv_rts_smoother(Vec3* pos, const int64_t* times, size_t count, double dt_nom) {
    if (count < 15 || dt_nom <= 0.0) {
        return;
    }

    // Check spatial span of trajectory
    double min_x = pos[0].data[0], max_x = pos[0].data[0];
    double min_y = pos[0].data[1], max_y = pos[0].data[1];
    double min_z = pos[0].data[2], max_z = pos[0].data[2];
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(pos[i].data[0]) || !std::isfinite(pos[i].data[1]) || !std::isfinite(pos[i].data[2])) {
            pos[i] = math::make_vec3(0.0, 0.0, 0.0);
        }
        if (pos[i].data[0] < min_x) min_x = pos[i].data[0];
        if (pos[i].data[0] > max_x) max_x = pos[i].data[0];
        if (pos[i].data[1] < min_y) min_y = pos[i].data[1];
        if (pos[i].data[1] > max_y) max_y = pos[i].data[1];
        if (pos[i].data[2] < min_z) min_z = pos[i].data[2];
        if (pos[i].data[2] > max_z) max_z = pos[i].data[2];
    }
    double span_x = max_x - min_x;
    double span_y = max_y - min_y;
    double span_z = max_z - min_z;
    double max_span = std::max({span_x, span_y, span_z});
    if (max_span < params::rts_min_span) {
        // Bypass smoothing for static or degenerate synthetic trajectories
        return;
    }

    const double sigma_a = params::rts_sigma_a;
    const double sigma_a_sq = sigma_a * sigma_a;
    const double R_p = params::rts_r_p;

    // Estimate initial velocity at step 0
    double dt0 = (count > 1) ? static_cast<double>(times[1] - times[0]) * 1e-9 : dt_nom;
    if (dt0 <= 0.0 || dt0 > 1.0) dt0 = dt_nom;
    Vec3 v0 = (pos[1] - pos[0]) / dt0;

    // Checkpoint 0
    s_rts_checkpoints[0].x[0] = pos[0];
    s_rts_checkpoints[0].x[1] = v0;
    s_rts_checkpoints[0].P00 = 1e-4;
    s_rts_checkpoints[0].P01 = 0.0;
    s_rts_checkpoints[0].P11 = 1e-3;

    Vec3 curr_x[2] = { pos[0], v0 };
    double curr_P00 = 1e-4;
    double curr_P01 = 0.0;
    double curr_P11 = 1e-3;

    const size_t num_checkpoints = (count + RTS_CHUNK_SIZE - 1) / RTS_CHUNK_SIZE;

    // Pass 1: Forward propagation with checkpointing every RTS_CHUNK_SIZE steps
    for (size_t k = 0; k < count - 1; ++k) {
        double dt_k = static_cast<double>(times[k + 1] - times[k]) * 1e-9;
        if (dt_k <= 0.0 || dt_k > 1.0) {
            dt_k = dt_nom;
        }

        double dt_sq = dt_k * dt_k;
        double q00 = sigma_a_sq * 0.25 * dt_sq * dt_sq;
        double q01 = sigma_a_sq * 0.5 * dt_sq * dt_k;
        double q11 = sigma_a_sq * dt_sq;

        double P_pred_00 = curr_P00 + 2.0 * dt_k * curr_P01 + dt_sq * curr_P11 + q00;
        double P_pred_01 = curr_P01 + dt_k * curr_P11 + q01;
        double P_pred_11 = curr_P11 + q11;

        Vec3 x_pred_0 = curr_x[0] + curr_x[1] * dt_k;
        Vec3 x_pred_1 = curr_x[1];

        double S = P_pred_00 + R_p;
        double inv_S = 1.0 / S;
        double K0 = P_pred_00 * inv_S;
        double K1 = P_pred_01 * inv_S;

        curr_P00 = (1.0 - K0) * P_pred_00;
        curr_P01 = (1.0 - K0) * P_pred_01;
        curr_P11 = P_pred_11 - K1 * P_pred_01;

        Vec3 innov = pos[k + 1] - x_pred_0;
        curr_x[0] = x_pred_0 + innov * K0;
        curr_x[1] = x_pred_1 + innov * K1;

        if ((k + 1) % RTS_CHUNK_SIZE == 0) {
            size_t idx = (k + 1) / RTS_CHUNK_SIZE;
            if (idx < num_checkpoints && idx < MAX_RTS_CHECKPOINTS) {
                s_rts_checkpoints[idx].x[0] = curr_x[0];
                s_rts_checkpoints[idx].x[1] = curr_x[1];
                s_rts_checkpoints[idx].P00 = curr_P00;
                s_rts_checkpoints[idx].P01 = curr_P01;
                s_rts_checkpoints[idx].P11 = curr_P11;
            }
        }
    }

    // Terminal condition for backward pass
    Vec3 smooth_boundary_x[2] = { curr_x[0], curr_x[1] };
    pos[count - 1] = smooth_boundary_x[0];

    // Pass 2: Chunk-by-chunk backward RTS smoothing
    size_t num_chunks = (count - 1 + RTS_CHUNK_SIZE - 1) / RTS_CHUNK_SIZE;
    for (size_t chunk_idx = num_chunks; chunk_idx > 0; --chunk_idx) {
        size_t chunk = chunk_idx - 1;
        size_t k_start = chunk * RTS_CHUNK_SIZE;
        size_t k_end = std::min((chunk + 1) * RTS_CHUNK_SIZE, count - 1);
        size_t chunk_len = k_end - k_start;

        if (chunk >= MAX_RTS_CHECKPOINTS) {
            continue;
        }

        // Initialize local chunk scratch from checkpoint
        s_chunk_x_filt[0][0] = s_rts_checkpoints[chunk].x[0];
        s_chunk_x_filt[0][1] = s_rts_checkpoints[chunk].x[1];
        s_chunk_P_filt[0][0] = s_rts_checkpoints[chunk].P00;
        s_chunk_P_filt[0][1] = s_rts_checkpoints[chunk].P01;
        s_chunk_P_filt[0][2] = s_rts_checkpoints[chunk].P11;

        // Re-simulate forward for this chunk
        for (size_t i = 0; i < chunk_len; ++i) {
            size_t k = k_start + i;
            double dt_k = static_cast<double>(times[k + 1] - times[k]) * 1e-9;
            if (dt_k <= 0.0 || dt_k > 1.0) {
                dt_k = dt_nom;
            }

            double dt_sq = dt_k * dt_k;
            double q00 = sigma_a_sq * 0.25 * dt_sq * dt_sq;
            double q01 = sigma_a_sq * 0.5 * dt_sq * dt_k;
            double q11 = sigma_a_sq * dt_sq;

            double P_prev_00 = s_chunk_P_filt[i][0];
            double P_prev_01 = s_chunk_P_filt[i][1];
            double P_prev_11 = s_chunk_P_filt[i][2];

            double P_pred_00 = P_prev_00 + 2.0 * dt_k * P_prev_01 + dt_sq * P_prev_11 + q00;
            double P_pred_01 = P_prev_01 + dt_k * P_prev_11 + q01;
            double P_pred_11 = P_prev_11 + q11;

            s_chunk_P_pred[i + 1][0] = P_pred_00;
            s_chunk_P_pred[i + 1][1] = P_pred_01;
            s_chunk_P_pred[i + 1][2] = P_pred_11;

            s_chunk_x_pred[i + 1][0] = s_chunk_x_filt[i][0] + s_chunk_x_filt[i][1] * dt_k;
            s_chunk_x_pred[i + 1][1] = s_chunk_x_filt[i][1];

            double S = P_pred_00 + R_p;
            double inv_S = 1.0 / S;
            double K0 = P_pred_00 * inv_S;
            double K1 = P_pred_01 * inv_S;

            s_chunk_P_filt[i + 1][0] = (1.0 - K0) * P_pred_00;
            s_chunk_P_filt[i + 1][1] = (1.0 - K0) * P_pred_01;
            s_chunk_P_filt[i + 1][2] = P_pred_11 - K1 * P_pred_01;

            Vec3 innov = pos[k + 1] - s_chunk_x_pred[i + 1][0];
            s_chunk_x_filt[i + 1][0] = s_chunk_x_pred[i + 1][0] + innov * K0;
            s_chunk_x_filt[i + 1][1] = s_chunk_x_pred[i + 1][1] + innov * K1;
        }

        // Backward recursion inside chunk
        s_chunk_x_smooth[chunk_len][0] = smooth_boundary_x[0];
        s_chunk_x_smooth[chunk_len][1] = smooth_boundary_x[1];

        for (size_t i_rev = chunk_len; i_rev > 0; --i_rev) {
            size_t i = i_rev - 1;
            size_t k = k_start + i;
            double dt_k = static_cast<double>(times[k + 1] - times[k]) * 1e-9;
            if (dt_k <= 0.0 || dt_k > 1.0) {
                dt_k = dt_nom;
            }

            // P_filt at i * F^T
            double P_f_00 = s_chunk_P_filt[i][0];
            double P_f_01 = s_chunk_P_filt[i][1];
            double P_f_11 = s_chunk_P_filt[i][2];

            double M00 = P_f_00 + dt_k * P_f_01;
            double M01 = P_f_01;
            double M10 = P_f_01 + dt_k * P_f_11;
            double M11 = P_f_11;

            // Invert P_pred at i+1
            double A = s_chunk_P_pred[i + 1][0];
            double B = s_chunk_P_pred[i + 1][1];
            double D = s_chunk_P_pred[i + 1][2];

            double det = A * D - B * B;
            if (det < 1e-15) {
                det = 1e-15;
            }
            double inv_det = 1.0 / det;

            double C00 = (M00 * D - M01 * B) * inv_det;
            double C01 = (-M00 * B + M01 * A) * inv_det;
            double C10 = (M10 * D - M11 * B) * inv_det;
            double C11 = (-M10 * B + M11 * A) * inv_det;

            Vec3 diff_p = s_chunk_x_smooth[i + 1][0] - s_chunk_x_pred[i + 1][0];
            Vec3 diff_v = s_chunk_x_smooth[i + 1][1] - s_chunk_x_pred[i + 1][1];

            s_chunk_x_smooth[i][0] = s_chunk_x_filt[i][0] + diff_p * C00 + diff_v * C01;
            s_chunk_x_smooth[i][1] = s_chunk_x_filt[i][1] + diff_p * C10 + diff_v * C11;

            pos[k] = s_chunk_x_smooth[i][0];
        }

        smooth_boundary_x[0] = s_chunk_x_smooth[0][0];
        smooth_boundary_x[1] = s_chunk_x_smooth[0][1];
    }

    // Ensure strict start at p(0) = 0
    Vec3 p0 = pos[0];
    for (size_t i = 0; i < count; ++i) {
        pos[i] = pos[i] - p0;
    }
}

class DelayedOutputBuffer {
public:
    static constexpr size_t CAPACITY = 512;

    DelayedOutputBuffer() : head_(0), tail_(0), count_(0) {}

    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

    template<typename WriterType>
    void push(int64_t t_ns, const Vec3& p, const Quat& q, bool is_gap, WriterType& writer) {
        if (params::delayed_output_frames <= 0) {
            writer.write_pose(t_ns, p.data[0], p.data[1], p.data[2], q.x, q.y, q.z, q.w);
            return;
        }

        if (count_ >= static_cast<size_t>(params::delayed_output_frames) || count_ >= CAPACITY) {
            const auto& oldest = buffer_[tail_];
            writer.write_pose(oldest.t_ns, oldest.p.data[0], oldest.p.data[1], oldest.p.data[2],
                              oldest.q.x, oldest.q.y, oldest.q.z, oldest.q.w);
            tail_ = (tail_ + 1) % CAPACITY;
            count_--;
        }

        buffer_[head_] = {t_ns, p, q, is_gap};
        head_ = (head_ + 1) % CAPACITY;
        count_++;
    }

    void apply_bridge(const Vec3& p_recovered) {
        if (count_ == 0 || params::delayed_output_frames <= 0) {
            return;
        }
        size_t gap_count = 0;
        for (size_t i = 0; i < count_; ++i) {
            size_t idx = (head_ - 1 - i + CAPACITY) % CAPACITY;
            if (buffer_[idx].is_gap) {
                gap_count++;
            } else {
                break;
            }
        }
        if (gap_count == 0) {
            return;
        }

        size_t last_gap_idx = (head_ - 1 + CAPACITY) % CAPACITY;
        Vec3 delta_p = p_recovered - buffer_[last_gap_idx].p;

        for (size_t step = 0; step < gap_count; ++step) {
            size_t idx = (head_ - gap_count + step + CAPACITY) % CAPACITY;
            double alpha = static_cast<double>(step + 1) / static_cast<double>(gap_count + 1);
            double s = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha;
            buffer_[idx].p = buffer_[idx].p + delta_p * s;
            buffer_[idx].is_gap = false;
        }
    }

    template<typename WriterType>
    void flush(WriterType& writer) {
        while (count_ > 0) {
            const auto& oldest = buffer_[tail_];
            writer.write_pose(oldest.t_ns, oldest.p.data[0], oldest.p.data[1], oldest.p.data[2],
                              oldest.q.x, oldest.q.y, oldest.q.z, oldest.q.w);
            tail_ = (tail_ + 1) % CAPACITY;
            count_--;
        }
    }

private:
    DelayedPose buffer_[CAPACITY];
    size_t head_;
    size_t tail_;
    size_t count_;
};

} // namespace

AblationMode parse_ablation_mode(const char* str) {
    if (!str) {
        return AblationMode::ESKF_FULL;
    }
    if (strcasecmp(str, "naive") == 0) {
        return AblationMode::NAIVE;
    }
    if (strcasecmp(str, "mahony") == 0) {
        return AblationMode::MAHONY;
    }
    if (strcasecmp(str, "eskf_basic") == 0) {
        return AblationMode::ESKF_BASIC;
    }
    if (strcasecmp(str, "eskf_full") == 0) {
        return AblationMode::ESKF_FULL;
    }
    return AblationMode::ESKF_FULL;
}

int run_eskf_full(const char* input_path,
                  const char* output_path,
                  const PipelineOptions& options);

int run_pipeline_with_options(const char* input_path,
                              const char* output_path,
                              const PipelineOptions& options) {
    switch (options.mode) {
        case AblationMode::NAIVE:
            return run_naive(input_path, output_path, options.append_blind);
        case AblationMode::MAHONY:
            return run_mahony(input_path, output_path, options.append_blind);
        case AblationMode::ESKF_BASIC:
            return run_eskf_basic(input_path, output_path, options.append_blind);
        case AblationMode::ESKF_FULL:
        default:
            return run_eskf_full(input_path, output_path, options);
    }
}

int run_pipeline(const char* input_path,
                 const char* output_path,
                 AblationMode mode,
                 bool append_blind) {
    PipelineOptions options;
    options.mode = mode;
    options.append_blind = append_blind;
    return run_pipeline_with_options(input_path, output_path, options);
}

int run_eskf_full(const char* input_path,
                  const char* output_path,
                  bool append_blind) {
    PipelineOptions options;
    options.append_blind = append_blind;
    return run_eskf_full(input_path, output_path, options);
}

// ---------------------------------------------------------------------------
// 1. Naive Baseline (SPEC §11.5, §12.2, §12.3)
// ---------------------------------------------------------------------------
int run_naive(const char* input_path,
              const char* output_path,
              bool append_blind) {
    io::IMUReader reader;
    if (!reader.open(input_path)) {
        std::fprintf(stderr, "Error: failed to open input '%s'\n", input_path);
        return 1;
    }

    io::TrajectoryWriter writer;
    if (!writer.open(output_path)) {
        std::fprintf(stderr, "Error: failed to open output '%s'\n", output_path);
        reader.close();
        return 1;
    }

    io::IMUSample sample;
    size_t init_count = 0;
    bool init_done = false;

    double dt_samples[200];
    size_t dt_count = 0;

    int64_t prev_t_ns = 0;
    Vec3 p = math::make_vec3(0.0, 0.0, 0.0);
    Vec3 v = math::make_vec3(0.0, 0.0, 0.0);
    Quat q = Quat::identity();
    Vec3 g_w = math::make_vec3(0.0, 0.0, -9.80665);
    double g_mag = 9.80665;

    while (reader.next(sample)) {
        Vec3 w_curr = math::make_vec3(sample.wx, sample.wy, sample.wz);
        Vec3 a_curr = math::make_vec3(sample.ax, sample.ay, sample.az);

        if (!init_done) {
            if (init_count < MAX_INIT_SAMPLES) {
                s_init_buf[init_count].t_ns = sample.t_ns;
                s_init_buf[init_count].omega = w_curr;
                s_init_buf[init_count].acc = a_curr;
                s_init_buf[init_count].is_valid = sample.is_valid;
                init_count++;
            }

            int64_t span_ns = sample.t_ns - s_init_buf[0].t_ns;
            if (span_ns >= 1000000000LL) {
                init_done = true;

                // Initialise tilt over samples within 1.0s
                size_t mask_count = 0;
                Vec3 a_sum = math::make_vec3(0.0, 0.0, 0.0);
                for (size_t i = 0; i < init_count; ++i) {
                    if (s_init_buf[i].t_ns - s_init_buf[0].t_ns <= 1000000000LL) {
                        a_sum += s_init_buf[i].acc;
                        mask_count++;
                    } else {
                        break;
                    }
                }
                if (mask_count < 2) {
                    mask_count = (init_count < 20) ? init_count : 20;
                    a_sum = math::make_vec3(0.0, 0.0, 0.0);
                    for (size_t i = 0; i < mask_count; ++i) {
                        a_sum += s_init_buf[i].acc;
                    }
                }
                Vec3 a_init_mean = (mask_count > 0)
                                       ? (a_sum / static_cast<double>(mask_count))
                                       : math::make_vec3(0.0, 0.0, 9.80665);

                q = compute_initial_tilt(a_init_mean);
                double g_raw = a_init_mean.norm();
                g_mag = g_raw;
                if (g_mag < 9.0) g_mag = 9.0;
                if (g_mag > 10.5) g_mag = 10.5;
                g_w = math::make_vec3(0.0, 0.0, -g_mag);

                p = math::make_vec3(0.0, 0.0, 0.0);
                v = math::make_vec3(0.0, 0.0, 0.0);

                // Drain initial buffer
                for (size_t k = 0; k < init_count; ++k) {
                    int64_t t_k = s_init_buf[k].t_ns;
                    if (k == 0) {
                        prev_t_ns = t_k;
                    } else {
                        double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
                        prev_t_ns = t_k;
                        if (dt > 0.0 && dt < 10.0) {
                            if (dt_count < 200) {
                                dt_samples[dt_count++] = dt;
                            }
                            Quat delta_q = math::exp_map(s_init_buf[k].omega * dt);
                            Quat q_new = math::normalize(q * delta_q);
                            math::ensure_continuity(q_new, q);
                            Mat3 R_old = q.to_rot_mat();
                            Vec3 a_w = R_old * s_init_buf[k].acc + g_w;
                            p += v * dt + a_w * (0.5 * dt * dt);
                            v += a_w * dt;
                            q = q_new;
                        }
                    }
                    writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                                      q.x, q.y, q.z, q.w);
                }
            }
            continue;
        }

        // Streaming step once initialized
        int64_t t_k = sample.t_ns;
        double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
        prev_t_ns = t_k;
        if (dt > 0.0 && dt < 10.0 && sample.is_valid) {
            if (dt_count < 200) {
                dt_samples[dt_count++] = dt;
            }
            Quat delta_q = math::exp_map(w_curr * dt);
            Quat q_new = math::normalize(q * delta_q);
            math::ensure_continuity(q_new, q);
            Mat3 R_old = q.to_rot_mat();
            Vec3 a_w = R_old * a_curr + g_w;
            p += v * dt + a_w * (0.5 * dt * dt);
            v += a_w * dt;
            q = q_new;
        }
        writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                          q.x, q.y, q.z, q.w);
    }

    // Fallback if file was shorter than 1.0s
    if (!init_done && init_count > 0) {
        Vec3 a_sum = math::make_vec3(0.0, 0.0, 0.0);
        for (size_t i = 0; i < init_count; ++i) {
            a_sum += s_init_buf[i].acc;
        }
        Vec3 a_init_mean = a_sum / static_cast<double>(init_count);
        q = compute_initial_tilt(a_init_mean);
        double g_raw = a_init_mean.norm();
        g_mag = g_raw;
        if (g_mag < 9.0) g_mag = 9.0;
        if (g_mag > 10.5) g_mag = 10.5;
        g_w = math::make_vec3(0.0, 0.0, -g_mag);
        p = math::make_vec3(0.0, 0.0, 0.0);
        v = math::make_vec3(0.0, 0.0, 0.0);

        for (size_t k = 0; k < init_count; ++k) {
            int64_t t_k = s_init_buf[k].t_ns;
            if (k == 0) {
                prev_t_ns = t_k;
            } else {
                double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
                prev_t_ns = t_k;
                if (dt > 0.0 && dt < 10.0 && s_init_buf[k].is_valid) {
                    if (dt_count < 200) dt_samples[dt_count++] = dt;
                    Quat delta_q = math::exp_map(s_init_buf[k].omega * dt);
                    Quat q_new = math::normalize(q * delta_q);
                    math::ensure_continuity(q_new, q);
                    Mat3 R_old = q.to_rot_mat();
                    Vec3 a_w = R_old * s_init_buf[k].acc + g_w;
                    p += v * dt + a_w * (0.5 * dt * dt);
                    v += a_w * dt;
                    q = q_new;
                }
            }
            if (!std::isfinite(p.data[0]) || !std::isfinite(p.data[1]) || !std::isfinite(p.data[2])) {
                p = math::make_vec3(0.0, 0.0, 0.0);
            }
            if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
                q = Quat::identity();
            }
            writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                              q.x, q.y, q.z, q.w);
        }
    }

    if (append_blind && prev_t_ns > 0) {
        double median_dt = compute_dt_nom(dt_samples, dt_count, 0.005);
        int blind_steps = static_cast<int>(std::round(5.0 / median_dt));
        int64_t dt_ns_step = std::max(static_cast<int64_t>(1), static_cast<int64_t>(std::round(median_dt * 1e9)));
        for (int step = 1; step <= blind_steps; ++step) {
            int64_t t_extrap = prev_t_ns + step * dt_ns_step;
            Vec3 p_extrap = p + v * (step * median_dt);
            writer.write_pose(t_extrap, p_extrap.data[0], p_extrap.data[1], p_extrap.data[2],
                              q.x, q.y, q.z, q.w);
        }
    }

    writer.close();
    reader.close();
    return 0;
}

// ---------------------------------------------------------------------------
// 2. Mahony Baseline (SPEC §11.5)
// ---------------------------------------------------------------------------
int run_mahony(const char* input_path,
               const char* output_path,
               bool append_blind) {
    io::IMUReader reader;
    if (!reader.open(input_path)) {
        std::fprintf(stderr, "Error: failed to open input '%s'\n", input_path);
        return 1;
    }

    io::TrajectoryWriter writer;
    if (!writer.open(output_path)) {
        std::fprintf(stderr, "Error: failed to open output '%s'\n", output_path);
        reader.close();
        return 1;
    }

    io::IMUSample sample;
    size_t init_count = 0;
    bool init_done = false;

    double dt_samples[200];
    size_t dt_count = 0;

    int64_t prev_t_ns = 0;
    Vec3 p = math::make_vec3(0.0, 0.0, 0.0);
    Vec3 v = math::make_vec3(0.0, 0.0, 0.0);
    Quat q = Quat::identity();
    Vec3 g_w = math::make_vec3(0.0, 0.0, -9.80665);
    double g_mag = 9.80665;
    Vec3 e_int = math::make_vec3(0.0, 0.0, 0.0);

    const double kp = 1.0;
    const double ki = 0.05;

    while (reader.next(sample)) {
        Vec3 w_curr = math::make_vec3(sample.wx, sample.wy, sample.wz);
        Vec3 a_curr = math::make_vec3(sample.ax, sample.ay, sample.az);

        if (!init_done) {
            if (init_count < MAX_INIT_SAMPLES) {
                s_init_buf[init_count].t_ns = sample.t_ns;
                s_init_buf[init_count].omega = w_curr;
                s_init_buf[init_count].acc = a_curr;
                s_init_buf[init_count].is_valid = sample.is_valid;
                init_count++;
            }

            int64_t span_ns = sample.t_ns - s_init_buf[0].t_ns;
            if (span_ns >= 1000000000LL) {
                init_done = true;

                size_t mask_count = 0;
                Vec3 a_sum = math::make_vec3(0.0, 0.0, 0.0);
                for (size_t i = 0; i < init_count; ++i) {
                    if (s_init_buf[i].t_ns - s_init_buf[0].t_ns <= 1000000000LL) {
                        a_sum += s_init_buf[i].acc;
                        mask_count++;
                    } else {
                        break;
                    }
                }
                if (mask_count < 2) {
                    mask_count = (init_count < 20) ? init_count : 20;
                    a_sum = math::make_vec3(0.0, 0.0, 0.0);
                    for (size_t i = 0; i < mask_count; ++i) {
                        a_sum += s_init_buf[i].acc;
                    }
                }
                Vec3 a_init_mean = (mask_count > 0)
                                       ? (a_sum / static_cast<double>(mask_count))
                                       : math::make_vec3(0.0, 0.0, 9.80665);

                q = compute_initial_tilt(a_init_mean);
                double g_raw = a_init_mean.norm();
                g_mag = g_raw;
                if (g_mag < 9.0) g_mag = 9.0;
                if (g_mag > 10.5) g_mag = 10.5;
                g_w = math::make_vec3(0.0, 0.0, -g_mag);

                p = math::make_vec3(0.0, 0.0, 0.0);
                v = math::make_vec3(0.0, 0.0, 0.0);
                e_int = math::make_vec3(0.0, 0.0, 0.0);

                for (size_t k = 0; k < init_count; ++k) {
                    int64_t t_k = s_init_buf[k].t_ns;
                    if (k == 0) {
                        prev_t_ns = t_k;
                    } else {
                        double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
                        prev_t_ns = t_k;
                        if (dt > 0.0 && dt < 10.0) {
                            if (dt_count < 200) {
                                dt_samples[dt_count++] = dt;
                            }
                            Mat3 R_mat = q.to_rot_mat();
                            Vec3 v_b = math::make_vec3(R_mat(2, 0), R_mat(2, 1), R_mat(2, 2));

                            double a_norm = s_init_buf[k].acc.norm();
                            Vec3 e = math::make_vec3(0.0, 0.0, 0.0);
                            if (a_norm > 1e-3 && std::abs(a_norm - g_mag) < 3.0) {
                                Vec3 a_b = s_init_buf[k].acc / a_norm;
                                e = math::cross(a_b, v_b);
                            }
                            e_int += e * dt;
                            Vec3 w_corr = s_init_buf[k].omega + e * kp + e_int * ki;

                            Quat delta_q = math::exp_map(w_corr * dt);
                            Quat q_new = math::normalize(q * delta_q);
                            math::ensure_continuity(q_new, q);

                            Vec3 a_w = R_mat * s_init_buf[k].acc + g_w;
                            p += v * dt + a_w * (0.5 * dt * dt);
                            v += a_w * dt;
                            q = q_new;
                        }
                    }
                    writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                                      q.x, q.y, q.z, q.w);
                }
            }
            continue;
        }

        int64_t t_k = sample.t_ns;
        double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
        prev_t_ns = t_k;
        if (dt > 0.0 && dt < 10.0) {
            if (dt_count < 200) {
                dt_samples[dt_count++] = dt;
            }
            Mat3 R_mat = q.to_rot_mat();
            Vec3 v_b = math::make_vec3(R_mat(2, 0), R_mat(2, 1), R_mat(2, 2));

            double a_norm = a_curr.norm();
            Vec3 e = math::make_vec3(0.0, 0.0, 0.0);
            if (a_norm > 1e-3 && std::abs(a_norm - g_mag) < 3.0) {
                Vec3 a_b = a_curr / a_norm;
                e = math::cross(a_b, v_b);
            }
            e_int += e * dt;
            Vec3 w_corr = w_curr + e * kp + e_int * ki;

            Quat delta_q = math::exp_map(w_corr * dt);
            Quat q_new = math::normalize(q * delta_q);
            math::ensure_continuity(q_new, q);

            Vec3 a_w = R_mat * a_curr + g_w;
            p += v * dt + a_w * (0.5 * dt * dt);
            v += a_w * dt;
            q = q_new;
        }
        writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                          q.x, q.y, q.z, q.w);
    }

    if (!init_done && init_count > 0) {
        Vec3 a_sum = math::make_vec3(0.0, 0.0, 0.0);
        for (size_t i = 0; i < init_count; ++i) {
            a_sum += s_init_buf[i].acc;
        }
        Vec3 a_init_mean = a_sum / static_cast<double>(init_count);
        q = compute_initial_tilt(a_init_mean);
        double g_raw = a_init_mean.norm();
        g_mag = g_raw;
        if (g_mag < 9.0) g_mag = 9.0;
        if (g_mag > 10.5) g_mag = 10.5;
        g_w = math::make_vec3(0.0, 0.0, -g_mag);
        p = math::make_vec3(0.0, 0.0, 0.0);
        v = math::make_vec3(0.0, 0.0, 0.0);

        for (size_t k = 0; k < init_count; ++k) {
            int64_t t_k = s_init_buf[k].t_ns;
            if (k == 0) {
                prev_t_ns = t_k;
            } else {
                double dt = static_cast<double>(t_k - prev_t_ns) * 1e-9;
                prev_t_ns = t_k;
                if (dt > 0.0 && dt < 10.0 && s_init_buf[k].is_valid) {
                    if (dt_count < 200) dt_samples[dt_count++] = dt;
                    Quat delta_q = math::exp_map(s_init_buf[k].omega * dt);
                    Quat q_new = math::normalize(q * delta_q);
                    math::ensure_continuity(q_new, q);
                    Mat3 R_old = q.to_rot_mat();
                    Vec3 a_w = R_old * s_init_buf[k].acc + g_w;
                    p += v * dt + a_w * (0.5 * dt * dt);
                    v += a_w * dt;
                    q = q_new;
                }
            }
            if (!std::isfinite(p.data[0]) || !std::isfinite(p.data[1]) || !std::isfinite(p.data[2])) {
                p = math::make_vec3(0.0, 0.0, 0.0);
            }
            if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
                q = Quat::identity();
            }
            writer.write_pose(t_k, p.data[0], p.data[1], p.data[2],
                              q.x, q.y, q.z, q.w);
        }
    }

    if (append_blind && prev_t_ns > 0) {
        double median_dt = compute_dt_nom(dt_samples, dt_count, 0.005);
        int blind_steps = static_cast<int>(std::round(5.0 / median_dt));
        int64_t dt_ns_step = std::max(static_cast<int64_t>(1), static_cast<int64_t>(std::round(median_dt * 1e9)));
        for (int step = 1; step <= blind_steps; ++step) {
            int64_t t_extrap = prev_t_ns + step * dt_ns_step;
            Vec3 p_extrap = p + v * (step * median_dt);
            writer.write_pose(t_extrap, p_extrap.data[0], p_extrap.data[1], p_extrap.data[2],
                              q.x, q.y, q.z, q.w);
        }
    }

    writer.close();
    reader.close();
    return 0;
}

// ---------------------------------------------------------------------------
// 3. ESKF Basic Ablation (SPEC §11.5)
// ---------------------------------------------------------------------------
int run_eskf_basic(const char* input_path,
                   const char* output_path,
                   bool append_blind) {
    io::IMUReader reader;
    if (!reader.open(input_path)) {
        std::fprintf(stderr, "Error: failed to open input '%s'\n", input_path);
        return 1;
    }

    io::TrajectoryWriter writer;
    if (!writer.open(output_path)) {
        std::fprintf(stderr, "Error: failed to open output '%s'\n", output_path);
        reader.close();
        return 1;
    }

    Initializer initializer;
    StationaryBuffer stat_buf;

    io::IMUSample sample;
    size_t init_count = 0;
    bool is_initialized = false;

    NominalState state = NominalState::zeros();
    Mat15 P = Mat15::identity();
    Vec3 g_w = math::make_vec3(0.0, 0.0, -9.80665);
    double g_mag = 9.80665;
    int64_t t0_ns = 0;
    int64_t prev_t_ns = 0;

    double dt_samples[200];
    size_t dt_count = 0;

    while (reader.next(sample)) {
        Vec3 w_curr = math::make_vec3(sample.wx, sample.wy, sample.wz);
        Vec3 acc_curr = math::make_vec3(sample.ax, sample.ay, sample.az);

        if (!is_initialized) {
            if (init_count < MAX_INIT_SAMPLES) {
                s_init_buf[init_count].t_ns = sample.t_ns;
                s_init_buf[init_count].omega = w_curr;
                s_init_buf[init_count].acc = acc_curr;
                s_init_buf[init_count].is_valid = sample.is_valid;
                init_count++;
            }

            if (sample.is_valid) {
                initializer.add_sample(sample.t_ns, w_curr, acc_curr);
            }

            if (initializer.is_ready()) {
                is_initialized = true;
                initializer.finish(state, P, g_w);
                g_mag = initializer.g_mag();
                t0_ns = s_init_buf[0].t_ns;

                for (size_t idx = 0; idx < init_count; ++idx) {
                    int64_t t_smp = s_init_buf[idx].t_ns;
                    Vec3 w_smp = s_init_buf[idx].omega;
                    Vec3 a_smp = s_init_buf[idx].acc;
                    bool v_smp = s_init_buf[idx].is_valid;

                    if (idx == 0) {
                        prev_t_ns = t_smp;
                        writer.write_pose(t_smp, state.p.data[0], state.p.data[1], state.p.data[2],
                                          state.q.x, state.q.y, state.q.z, state.q.w);
                        continue;
                    }

                    double dt = static_cast<double>(t_smp - prev_t_ns) * 1e-9;
                    prev_t_ns = t_smp;

                    if (dt <= 0.0 || !v_smp) {
                        // Skip integration
                    } else {
                        if (dt_count < 200) {
                            dt_samples[dt_count++] = dt;
                        }
                        state = strapdown_step(state, w_smp, a_smp, dt, g_w);
                        Mat15 F = build_f_matrix(state, w_smp, a_smp, dt);
                        Mat15 Q = build_q_matrix(dt, params::noise_mult_gyro, params::noise_mult_acc,
                                                 params::walk_mult_bg, params::walk_mult_ba, 1.0);
                        P = propagate_covariance(P, F, Q);

                        double t_sec = static_cast<double>(t_smp - t0_ns) * 1e-9;
                        stat_buf.add_sample(t_sec, w_smp, a_smp, state.b_g);
                        bool is_stat = stat_buf.is_stationary(g_mag, 1.0, params::zupt_window_s);

                        apply_tilt_update(state, P, a_smp, w_smp, 1.0, g_w, 1e9, 1.0);
                        if (is_stat) {
                            apply_zupt_update(state, P, 1e9, 1.0);
                        }
                    }

                    writer.write_pose(t_smp, state.p.data[0], state.p.data[1], state.p.data[2],
                                      state.q.x, state.q.y, state.q.z, state.q.w);
                }
            }
            continue;
        }

        // Streaming step once initialized
        int64_t t_curr = sample.t_ns;
        double dt = static_cast<double>(t_curr - prev_t_ns) * 1e-9;
        prev_t_ns = t_curr;

        if (dt <= 0.0 || !sample.is_valid) {
            // Skip integration
        } else {
            if (dt_count < 200) {
                dt_samples[dt_count++] = dt;
            }
            state = strapdown_step(state, w_curr, acc_curr, dt, g_w);
            Mat15 F = build_f_matrix(state, w_curr, acc_curr, dt);
            Mat15 Q = build_q_matrix(dt, params::noise_mult_gyro, params::noise_mult_acc,
                                     params::walk_mult_bg, params::walk_mult_ba, 1.0);
            P = propagate_covariance(P, F, Q);

            double t_sec = static_cast<double>(t_curr - t0_ns) * 1e-9;
            stat_buf.add_sample(t_sec, w_curr, acc_curr, state.b_g);
            bool is_stat = stat_buf.is_stationary(g_mag, 1.0, params::zupt_window_s);

            apply_tilt_update(state, P, acc_curr, w_curr, 1.0, g_w, 1e9, 1.0);
            if (is_stat) {
                apply_zupt_update(state, P, 1e9, 1.0);
            }
        }

        writer.write_pose(t_curr, state.p.data[0], state.p.data[1], state.p.data[2],
                          state.q.x, state.q.y, state.q.z, state.q.w);
    }

    // Fallback if file was shorter than T_init
    if (!is_initialized && init_count > 0) {
        initializer.finish(state, P, g_w);
        g_mag = initializer.g_mag();

        for (size_t idx = 0; idx < init_count; ++idx) {
            int64_t t_smp = s_init_buf[idx].t_ns;
            if (idx == 0) {
                prev_t_ns = t_smp;
            } else {
                double dt = static_cast<double>(t_smp - prev_t_ns) * 1e-9;
                prev_t_ns = t_smp;
                if (dt > 0.0 && s_init_buf[idx].is_valid) {
                    state = strapdown_step(state, s_init_buf[idx].omega, s_init_buf[idx].acc, dt, g_w);
                }
            }
            if (!std::isfinite(state.p.data[0]) || !std::isfinite(state.p.data[1]) || !std::isfinite(state.p.data[2])) {
                state.p = math::make_vec3(0.0, 0.0, 0.0);
            }
            if (!std::isfinite(state.q.w) || !std::isfinite(state.q.x) || !std::isfinite(state.q.y) || !std::isfinite(state.q.z)) {
                state.q = Quat::identity();
            }
            writer.write_pose(t_smp, state.p.data[0], state.p.data[1], state.p.data[2],
                              state.q.x, state.q.y, state.q.z, state.q.w);
        }
    }

    // Append blind (naive constant velocity)
    if (append_blind && prev_t_ns > 0) {
        double dt_nom = compute_dt_nom(dt_samples, dt_count, 0.005);
        int64_t dt_ns_step = std::max(static_cast<int64_t>(1), static_cast<int64_t>(std::round(dt_nom * 1e9)));
        int blind_steps = static_cast<int>(std::round(5.0 / dt_nom));
        for (int step = 1; step <= blind_steps; ++step) {
            int64_t t_extrap = prev_t_ns + step * dt_ns_step;
            Vec3 p_extrap = state.p + state.v * (step * dt_nom);
            writer.write_pose(t_extrap, p_extrap.data[0], p_extrap.data[1], p_extrap.data[2],
                              state.q.x, state.q.y, state.q.z, state.q.w);
        }
    }

    writer.close();
    reader.close();
    return 0;
}

// ---------------------------------------------------------------------------
// 4. Full 15-State ESKF Pipeline (SPEC §11.5, §12)
// ---------------------------------------------------------------------------
int run_eskf_full(const char* input_path,
                  const char* output_path,
                  const PipelineOptions& options) {
    bool append_blind = options.append_blind;
    io::IMUReader reader;
    if (!reader.open(input_path)) {
        std::fprintf(stderr, "Error: failed to open input '%s'\n", input_path);
        return 1;
    }

    io::TrajectoryWriter writer;
    if (!writer.open(output_path)) {
        std::fprintf(stderr, "Error: failed to open output '%s'\n", output_path);
        reader.close();
        return 1;
    }

#ifdef ENABLE_ORACLES
    FILE* tel_fp = nullptr;
    if (options.telemetry_path) {
        tel_fp = std::fopen(options.telemetry_path, "w");
        if (tel_fp) {
            std::fprintf(tel_fp, "t_ns,t_sec,px,py,pz,vx,vy,vz,qx,qy,qz,qw,bax,bay,baz,bgx,bgy,bgz,is_stat,tilt_applied,vel_bounded,kappa,quality\n");
        }
    }

    GtTrajectory gt_traj;
    if (options.oracle_gt_path) {
        gt_traj.load(options.oracle_gt_path);
    }
    Mat3 R_gt_to_est = Mat3::identity();
    bool gt_init_done = false;
#endif

    Initializer initializer;
    GapHandler gap_handler;
    DegradationMonitor monitor;
    FailureDetector blind_predictor;
    StationaryBuffer stat_buf;
    Butterworth2LowPass tilt_filter;
    DelayedOutputBuffer delay_buf;
    bool was_in_gap = false;

    io::IMUSample sample;
    size_t init_count = 0;
    bool is_initialized = false;

    NominalState state = NominalState::zeros();
    Mat15 P = Mat15::identity();
    Vec3 g_w = math::make_vec3(0.0, 0.0, -9.80665);
    double g_mag = 9.80665;
    int64_t t0_ns = 0;
    int64_t prev_t_ns = 0;
    double q_gyro_mult = params::noise_mult_gyro;
    double q_acc_mult = params::noise_mult_acc;

    TrajectoryCollector collector;
    bool enable_two_pass = false;
    Vec3 refined_bg = math::make_vec3(0.0, 0.0, 0.0);
    Vec3 refined_ba = math::make_vec3(0.0, 0.0, 0.0);
    bool has_refined_bias = false;
    double dt_nom = 0.005;
    size_t stream_count = 0;
    double vibe_ema = 0.0;
    double ped_energy = 0.0;
    double dyn_acc_ema = 0.0;
    Vec3 prev_acc_raw = math::make_vec3(0.0, 0.0, 0.0);
    bool has_prev_acc = false;

    for (int pass = 1; pass <= (enable_two_pass ? 2 : 1); ++pass) {
        bool is_pass1 = (enable_two_pass && pass == 1);
        collector.active = !is_pass1;

        auto process_full_step = [&](int64_t t_curr,
                                     const Vec3& omega_curr,
                                     const Vec3& acc_curr) {
        if (blind_predictor.is_failed()) {
            Vec3 p_blind;
            Quat q_blind;
            blind_predictor.predict(t_curr, p_blind, q_blind);
            delay_buf.push(t_curr, p_blind, q_blind, false, collector);
            prev_t_ns = t_curr;
            return;
        }

        GapStepResult step_res = gap_handler.process_sample(t_curr, omega_curr, acc_curr, state, P);

        if (step_res.gap_case == GapCase::ZERO_OR_NEGATIVE_DT) {
            delay_buf.push(t_curr, state.p, state.q, false, collector);
            return;
        }

        if (step_res.gap_case == GapCase::GAP || step_res.gap_case == GapCase::INVALID_SAMPLE) {
            state = step_res.state;
            P = step_res.P;
            was_in_gap = true;
            bool failed = blind_predictor.update(t_curr, false, state.p, state.v, state.q, gap_handler.omega_s());
            Vec3 p_out = state.p;
            Quat q_out = state.q;
            if (failed || blind_predictor.is_failed()) {
                blind_predictor.predict(t_curr, p_out, q_out);
            }
            delay_buf.push(t_curr, p_out, q_out, true, collector);
            prev_t_ns = t_curr;
            return;
        }

        // Normal step
        double dt = static_cast<double>(t_curr - prev_t_ns) * 1e-9;
        prev_t_ns = t_curr;

        bool acc_finite = std::isfinite(acc_curr(0)) && std::isfinite(acc_curr(1)) && std::isfinite(acc_curr(2));
        if (acc_finite) {
            if (!has_prev_acc) {
                prev_acc_raw = acc_curr;
                has_prev_acc = true;
            } else {
                double d_a = (acc_curr - prev_acc_raw).norm();
                if (std::isfinite(d_a)) {
                    vibe_ema = (1.0 - 0.02) * vibe_ema + 0.02 * d_a;
                    double dev_a = std::abs(acc_curr.norm() - g_mag);
                    ped_energy = (1.0 - 0.01) * ped_energy + 0.01 * (dev_a * dev_a);
                    if (monitor.quality() >= 0.5 && dev_a < 12.0) {
                        double a_excess = std::max(0.0, acc_curr.norm() - g_mag);
                        dyn_acc_ema = (1.0 - 0.05) * dyn_acc_ema + 0.05 * a_excess;
                    }
                    prev_acc_raw = acc_curr;
                }
            }
        }
        bool is_flight = (vibe_ema > params::flight_vibe_thresh);
        bool is_pedestrian = (!is_flight && ped_energy > params::ped_energy_thresh);
        double dynamic_envelope = params::max_velocity_mps + (is_flight ? 1.0 : 0.0) * dyn_acc_ema;
        if (dynamic_envelope > 15.0) dynamic_envelope = 15.0;
        double max_v = is_flight ? dynamic_envelope : (is_pedestrian ? params::ped_max_velocity_mps : dynamic_envelope);

#ifdef ENABLE_ORACLES
        double cur_tau_ou = options.disable_tau_ou ? 0.0 : params::tau_ou;
#else
        double cur_tau_ou = params::tau_ou;
#endif
        state = strapdown_step_midpoint(state, omega_curr, acc_curr, dt, g_w, cur_tau_ou);
        Mat15 F = build_f_matrix(state, omega_curr, acc_curr, dt, cur_tau_ou);
        Mat15 Q = build_q_matrix(dt, q_gyro_mult, q_acc_mult,
                                 params::walk_mult_bg, params::walk_mult_ba, monitor.kappa());
        P = propagate_covariance(P, F, Q);

        double t_sec = static_cast<double>(t_curr - t0_ns) * 1e-9;
        Vec3 acc_filt = params::enable_iir_filter ? tilt_filter.step(acc_curr) : acc_curr;

        stat_buf.add_sample(t_sec, omega_curr, acc_curr, state.b_g);
        bool is_stat = stat_buf.is_stationary(g_mag, monitor.quality(), params::zupt_window_s);
        Vec3 omega_stat_mean = stat_buf.get_mean_gyro(params::zupt_window_s);
        monitor.step(t_sec, omega_curr, acc_curr, is_stat, is_stat ? &omega_stat_mean : nullptr);

        bool tilt_applied = false;
        bool vel_bounded = false;
        if (step_res.apply_updates) {
#ifdef ENABLE_ORACLES
            if (!options.disable_tilt) {
                tilt_applied = apply_tilt_update(state, P, acc_filt, omega_curr, monitor.quality(), g_w, 1e9, step_res.r_scale);
            }
            if (is_stat && !options.disable_zupt) {
                apply_zupt_update(state, P, 1e9, step_res.r_scale);
            } else if (is_flight && params::enable_rotor_drag) {
                apply_rotor_drag_update(state, P, acc_curr, omega_curr, g_w, params::rotor_drag_kd, params::rotor_drag_sigma, params::rotor_drag_gate);
            } else if (is_pedestrian && params::enable_pedestrian_prior) {
                apply_pedestrian_prior(state, P, params::ped_sigma_lat, params::ped_sigma_vert, params::ped_gate_threshold);
            }
            if (!options.disable_vel_bound) {
                vel_bounded = apply_velocity_bounding(state, P, max_v);
            }
#else
            tilt_applied = apply_tilt_update(state, P, acc_filt, omega_curr, monitor.quality(), g_w, 1e9, step_res.r_scale);
            if (is_stat) {
                apply_zupt_update(state, P, 1e9, step_res.r_scale);
            } else if (is_flight && params::enable_rotor_drag) {
                apply_rotor_drag_update(state, P, acc_curr, omega_curr, g_w, params::rotor_drag_kd, params::rotor_drag_sigma, params::rotor_drag_gate);
            } else if (is_pedestrian && params::enable_pedestrian_prior) {
                apply_pedestrian_prior(state, P, params::ped_sigma_lat, params::ped_sigma_vert, params::ped_gate_threshold);
            }
            vel_bounded = apply_velocity_bounding(state, P, max_v);
#endif
        }

#ifdef ENABLE_ORACLES
        if (options.oracle_att && gt_traj.loaded) {
            Vec3 p_gt;
            Quat q_gt;
            if (gt_traj.get_pose(t_curr, p_gt, q_gt)) {
                Mat3 R_gt_w = R_gt_to_est * q_gt.to_rot_mat();
                Quat q_new = math::from_rot_mat(R_gt_w);
                math::ensure_continuity(q_new, state.q);
                state.q = q_new;
            }
        } else if (options.oracle_yaw && gt_traj.loaded) {
            Vec3 p_gt;
            Quat q_gt;
            if (gt_traj.get_pose(t_curr, p_gt, q_gt)) {
                Mat3 R_gt_w = R_gt_to_est * q_gt.to_rot_mat();
                double psi_gt = std::atan2(R_gt_w.data[3], R_gt_w.data[0]);

                Mat3 R_est = state.q.to_rot_mat();
                double r20 = R_est.data[6];
                if (r20 < -1.0) r20 = -1.0;
                if (r20 > 1.0) r20 = 1.0;
                double theta_est = -std::asin(r20);
                double phi_est = std::atan2(R_est.data[7], R_est.data[8]);

                double c_psi = std::cos(psi_gt), s_psi = std::sin(psi_gt);
                double c_th = std::cos(theta_est), s_th = std::sin(theta_est);
                double c_ph = std::cos(phi_est), s_ph = std::sin(phi_est);

                Mat3 R_new;
                R_new.data[0] = c_psi * c_th;
                R_new.data[1] = c_psi * s_th * s_ph - s_psi * c_ph;
                R_new.data[2] = c_psi * s_th * c_ph + s_psi * s_ph;

                R_new.data[3] = s_psi * c_th;
                R_new.data[4] = s_psi * s_th * s_ph + c_psi * c_ph;
                R_new.data[5] = s_psi * s_th * c_ph - c_psi * s_ph;

                R_new.data[6] = -s_th;
                R_new.data[7] = c_th * s_ph;
                R_new.data[8] = c_th * c_ph;

                Quat q_new = math::from_rot_mat(R_new);
                math::ensure_continuity(q_new, state.q);
                state.q = q_new;
            }
        }

        if (tel_fp) {
            std::fprintf(tel_fp, "%" PRId64 ",%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%.4f,%.4f\n",
                         t_curr, t_sec,
                         state.p.data[0], state.p.data[1], state.p.data[2],
                         state.v.data[0], state.v.data[1], state.v.data[2],
                         state.q.x, state.q.y, state.q.z, state.q.w,
                         state.b_a.data[0], state.b_a.data[1], state.b_a.data[2],
                         state.b_g.data[0], state.b_g.data[1], state.b_g.data[2],
                         is_stat ? 1 : 0,
                         tilt_applied ? 1 : 0,
                         vel_bounded ? 1 : 0,
                         monitor.kappa(),
                         monitor.quality());
        }
#else
        (void)tilt_applied;
        (void)vel_bounded;
#endif

        if (was_in_gap) {
            delay_buf.apply_bridge(state.p);
            was_in_gap = false;
        }

        blind_predictor.update(t_curr, true, state.p, state.v, state.q, gap_handler.omega_s());
        delay_buf.push(t_curr, state.p, state.q, false, collector);
    };

    while (reader.next(sample)) {
        Vec3 w_curr = math::make_vec3(sample.wx, sample.wy, sample.wz);
        Vec3 acc_curr = math::make_vec3(sample.ax, sample.ay, sample.az);

        if (!is_initialized) {
            if (init_count < MAX_INIT_SAMPLES) {
                s_init_buf[init_count].t_ns = sample.t_ns;
                s_init_buf[init_count].omega = w_curr;
                s_init_buf[init_count].acc = acc_curr;
                s_init_buf[init_count].is_valid = sample.is_valid;
                init_count++;
            }

            if (sample.is_valid) {
                initializer.add_sample(sample.t_ns, w_curr, acc_curr);
            }

            if (initializer.is_ready() || (!is_initialized && init_count >= MAX_INIT_SAMPLES)) {
                is_initialized = true;
                initializer.finish(state, P, g_w);
                g_mag = initializer.g_mag();
                t0_ns = s_init_buf[0].t_ns;
                tilt_filter.reset(initializer.a_mean());

#ifdef ENABLE_ORACLES
                if (gt_traj.loaded && !gt_init_done) {
                    Vec3 p_gt0;
                    Quat q_gt0;
                    if (gt_traj.get_pose(t0_ns, p_gt0, q_gt0)) {
                        R_gt_to_est = state.q.to_rot_mat() * q_gt0.to_rot_mat().transpose();
                        gt_init_done = true;
                        if (options.oracle_v0) {
                            Vec3 v_gt0;
                            if (gt_traj.get_velocity(t0_ns, v_gt0)) {
                                state.v = R_gt_to_est * v_gt0;
                            }
                        }
                    }
                }
                if (options.oracle_bias) {
                    Vec3 g_body = state.q.to_rot_mat().transpose() * g_w;
                    state.b_a = initializer.a_mean() + g_body;
                    for (size_t i = 0; i < 3; ++i) {
                        P(9 + i, 9 + i) = 1e-12;
                        P(12 + i, 12 + i) = 1e-12;
                    }
                }
#endif

                double sigma_w_init = initializer.gyro_noise_std();
                double sigma_a_init = initializer.acc_noise_std();
                double scale_g = (sigma_w_init > 1e-5) ? (sigma_w_init / 0.020) : 0.2;
                double scale_a = (sigma_a_init > 1e-5) ? (sigma_a_init / 0.300) : 0.2;
                if (scale_g < 0.2) scale_g = 0.2;
                if (scale_g > 1.0) scale_g = 1.0;
                if (scale_a < 0.2) scale_a = 0.2;
                if (scale_a > 1.0) scale_a = 1.0;
                q_gyro_mult = params::noise_mult_gyro * scale_g;
                q_acc_mult = params::noise_mult_acc * scale_a;

                if (has_refined_bias) {
                    state.b_g = refined_bg;
                    state.b_a = refined_ba;
                    for (size_t i = 0; i < 3; ++i) {
                        P(9 + i, 9 + i) = 1e-6;
                        P(12 + i, 12 + i) = 1e-4;
                    }
                }

                for (size_t idx = 0; idx < init_count; ++idx) {
                    int64_t t_smp = s_init_buf[idx].t_ns;
                    Vec3 w_smp = s_init_buf[idx].omega;
                    Vec3 a_smp = s_init_buf[idx].acc;

                    if (idx == 0) {
                        prev_t_ns = t_smp;
                        blind_predictor.record_reliable_state(t_smp, state.p, state.v, state.q,
                                                              math::make_vec3(0.0, 0.0, 0.0));
                        gap_handler.process_sample(t_smp, w_smp, a_smp, state, P);

                        delay_buf.push(t_smp, state.p, state.q, false, collector);
                        continue;
                    }

                    process_full_step(t_smp, w_smp, a_smp);
                }
            }
            continue;
        }

        process_full_step(sample.t_ns, w_curr, acc_curr);
    }

    // Fallback if file was shorter than T_init
    if (!is_initialized && init_count > 0) {
        initializer.finish(state, P, g_w);
        g_mag = initializer.g_mag();

        for (size_t idx = 0; idx < init_count; ++idx) {
            int64_t t_smp = s_init_buf[idx].t_ns;
            if (idx == 0) {
                prev_t_ns = t_smp;
            } else {
                double dt = static_cast<double>(t_smp - prev_t_ns) * 1e-9;
                prev_t_ns = t_smp;
                if (dt > 0.0 && s_init_buf[idx].is_valid) {
                    state = strapdown_step(state, s_init_buf[idx].omega, s_init_buf[idx].acc, dt, g_w);
                }
            }
            if (!std::isfinite(state.p.data[0]) || !std::isfinite(state.p.data[1]) || !std::isfinite(state.p.data[2])) {
                state.p = math::make_vec3(0.0, 0.0, 0.0);
            }
            if (!std::isfinite(state.q.w) || !std::isfinite(state.q.x) || !std::isfinite(state.q.y) || !std::isfinite(state.q.z)) {
                state.q = Quat::identity();
            }
            delay_buf.push(t_smp, state.p, state.q, false, collector);
        }
    }

    delay_buf.flush(collector);

    if (is_pass1) {
        refined_bg = state.b_g;
        refined_ba = state.b_a;
        if (std::isfinite(refined_bg.norm()) && refined_bg.norm() < 0.2 &&
            std::isfinite(refined_ba.norm()) && refined_ba.norm() < 0.5) {
            has_refined_bias = true;
        }
        reader.rewind();
        initializer.reset();
        gap_handler.reset();
        monitor.reset();
        blind_predictor.reset();
        stat_buf.clear();
        tilt_filter = Butterworth2LowPass();
        delay_buf.clear();
        collector.reset();
        init_count = 0;
        is_initialized = false;
        was_in_gap = false;
        state = NominalState::zeros();
        P = Mat15::identity();
        t0_ns = 0;
        prev_t_ns = 0;
        continue;
    }

    stream_count = collector.count;
    dt_nom = (gap_handler.dt_nom() > 0.0) ? gap_handler.dt_nom() : 0.005;
    break;
    } // end for pass

    // Determine if IMU failure occurred during the stream
    size_t active_count = stream_count;
    Vec3 p_raw_split = math::make_vec3(0.0, 0.0, 0.0);
    bool failed_in_stream = false;

    if (blind_predictor.is_failed() && stream_count > 0 && collector.tmp_file) {
        int64_t t_fail = blind_predictor.failure_time_ns();
        std::rewind(collector.tmp_file);
        for (size_t i = 0; i < stream_count; ++i) {
            TrajectoryCollector::Record rec;
            if (std::fread(&rec, sizeof(rec), 1, collector.tmp_file) == 1) {
                if (rec.t_ns >= t_fail) {
                    active_count = i;
                    failed_in_stream = true;
                    if (i > 0) {
                        p_raw_split = s_pos_pass1[i - 1];
                    }
                    break;
                }
            }
        }
    }

    // Apply Pass 2 bounded-workspace filter ONLY to the active flight segment
    if (params::enable_two_pass_filter && active_count > 0) {
        apply_bounded_workspace_filter(s_pos_pass1, active_count, dt_nom, params::two_pass_cutoff_period);
    }

    // Apply Checkpointed PV-RTS backward smoother ONLY to the active flight segment
    if (params::enable_rts_smoother && active_count > 0) {
        apply_pv_rts_smoother(s_pos_pass1, s_time_pass1, active_count, dt_nom);
    }

    // If failure occurred during stream, shift subsequent dead-reckoned poses
    // so they project forward continuously from the filtered failure point without backward-pass distortion
    if (failed_in_stream && active_count > 0 && active_count < stream_count) {
        Vec3 p_filtered_split = s_pos_pass1[active_count - 1];
        Vec3 delta_align = p_filtered_split - p_raw_split;
        for (size_t i = active_count; i < stream_count; ++i) {
            s_pos_pass1[i] = s_pos_pass1[i] + delta_align;
        }
    }

    // Write all active and in-stream poses
    if (collector.tmp_file) {
        std::rewind(collector.tmp_file);
        for (size_t i = 0; i < stream_count; ++i) {
            TrajectoryCollector::Record rec;
            if (std::fread(&rec, sizeof(rec), 1, collector.tmp_file) == 1) {
                writer.write_pose(rec.t_ns,
                                  s_pos_pass1[i].data[0],
                                  s_pos_pass1[i].data[1],
                                  s_pos_pass1[i].data[2],
                                  rec.qx, rec.qy, rec.qz, rec.qw);
            }
        }
    }

    // Append blind (5.0s extrapolation) from clean filtered endpoint
    if (append_blind && prev_t_ns > 0 && stream_count > 0) {
        if (!blind_predictor.is_failed()) {
            blind_predictor.notify_stream_end(prev_t_ns);
        }

        Vec3 p_last_filtered = s_pos_pass1[stream_count - 1];
        Vec3 p_raw_endpoint = blind_predictor.frozen_state().p;
        Vec3 delta_endpoint_align = p_last_filtered - p_raw_endpoint;

        int64_t dt_ns_step = std::max(static_cast<int64_t>(1), static_cast<int64_t>(std::round(dt_nom * 1e9)));
        int blind_steps = static_cast<int>(std::round(5.0 * 1e9 / static_cast<double>(dt_ns_step)));

        for (int step = 1; step <= blind_steps; ++step) {
            int64_t t_extrap = prev_t_ns + step * dt_ns_step;
            Vec3 p_extrap;
            Quat q_extrap;
            blind_predictor.predict(t_extrap, p_extrap, q_extrap);
            Vec3 p_final = p_extrap + delta_endpoint_align;
            writer.write_pose(t_extrap, p_final.data[0], p_final.data[1], p_final.data[2],
                              q_extrap.x, q_extrap.y, q_extrap.z, q_extrap.w);
        }
    }

#ifdef ENABLE_ORACLES
    if (tel_fp) {
        std::fclose(tel_fp);
    }
#endif

    writer.close();
    reader.close();
    return 0;
}
