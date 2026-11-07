#include "pipeline.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <strings.h>

namespace {

struct BufferedSample {
    int64_t t_ns;
    Vec3 omega;
    Vec3 acc;
    bool is_valid;
};

// Static preallocated sample buffer for initial window to avoid heap allocations
static constexpr size_t MAX_INIT_SAMPLES = 1000;
static BufferedSample s_init_buf[MAX_INIT_SAMPLES];

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

int run_pipeline(const char* input_path,
                 const char* output_path,
                 AblationMode mode,
                 bool append_blind) {
    switch (mode) {
        case AblationMode::NAIVE:
            return run_naive(input_path, output_path, append_blind);
        case AblationMode::MAHONY:
            return run_mahony(input_path, output_path, append_blind);
        case AblationMode::ESKF_BASIC:
            return run_eskf_basic(input_path, output_path, append_blind);
        case AblationMode::ESKF_FULL:
        default:
            return run_eskf_full(input_path, output_path, append_blind);
    }
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
        if (dt > 0.0 && dt < 10.0) {
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
                if (dt > 0.0 && dt < 10.0) {
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
                if (dt > 0.0 && dt < 10.0) {
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
                        stat_buf.add_sample(t_sec, w_smp, a_smp);
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
            stat_buf.add_sample(t_sec, w_curr, acc_curr);
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
                if (dt > 0.0) {
                    state = strapdown_step(state, s_init_buf[idx].omega, s_init_buf[idx].acc, dt, g_w);
                }
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
    GapHandler gap_handler;
    DegradationMonitor monitor;
    FailureDetector blind_predictor;
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

    auto process_full_step = [&](int64_t t_curr,
                                 const Vec3& omega_curr,
                                 const Vec3& acc_curr) {
        if (blind_predictor.is_failed()) {
            Vec3 p_blind;
            Quat q_blind;
            blind_predictor.predict(t_curr, p_blind, q_blind);
            writer.write_pose(t_curr, p_blind.data[0], p_blind.data[1], p_blind.data[2],
                              q_blind.x, q_blind.y, q_blind.z, q_blind.w);
            prev_t_ns = t_curr;
            return;
        }

        GapStepResult step_res = gap_handler.process_sample(t_curr, omega_curr, acc_curr, state, P);

        if (step_res.gap_case == GapCase::ZERO_OR_NEGATIVE_DT) {
            writer.write_pose(t_curr, state.p.data[0], state.p.data[1], state.p.data[2],
                              state.q.x, state.q.y, state.q.z, state.q.w);
            return;
        }

        if (step_res.gap_case == GapCase::GAP || step_res.gap_case == GapCase::INVALID_SAMPLE) {
            state = step_res.state;
            P = step_res.P;
            bool failed = blind_predictor.update(t_curr, false, state.p, state.v, state.q, gap_handler.omega_s());
            Vec3 p_out = state.p;
            Quat q_out = state.q;
            if (failed || blind_predictor.is_failed()) {
                blind_predictor.predict(t_curr, p_out, q_out);
            }
            writer.write_pose(t_curr, p_out.data[0], p_out.data[1], p_out.data[2],
                              q_out.x, q_out.y, q_out.z, q_out.w);
            prev_t_ns = t_curr;
            return;
        }

        // Normal step
        double dt = static_cast<double>(t_curr - prev_t_ns) * 1e-9;
        prev_t_ns = t_curr;

        state = strapdown_step(state, omega_curr, acc_curr, dt, g_w);
        Mat15 F = build_f_matrix(state, omega_curr, acc_curr, dt);
        Mat15 Q = build_q_matrix(dt, params::noise_mult_gyro, params::noise_mult_acc,
                                 params::walk_mult_bg, params::walk_mult_ba, monitor.kappa());
        P = propagate_covariance(P, F, Q);

        double t_sec = static_cast<double>(t_curr - t0_ns) * 1e-9;
        stat_buf.add_sample(t_sec, omega_curr, acc_curr);
        bool is_stat = stat_buf.is_stationary(g_mag, monitor.quality(), params::zupt_window_s);
        Vec3 omega_stat_mean = stat_buf.get_mean_gyro(params::zupt_window_s);
        monitor.step(t_sec, omega_curr, acc_curr, is_stat, is_stat ? &omega_stat_mean : nullptr);

        if (step_res.apply_updates) {
            apply_tilt_update(state, P, acc_curr, omega_curr, monitor.quality(), g_w, 1e9, step_res.r_scale);
            if (is_stat) {
                apply_zupt_update(state, P, 1e9, step_res.r_scale);
            }
        }

        blind_predictor.update(t_curr, true, state.p, state.v, state.q, gap_handler.omega_s());
        writer.write_pose(t_curr, state.p.data[0], state.p.data[1], state.p.data[2],
                          state.q.x, state.q.y, state.q.z, state.q.w);
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

            if (initializer.is_ready()) {
                is_initialized = true;
                initializer.finish(state, P, g_w);
                g_mag = initializer.g_mag();
                t0_ns = s_init_buf[0].t_ns;

                for (size_t idx = 0; idx < init_count; ++idx) {
                    int64_t t_smp = s_init_buf[idx].t_ns;
                    Vec3 w_smp = s_init_buf[idx].omega;
                    Vec3 a_smp = s_init_buf[idx].acc;

                    if (idx == 0) {
                        prev_t_ns = t_smp;
                        blind_predictor.record_reliable_state(t_smp, state.p, state.v, state.q,
                                                              math::make_vec3(0.0, 0.0, 0.0));
                        gap_handler.process_sample(t_smp, w_smp, a_smp, state, P);

                        writer.write_pose(t_smp, state.p.data[0], state.p.data[1], state.p.data[2],
                                          state.q.x, state.q.y, state.q.z, state.q.w);
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
                if (dt > 0.0) {
                    state = strapdown_step(state, s_init_buf[idx].omega, s_init_buf[idx].acc, dt, g_w);
                }
            }
            writer.write_pose(t_smp, state.p.data[0], state.p.data[1], state.p.data[2],
                              state.q.x, state.q.y, state.q.z, state.q.w);
        }
    }

    // Append blind (5.0s extrapolation)
    if (append_blind && prev_t_ns > 0) {
        if (!blind_predictor.is_failed()) {
            blind_predictor.notify_stream_end(prev_t_ns);
        }

        double dt_nom = (gap_handler.dt_nom() > 0.0) ? gap_handler.dt_nom() : 0.005;
        int64_t dt_ns_step = std::max(static_cast<int64_t>(1), static_cast<int64_t>(std::round(dt_nom * 1e9)));
        int blind_steps = static_cast<int>(std::round(5.0 * 1e9 / static_cast<double>(dt_ns_step)));

        for (int step = 1; step <= blind_steps; ++step) {
            int64_t t_extrap = prev_t_ns + step * dt_ns_step;
            Vec3 p_extrap;
            Quat q_extrap;
            blind_predictor.predict(t_extrap, p_extrap, q_extrap);
            writer.write_pose(t_extrap, p_extrap.data[0], p_extrap.data[1], p_extrap.data[2],
                              q_extrap.x, q_extrap.y, q_extrap.z, q_extrap.w);
        }
    }

    writer.close();
    reader.close();
    return 0;
}
