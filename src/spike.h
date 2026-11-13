#ifndef SPIKE_H
#define SPIKE_H

#include <cmath>
#include <cstdint>
#include <cstddef>
#include "io.h"
#include "params.h"

class SpikeFilter {
public:
    SpikeFilter() {
        reset();
    }

    void reset() {
        count_ = 0;
    }

    static inline double median3(double a, double b, double c) {
        if (a > b) {
            if (b > c) return b;
            return (a > c) ? c : a;
        } else {
            if (a > c) return a;
            return (b > c) ? c : b;
        }
    }

    static inline double hampel3(double x0, double x1, double x2, double thresh) {
        double d1 = x1 - x0;
        double d2 = x1 - x2;
        if (d1 * d2 > 0.0) { // Local peak / impulse
            double x_interp = 0.5 * (x0 + x2);
            if (std::abs(x1 - x_interp) > thresh) {
                return x_interp;
            }
        }
        return x1;
    }

    // Streaming feed: returns true and populates out if a filtered sample is ready.
    bool process(const io::IMUSample& in, io::IMUSample& out) {
        if (count_ == 0) {
            buf_[0] = in;
            count_ = 1;
            return false;
        }

        if (count_ == 1) {
            buf_[1] = in;
            count_ = 2;
            out = buf_[0];
            return true;
        }

        // count_ == 2: now we have 3 samples: buf_[0], buf_[1], and in
        buf_[2] = in;
        out = filter_sample(buf_[0], buf_[1], buf_[2]);

        // Shift window: store filtered sample into buf_[0] so spikes do not contaminate the next window
        buf_[0] = out;
        buf_[1] = buf_[2];
        count_ = 2;
        return true;
    }

    // Flush final sample at stream EOF
    bool flush(io::IMUSample& out) {
        if (count_ == 2) {
            out = buf_[1];
            count_ = 0;
            return true;
        } else if (count_ == 1) {
            out = buf_[0];
            count_ = 0;
            return true;
        }
        return false;
    }

private:
    io::IMUSample buf_[3];
    size_t count_;

    static io::IMUSample filter_sample(const io::IMUSample& s0,
                                       const io::IMUSample& s1,
                                       const io::IMUSample& s2) {
        io::IMUSample out = s1;

        // If any sample is invalid, do not attempt to median-filter across the invalidity
        if (!s0.is_valid || !s1.is_valid || !s2.is_valid) {
            return out;
        }

        // Check timestamp continuity: reject jumps or reversals
        double dt1 = static_cast<double>(s1.t_ns - s0.t_ns) * 1e-9;
        double dt2 = static_cast<double>(s2.t_ns - s1.t_ns) * 1e-9;
        if (dt1 <= 0.0 || dt1 > 0.05 || dt2 <= 0.0 || dt2 > 0.05) {
            return out;
        }

        // Gyro filtering
        if (params::spike_filter_hampel) {
            out.wx = hampel3(s0.wx, s1.wx, s2.wx, params::spike_thresh_gyro);
            out.wy = hampel3(s0.wy, s1.wy, s2.wy, params::spike_thresh_gyro);
            out.wz = hampel3(s0.wz, s1.wz, s2.wz, params::spike_thresh_gyro);
        } else {
            out.wx = median3(s0.wx, s1.wx, s2.wx);
            out.wy = median3(s0.wy, s1.wy, s2.wy);
            out.wz = median3(s0.wz, s1.wz, s2.wz);
        }

        // Accel filtering
        if (params::spike_filter_acc) {
            if (params::spike_filter_hampel) {
                out.ax = hampel3(s0.ax, s1.ax, s2.ax, params::spike_thresh_acc);
                out.ay = hampel3(s0.ay, s1.ay, s2.ay, params::spike_thresh_acc);
                out.az = hampel3(s0.az, s1.az, s2.az, params::spike_thresh_acc);
            } else {
                out.ax = median3(s0.ax, s1.ax, s2.ax);
                out.ay = median3(s0.ay, s1.ay, s2.ay);
                out.az = median3(s0.az, s1.az, s2.az);
            }
        }

        return out;
    }
};

#endif // SPIKE_H
