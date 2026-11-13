#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#include "src/spike.h"

void test_median3_permutations() {
    double a = 1.0, b = 2.0, c = 3.0;
    assert(SpikeFilter::median3(a, b, c) == 2.0);
    assert(SpikeFilter::median3(a, c, b) == 2.0);
    assert(SpikeFilter::median3(b, a, c) == 2.0);
    assert(SpikeFilter::median3(b, c, a) == 2.0);
    assert(SpikeFilter::median3(c, a, b) == 2.0);
    assert(SpikeFilter::median3(c, b, a) == 2.0);

    // Duplicates
    assert(SpikeFilter::median3(1.0, 1.0, 2.0) == 1.0);
    assert(SpikeFilter::median3(1.0, 2.0, 2.0) == 2.0);
    assert(SpikeFilter::median3(5.0, 5.0, 5.0) == 5.0);
    std::printf("[PASS] test_median3_permutations\n");
}

void test_hampel3() {
    // Isolated spike: 0.0, 1.0, 0.0 with threshold 0.3 -> should be clamped to 0.0
    double res1 = SpikeFilter::hampel3(0.0, 1.0, 0.0, 0.3);
    assert(std::abs(res1 - 0.0) < 1e-9);

    // Negative spike: 0.0, -1.0, 0.0 with threshold 0.3 -> should be clamped to 0.0
    double res2 = SpikeFilter::hampel3(0.0, -1.0, 0.0, 0.3);
    assert(std::abs(res2 - 0.0) < 1e-9);

    // Ramp: 1.0, 2.0, 3.0 -> not a local peak, must remain 2.0
    double res3 = SpikeFilter::hampel3(1.0, 2.0, 3.0, 0.3);
    assert(std::abs(res3 - 2.0) < 1e-9);

    // Small peak below threshold: 1.0, 1.1, 1.0 with threshold 0.3 -> remain 1.1
    double res4 = SpikeFilter::hampel3(1.0, 1.1, 1.0, 0.3);
    assert(std::abs(res4 - 1.1) < 1e-9);

    std::printf("[PASS] test_hampel3\n");
}

void test_streaming_count_and_timestamps() {
    for (size_t n : {0, 1, 2, 3, 4, 10, 100}) {
        SpikeFilter filter;
        std::vector<io::IMUSample> in_samples(n);
        std::vector<io::IMUSample> out_samples;

        for (size_t i = 0; i < n; ++i) {
            in_samples[i].t_ns = 1000000000LL + static_cast<int64_t>(i) * 5000000LL;
            in_samples[i].wx = static_cast<double>(i) * 0.01;
            in_samples[i].wy = 0.0;
            in_samples[i].wz = 0.0;
            in_samples[i].ax = 0.0;
            in_samples[i].ay = 0.0;
            in_samples[i].az = 9.81;
            in_samples[i].is_valid = true;
            in_samples[i].flags = io::FLAG_OK;

            io::IMUSample out;
            if (filter.process(in_samples[i], out)) {
                out_samples.push_back(out);
            }
        }

        io::IMUSample out_flush;
        if (filter.flush(out_flush)) {
            out_samples.push_back(out_flush);
        }

        assert(out_samples.size() == n);
        for (size_t i = 0; i < n; ++i) {
            assert(out_samples[i].t_ns == in_samples[i].t_ns);
        }
    }
    std::printf("[PASS] test_streaming_count_and_timestamps\n");
}

void test_spike_suppression() {
    SpikeFilter filter;
    size_t n = 20;
    std::vector<io::IMUSample> in_samples(n);
    std::vector<io::IMUSample> out_samples;

    for (size_t i = 0; i < n; ++i) {
        in_samples[i].t_ns = 1000000000LL + static_cast<int64_t>(i) * 5000000LL;
        in_samples[i].wx = 0.1;
        in_samples[i].wy = 0.2;
        in_samples[i].wz = -0.1;
        in_samples[i].ax = 0.5;
        in_samples[i].ay = -0.5;
        in_samples[i].az = 9.81;
        in_samples[i].is_valid = true;
        in_samples[i].flags = io::FLAG_OK;
    }

    // Inject massive spikes at sample 5 and sample 12
    in_samples[5].wx += 0.5;   // gyro spike
    in_samples[5].ax += 5.0;   // accel spike
    in_samples[12].wz -= 0.5;  // negative gyro spike
    in_samples[12].az -= 5.0;  // negative accel spike

    for (size_t i = 0; i < n; ++i) {
        io::IMUSample out;
        if (filter.process(in_samples[i], out)) {
            out_samples.push_back(out);
        }
    }
    io::IMUSample out_flush;
    if (filter.flush(out_flush)) {
        out_samples.push_back(out_flush);
    }

    assert(out_samples.size() == n);
    // Verify that gyro spikes at index 5 and 12 were completely suppressed
    assert(std::abs(out_samples[5].wx - 0.1) < 1e-6);
    assert(std::abs(out_samples[12].wz - (-0.1)) < 1e-6);

    // Accel spike is suppressed only if spike_filter_acc is enabled
    if (params::spike_filter_acc) {
        assert(std::abs(out_samples[5].ax - 0.5) < 1e-6);
        assert(std::abs(out_samples[12].az - 9.81) < 1e-6);
    } else {
        assert(std::abs(out_samples[5].ax - 5.5) < 1e-6);
        assert(std::abs(out_samples[12].az - 4.81) < 1e-6);
    }

    std::printf("[PASS] test_spike_suppression\n");
}

void test_gap_and_invalid_handling() {
    SpikeFilter filter;
    size_t n = 10;
    std::vector<io::IMUSample> in_samples(n);
    std::vector<io::IMUSample> out_samples;

    for (size_t i = 0; i < n; ++i) {
        in_samples[i].t_ns = 1000000000LL + static_cast<int64_t>(i) * 5000000LL;
        in_samples[i].wx = 0.1;
        in_samples[i].ax = 0.5;
        in_samples[i].is_valid = true;
    }

    // Sample 4 is invalid (NaN dropout)
    in_samples[4].is_valid = false;
    in_samples[4].wx = std::nan("");

    // Sample 7 has a 2.0s gap
    in_samples[7].t_ns = in_samples[6].t_ns + 2000000000LL;

    for (size_t i = 0; i < n; ++i) {
        io::IMUSample out;
        if (filter.process(in_samples[i], out)) {
            out_samples.push_back(out);
        }
    }
    io::IMUSample out_flush;
    if (filter.flush(out_flush)) {
        out_samples.push_back(out_flush);
    }

    assert(out_samples.size() == n);
    // Invalid sample 4 must remain invalid and not corrupt neighbor 3 or 5
    assert(!out_samples[4].is_valid);
    assert(out_samples[3].is_valid);
    assert(out_samples[5].is_valid);
    assert(!std::isnan(out_samples[3].wx));
    assert(!std::isnan(out_samples[5].wx));

    std::printf("[PASS] test_gap_and_invalid_handling\n");
}

int main() {
    std::printf("=== Running Spike Filter Unit Tests ===\n");
    test_median3_permutations();
    test_hampel3();
    test_streaming_count_and_timestamps();
    test_spike_suppression();
    test_gap_and_invalid_handling();
    std::printf("=== All Spike Filter Unit Tests Passed ===\n");
    return 0;
}
