#!/usr/bin/env python3
"""create_degraded_zip.py - Create a compliant Zipped EuRoC IMU input dataset.

Generates a zipped folder with:
- Internal file path: imu0/data.csv
- Header: #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
- 64-bit integer nanoseconds
- Injected degradations: progressive noise ramp, bias random walk, missing/NaN/zero/repeated dropouts, tail cutoff.
Uses standard Python library only (zero external dependencies).
"""

import math
import random
import sys
import zipfile
import io

def degrade_and_zip(input_csv: str, output_zip: str, seed: int = 42):
    random.seed(seed)

    # 1. Read input CSV
    with open(input_csv, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    header = lines[0]
    data_lines = [l for l in lines[1:] if not l.startswith("#")]

    rows = []
    for l in data_lines:
        parts = l.split(",")
        t_ns = int(parts[0])
        wx, wy, wz = float(parts[1]), float(parts[2]), float(parts[3])
        ax, ay, az = float(parts[4]), float(parts[5]), float(parts[6])
        rows.append((t_ns, wx, wy, wz, ax, ay, az))

    if not rows:
        raise ValueError("No data rows found in input CSV")

    t0_ns = rows[0][0]
    tend_ns = rows[-1][0]
    total_dur_s = (tend_ns - t0_ns) * 1e-9

    # Degradation parameters
    noise_rate = 0.05
    sigma_g_base = 1.7e-4
    sigma_a_base = 2.0e-3
    bias_walk_rate = 0.001
    sigma_bg_step = 1.9e-5 * math.sqrt(bias_walk_rate)
    sigma_ba_step = 3.0e-3 * math.sqrt(bias_walk_rate)

    # Dropout windows (relative to total duration)
    # 1. Missing rows (0.5s duration)
    t_d1_start = min(1.5, 0.15 * total_dur_s)
    t_d1_end = t_d1_start + min(0.5, 0.05 * total_dur_s)
    # 2. NaN values (0.8s duration)
    t_d2_start = min(3.0, 0.35 * total_dur_s)
    t_d2_end = t_d2_start + min(0.8, 0.08 * total_dur_s)
    # 3. Zeros (0.5s duration)
    t_d3_start = min(5.0, 0.55 * total_dur_s)
    t_d3_end = t_d3_start + min(0.5, 0.05 * total_dur_s)
    # 4. Repeated values (0.4s duration)
    t_d4_start = min(7.0, 0.70 * total_dur_s)
    t_d4_end = t_d4_start + min(0.4, 0.04 * total_dur_s)
    # 5. Blind failure cutoff at tail (2.5s or 5.0s depending on sequence duration)
    blind_tail_s = min(5.0, 0.20 * total_dur_s)
    t_fail_start = max(0.0, total_dur_s - blind_tail_s)

    bgx = bgy = bgz = 0.0
    bax = bay = baz = 0.0
    last_valid_sample = None

    out_csv_buffer = io.StringIO()
    # Write EuRoC header
    out_csv_buffer.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")

    written_count = 0
    prev_t_ns = t0_ns

    for t_ns, wx, wy, wz, ax, ay, az in rows:
        t_s = (t_ns - t0_ns) * 1e-9
        dt_s = max(0.001, (t_ns - prev_t_ns) * 1e-9)
        prev_t_ns = t_ns

        # Blind cutoff at tail: sensor stops reporting
        if t_s >= t_fail_start:
            break

        # Dropout 1: Missing rows (skip)
        if t_d1_start <= t_s < t_d1_end:
            continue

        # Progressive bias walk
        dt_sqrt = math.sqrt(dt_s)
        bgx += random.gauss(0.0, sigma_bg_step * dt_sqrt)
        bgy += random.gauss(0.0, sigma_bg_step * dt_sqrt)
        bgz += random.gauss(0.0, sigma_bg_step * dt_sqrt)
        bax += random.gauss(0.0, sigma_ba_step * dt_sqrt)
        bay += random.gauss(0.0, sigma_ba_step * dt_sqrt)
        baz += random.gauss(0.0, sigma_ba_step * dt_sqrt)

        # Progressive noise ramp
        g_ramp = sigma_g_base * (1.0 + noise_rate * t_s)
        a_ramp = sigma_a_base * (1.0 + noise_rate * t_s)

        deg_wx = wx + bgx + random.gauss(0.0, g_ramp)
        deg_wy = wy + bgy + random.gauss(0.0, g_ramp)
        deg_wz = wz + bgz + random.gauss(0.0, g_ramp)
        deg_ax = ax + bax + random.gauss(0.0, a_ramp)
        deg_ay = ay + bay + random.gauss(0.0, a_ramp)
        deg_az = az + baz + random.gauss(0.0, a_ramp)

        # Dropout 2: NaN
        if t_d2_start <= t_s < t_d2_end:
            line_str = f"{t_ns},nan,nan,nan,nan,nan,nan\n"
        # Dropout 3: Zeros
        elif t_d3_start <= t_s < t_d3_end:
            line_str = f"{t_ns},0.000000,0.000000,0.000000,0.000000,0.000000,0.000000\n"
        # Dropout 4: Repeated values
        elif t_d4_start <= t_s < t_d4_end:
            if last_valid_sample is not None:
                r_wx, r_wy, r_wz, r_ax, r_ay, r_az = last_valid_sample
                line_str = f"{t_ns},{r_wx:.6f},{r_wy:.6f},{r_wz:.6f},{r_ax:.6f},{r_ay:.6f},{r_az:.6f}\n"
            else:
                line_str = f"{t_ns},{deg_wx:.6f},{deg_wy:.6f},{deg_wz:.6f},{deg_ax:.6f},{deg_ay:.6f},{deg_az:.6f}\n"
        else:
            line_str = f"{t_ns},{deg_wx:.6f},{deg_wy:.6f},{deg_wz:.6f},{deg_ax:.6f},{deg_ay:.6f},{deg_az:.6f}\n"
            last_valid_sample = (deg_wx, deg_wy, deg_wz, deg_ax, deg_ay, deg_az)

        out_csv_buffer.write(line_str)
        written_count += 1

    # Package into Zip archive
    csv_bytes = out_csv_buffer.getvalue().encode("utf-8")
    with zipfile.ZipFile(output_zip, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("imu0/data.csv", csv_bytes)

    print(f"Created {output_zip} successfully:")
    print(f"  Internal path: imu0/data.csv")
    print(f"  Raw samples: {len(rows)} -> Degraded written samples: {written_count}")
    print(f"  Duration: {total_dur_s:.2f}s (Tail cutoff applied at t >= {t_fail_start:.2f}s)")
    print(f"  Dropouts injected: missing [{t_d1_start:.1f}-{t_d1_end:.1f}s], NaN [{t_d2_start:.1f}-{t_d2_end:.1f}s], Zeros [{t_d3_start:.1f}-{t_d3_end:.1f}s], Repeated [{t_d4_start:.1f}-{t_d4_end:.1f}s]")


if __name__ == "__main__":
    src_csv = sys.argv[1] if len(sys.argv) > 1 else "sample_imu.csv"
    dst_zip = sys.argv[2] if len(sys.argv) > 2 else "sample_input_degraded.zip"
    degrade_and_zip(src_csv, dst_zip)
