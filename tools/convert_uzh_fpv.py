#!/usr/bin/env python3
"""
Convert UZH-FPV Snapdragon dataset into:
1. EuRoC-compatible CSV for the C++ IMU pipeline
2. TUM-formatted ground truth for evaluation with evo
"""

import sys
import os

def convert_imu(input_path, output_path):
    print(f"Converting IMU: {input_path} -> {output_path}")
    count = 0
    with open(input_path, 'r') as fin, open(output_path, 'w') as fout:
        fout.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for line in fin:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # Format: index timestamp_sec wx wy wz ax ay az
            if len(parts) < 8:
                continue
            t_sec = float(parts[1])
            t_ns = int(round(t_sec * 1e9))
            wx = float(parts[2])
            wy = float(parts[3])
            wz = float(parts[4])
            ax = float(parts[5])
            ay = float(parts[6])
            az = float(parts[7])
            fout.write(f"{t_ns},{wx:.9f},{wy:.9f},{wz:.9f},{ax:.9f},{ay:.9f},{az:.9f}\n")
            count += 1
    print(f"Wrote {count} IMU rows.")

def convert_gt(input_path, output_path):
    print(f"Converting GT: {input_path} -> {output_path}")
    count = 0
    with open(input_path, 'r') as fin, open(output_path, 'w') as fout:
        for line in fin:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # Format: timestamp_sec tx ty tz qx qy qz qw
            if len(parts) < 8:
                continue
            t_sec = float(parts[0])
            tx = float(parts[1])
            ty = float(parts[2])
            tz = float(parts[3])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])
            fout.write(f"{t_sec:.9f} {tx:.7f} {ty:.7f} {tz:.7f} {qx:.7f} {qy:.7f} {qz:.7f} {qw:.7f}\n")
            count += 1
    print(f"Wrote {count} GT rows.")

if __name__ == '__main__':
    base_dir = "data/uzh_fpv"
    imu_in = os.path.join(base_dir, "imu.txt")
    imu_out = os.path.join(base_dir, "imu_indoor_forward_7.csv")
    gt_in = os.path.join(base_dir, "groundtruth.txt")
    gt_out = os.path.join(base_dir, "gt_indoor_forward_7.txt")

    convert_imu(imu_in, imu_out)
    convert_gt(gt_in, gt_out)
