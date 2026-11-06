"""Synthetic IMU and ground-truth trajectory generator.

Generates ideal IMU measurements (w_RS_S, a_RS_S) and ground-truth poses (TUM format)
for 5 analytic trajectories:
1. static
2. constant_yaw
3. circle
4. figure_eight
5. sum_of_sines
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, Tuple, Optional
import numpy as np

TRAJECTORY_TYPES = ["static", "constant_yaw", "circle", "figure_eight", "sum_of_sines"]


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product q1 (x) q2, where q = [w, x, y, z]."""
    w1, x1, y1, z1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    w2, x2, y2, z2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.stack([w, x, y, z], axis=-1)


def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert Hamilton quaternion [w, x, y, z] to 3x3 rotation matrix R (body to world)."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3), dtype=np.float64)
    R[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    R[..., 0, 1] = 2.0 * (x * y - w * z)
    R[..., 0, 2] = 2.0 * (x * z + w * y)

    R[..., 1, 0] = 2.0 * (x * y + w * z)
    R[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    R[..., 1, 2] = 2.0 * (y * z - w * x)

    R[..., 2, 0] = 2.0 * (x * z - w * y)
    R[..., 2, 1] = 2.0 * (y * z + w * x)
    R[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return R


def rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit Hamilton quaternion [w, x, y, z]."""
    tr = R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2]
    q = np.empty(R.shape[:-2] + (4,), dtype=np.float64)

    # Flatten batch dimensions if any
    orig_shape = R.shape[:-2]
    R_flat = R.reshape(-1, 3, 3)
    q_flat = q.reshape(-1, 4)

    for i in range(len(R_flat)):
        r = R_flat[i]
        trace = r[0, 0] + r[1, 1] + r[2, 2]
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (r[2, 1] - r[1, 2]) * s
            y = (r[0, 2] - r[2, 0]) * s
            z = (r[1, 0] - r[0, 1]) * s
        elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s

        norm = np.sqrt(w * w + x * x + y * y + z * z)
        q_flat[i] = [w / norm, x / norm, y / norm, z / norm]

    return q.reshape(orig_shape + (4,)) if orig_shape else q_flat[0]


def quat_exp(phi: np.ndarray) -> np.ndarray:
    """Exponential map of rotation vector phi to unit Hamilton quaternion [w, x, y, z]."""
    angle = np.linalg.norm(phi, axis=-1, keepdims=True)
    small_mask = (angle < 1e-8).squeeze(-1)

    q = np.empty(phi.shape[:-1] + (4,), dtype=np.float64)
    if np.any(~small_mask):
        phi_reg = phi[~small_mask]
        ang_reg = angle[~small_mask]
        w = np.cos(0.5 * ang_reg)
        xyz = np.sin(0.5 * ang_reg) * (phi_reg / ang_reg)
        q[~small_mask] = np.concatenate([w, xyz], axis=-1)

    if np.any(small_mask):
        phi_sm = phi[small_mask]
        w = np.ones(phi_sm.shape[:-1] + (1,), dtype=np.float64)
        xyz = 0.5 * phi_sm
        raw = np.concatenate([w, xyz], axis=-1)
        norm = np.linalg.norm(raw, axis=-1, keepdims=True)
        q[small_mask] = raw / norm

    return q


def euler_zyx_to_quat_and_omega(
    roll: np.ndarray,
    pitch: np.ndarray,
    yaw: np.ndarray,
    droll: np.ndarray,
    dpitch: np.ndarray,
    dyaw: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert Euler angles (roll phi around x, pitch theta around y, yaw psi around z)

    and their time derivatives to Hamilton quaternion [w, x, y, z] and body angular velocity omega_b.
    Convention: R = R_z(yaw) @ R_y(pitch) @ R_x(roll).
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    # Hamilton quaternion: q_z * q_y * q_x
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = cy * sp * cr + sy * cp * sr
    z = sy * cp * cr - cy * sp * sr
    q = np.stack([w, x, y, z], axis=-1)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)

    # Body angular velocity omega_b:
    # omega_b = [droll - sin(pitch)*dyaw,
    #            cos(roll)*dpitch + sin(roll)*cos(pitch)*dyaw,
    #           -sin(roll)*dpitch + cos(roll)*cos(pitch)*dyaw]
    c_roll = np.cos(roll)
    s_roll = np.sin(roll)
    c_pitch = np.cos(pitch)
    s_pitch = np.sin(pitch)

    wx = droll - s_pitch * dyaw
    wy = c_roll * dpitch + s_roll * c_pitch * dyaw
    wz = -s_roll * dpitch + c_roll * c_pitch * dyaw
    omega = np.stack([wx, wy, wz], axis=-1)

    return q, omega


def generate_trajectory(
    traj_type: str = "static",
    duration: float = 10.0,
    rate: float = 200.0,
    g_mag: float = 9.80665,
    t0_ns: int = 1403636579000000000,
    yaw_rate: float = 0.5,
    seed: int = 42,
) -> Dict[str, np.ndarray]:
    """Generate exact analytic trajectory with ideal IMU measurements.

    Returns dict containing:
      t_ns: int64 timestamps (ns)
      t: float seconds from 0
      p: (N, 3) position in world frame (m)
      v: (N, 3) velocity in world frame (m/s)
      q: (N, 4) orientation [w, x, y, z] body to world
      omega: (N, 3) body angular velocity w_RS_S (rad/s)
      acc: (N, 3) body specific force a_RS_S (m/s^2), including gravity
    """
    if traj_type not in TRAJECTORY_TYPES:
        raise ValueError(f"Unknown trajectory type {traj_type}, must be one of {TRAJECTORY_TYPES}")

    dt = 1.0 / rate
    n_samples = int(round(duration * rate))
    t = np.arange(n_samples, dtype=np.float64) * dt
    dt_ns = int(round(1e9 / rate))
    t_ns = t0_ns + np.arange(n_samples, dtype=np.int64) * dt_ns

    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    if traj_type == "static":
        p = np.zeros((n_samples, 3), dtype=np.float64)
        v = np.zeros((n_samples, 3), dtype=np.float64)
        p_ddot = np.zeros((n_samples, 3), dtype=np.float64)
        q = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (n_samples, 1))
        omega = np.zeros((n_samples, 3), dtype=np.float64)

    elif traj_type == "constant_yaw":
        p = np.zeros((n_samples, 3), dtype=np.float64)
        v = np.zeros((n_samples, 3), dtype=np.float64)
        p_ddot = np.zeros((n_samples, 3), dtype=np.float64)
        psi = yaw_rate * t
        w = np.cos(0.5 * psi)
        z = np.sin(0.5 * psi)
        q = np.stack([w, np.zeros_like(w), np.zeros_like(w), z], axis=-1)
        omega = np.tile(np.array([0.0, 0.0, yaw_rate]), (n_samples, 1))

    elif traj_type == "circle":
        # Banked circular trajectory:
        # Radius R, angular frequency omega_0
        R = 5.0
        v_speed = 2.0
        w0 = v_speed / R

        # Position, velocity, acceleration in world frame
        px = R * np.cos(w0 * t)
        py = R * np.sin(w0 * t)
        pz = np.zeros_like(t)
        p = np.stack([px, py, pz], axis=-1)

        vx = -R * w0 * np.sin(w0 * t)
        vy = R * w0 * np.cos(w0 * t)
        vz = np.zeros_like(t)
        v = np.stack([vx, vy, vz], axis=-1)

        ax = -R * (w0**2) * np.cos(w0 * t)
        ay = -R * (w0**2) * np.sin(w0 * t)
        az = np.zeros_like(t)
        p_ddot = np.stack([ax, ay, az], axis=-1)

        # In a coordinated turn:
        # Apparent acceleration (specific force in world): f_w = p_ddot - g_w
        # f_w = [ax, ay, g_mag]
        fw = p_ddot - g_w
        fw_norm = np.linalg.norm(fw, axis=1, keepdims=True)

        # Body frame definition:
        # Forward x_b = v / |v| (tangent)
        # Down z_b = fw / |fw| (direction of apparent gravity)
        # Right y_b = z_b x x_b
        # Note: v . fw = vx*ax + vy*ay + 0 = 0, so x_b and z_b are strictly orthogonal!
        xb = v / np.linalg.norm(v, axis=1, keepdims=True)
        zb = fw / fw_norm
        yb = np.cross(zb, xb)

        # Build rotation matrix R = [xb, yb, zb]
        R_mat = np.stack([xb, yb, zb], axis=-1)
        q = rot_matrix_to_quat(R_mat)

        # Time derivative of R_mat:
        # dot(xb) = ax / v_speed
        dot_xb = p_ddot / v_speed
        # dot(fw) = [-R*w0^3*(-sin), -R*w0^3*(cos), 0]
        dot_fw = np.stack([R * (w0**3) * np.sin(w0 * t), -R * (w0**3) * np.cos(w0 * t), np.zeros_like(t)], axis=-1)
        dot_zb = dot_fw / fw_norm  # fw_norm is constant!
        dot_yb = np.cross(dot_zb, xb) + np.cross(zb, dot_xb)

        dot_R = np.stack([dot_xb, dot_yb, dot_zb], axis=-1)
        # Omega skew = R^T @ dot_R
        # omega_b = [Omega_21, Omega_02, Omega_10]
        Omega = np.matmul(np.swapaxes(R_mat, -1, -2), dot_R)
        omega = np.stack([Omega[:, 2, 1], Omega[:, 0, 2], Omega[:, 1, 0]], axis=-1)

    elif traj_type == "figure_eight":
        A = 5.0
        w0 = 0.2
        B = 1.0
        wz = 0.4

        px = A * np.sin(w0 * t)
        py = 0.5 * A * np.sin(2.0 * w0 * t)
        pz = B * np.sin(wz * t)
        p = np.stack([px, py, pz], axis=-1)

        vx = A * w0 * np.cos(w0 * t)
        vy = A * w0 * np.cos(2.0 * w0 * t)
        vz = B * wz * np.cos(wz * t)
        v = np.stack([vx, vy, vz], axis=-1)

        ax = -A * (w0**2) * np.sin(w0 * t)
        ay = -2.0 * A * (w0**2) * np.sin(2.0 * w0 * t)
        az = -B * (wz**2) * np.sin(wz * t)
        p_ddot = np.stack([ax, ay, az], axis=-1)

        # Smooth orientation from trajectory angles
        yaw = 0.5 * np.sin(w0 * t)
        dyaw = 0.5 * w0 * np.cos(w0 * t)
        pitch = 0.2 * np.sin(wz * t)
        dpitch = 0.2 * wz * np.cos(wz * t)
        roll = 0.15 * np.sin(2.0 * w0 * t)
        droll = 0.3 * w0 * np.cos(2.0 * w0 * t)

        q, omega = euler_zyx_to_quat_and_omega(roll, pitch, yaw, droll, dpitch, dyaw)

    elif traj_type == "sum_of_sines":
        rng = np.random.RandomState(seed)
        # Sum of 3 sines for position and angles
        px, py, pz = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
        vx, vy, vz = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
        ax, ay, az = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)

        for _ in range(3):
            amp = rng.uniform(0.5, 2.0)
            freq = rng.uniform(0.1, 0.5)
            phase = rng.uniform(0.0, 2.0 * np.pi)
            px += amp * np.sin(freq * t + phase)
            vx += amp * freq * np.cos(freq * t + phase)
            ax -= amp * (freq**2) * np.sin(freq * t + phase)

            amp_y = rng.uniform(0.5, 2.0)
            freq_y = rng.uniform(0.1, 0.5)
            phase_y = rng.uniform(0.0, 2.0 * np.pi)
            py += amp_y * np.sin(freq_y * t + phase_y)
            vy += amp_y * freq_y * np.cos(freq_y * t + phase_y)
            ay -= amp_y * (freq_y**2) * np.sin(freq_y * t + phase_y)

            amp_z = rng.uniform(0.2, 0.8)
            freq_z = rng.uniform(0.1, 0.4)
            phase_z = rng.uniform(0.0, 2.0 * np.pi)
            pz += amp_z * np.sin(freq_z * t + phase_z)
            vz += amp_z * freq_z * np.cos(freq_z * t + phase_z)
            az -= amp_z * (freq_z**2) * np.sin(freq_z * t + phase_z)

        p = np.stack([px, py, pz], axis=-1)
        v = np.stack([vx, vy, vz], axis=-1)
        p_ddot = np.stack([ax, ay, az], axis=-1)

        # Smooth angles (max ~0.25 rad to avoid gimbal lock)
        roll, droll = np.zeros_like(t), np.zeros_like(t)
        pitch, dpitch = np.zeros_like(t), np.zeros_like(t)
        yaw, dyaw = np.zeros_like(t), np.zeros_like(t)

        for _ in range(2):
            ar, fr, pr = rng.uniform(0.05, 0.15), rng.uniform(0.1, 0.3), rng.uniform(0.0, 2.0 * np.pi)
            roll += ar * np.sin(fr * t + pr)
            droll += ar * fr * np.cos(fr * t + pr)

            ap, fp, pp = rng.uniform(0.05, 0.15), rng.uniform(0.1, 0.3), rng.uniform(0.0, 2.0 * np.pi)
            pitch += ap * np.sin(fp * t + pp)
            dpitch += ap * fp * np.cos(fp * t + pp)

            ayw, fyw, pyw = rng.uniform(0.1, 0.3), rng.uniform(0.05, 0.2), rng.uniform(0.0, 2.0 * np.pi)
            yaw += ayw * np.sin(fyw * t + pyw)
            dyaw += ayw * fyw * np.cos(fyw * t + pyw)

        q, omega = euler_zyx_to_quat_and_omega(roll, pitch, yaw, droll, dpitch, dyaw)

    # Specific force measured by accelerometer in body frame:
    # a_body = R(q)^T (p_ddot - g_w)
    R_bw = quat_to_rot_matrix(q)  # body to world
    R_wb = np.swapaxes(R_bw, -1, -2)  # world to body
    f_w = p_ddot - g_w
    acc = np.matmul(R_wb, f_w[..., np.newaxis]).squeeze(-1)

    return {
        "t_ns": t_ns,
        "t": t,
        "p": p,
        "v": v,
        "q": q,
        "omega": omega,
        "acc": acc,
    }


def strapdown_integrate(
    t_ns: np.ndarray,
    acc: np.ndarray,
    omega: np.ndarray,
    q0: np.ndarray,
    p0: Optional[np.ndarray] = None,
    v0: Optional[np.ndarray] = None,
    g_mag: float = 9.80665,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate ideal IMU measurements using standard strapdown integration (SPEC §12.3)."""
    n = len(t_ns)
    p_est = np.zeros((n, 3), dtype=np.float64)
    v_est = np.zeros((n, 3), dtype=np.float64)
    q_est = np.zeros((n, 4), dtype=np.float64)

    if p0 is not None:
        p_est[0] = p0
    if v0 is not None:
        v_est[0] = v0
    q_est[0] = q0 / np.linalg.norm(q0)

    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    for k in range(1, n):
        dt = (t_ns[k] - t_ns[k - 1]) * 1e-9
        if dt <= 0:
            p_est[k] = p_est[k - 1]
            v_est[k] = v_est[k - 1]
            q_est[k] = q_est[k - 1]
            continue

        q_old = q_est[k - 1]
        R_old = quat_to_rot_matrix(q_old)

        # Nominal propagation
        omega_hat = omega[k]
        a_hat = acc[k]

        delta_q = quat_exp(omega_hat * dt)
        q_new = quat_mult(q_old, delta_q)
        q_new = q_new / np.linalg.norm(q_new)
        # Sign continuity
        if np.dot(q_new, q_old) < 0.0:
            q_new = -q_new

        a_w = R_old @ a_hat + g_w
        p_new = p_est[k - 1] + v_est[k - 1] * dt + 0.5 * a_w * (dt**2)
        v_new = v_est[k - 1] + a_w * dt

        p_est[k] = p_new
        v_est[k] = v_new
        q_est[k] = q_new

    return p_est, v_est, q_est


def write_euroc_imu_csv(path: Path, t_ns: np.ndarray, omega: np.ndarray, acc: np.ndarray) -> None:
    """Write IMU data in EuRoC format: #timestamp [ns],w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for i in range(len(t_ns)):
            t_int = int(t_ns[i])
            wx, wy, wz = omega[i]
            ax, ay, az = acc[i]
            f.write(f"{t_int},{wx:.10f},{wy:.10f},{wz:.10f},{ax:.10f},{ay:.10f},{az:.10f}\n")


def write_tum_trajectory(path: Path, t_ns: np.ndarray, p: np.ndarray, q: np.ndarray) -> None:
    """Write ground truth trajectory in TUM format: timestamp tx ty tz qx qy qz qw.

    Timestamp is formatted as <sec>.<nsec zero-padded to 9 digits> via int64 integer arithmetic.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(t_ns)):
            t_int = int(t_ns[i])
            sec = t_int // 1_000_000_000
            nsec = t_int % 1_000_000_000
            t_str = f"{sec}.{nsec:09d}"
            tx, ty, tz = p[i]
            # q in state is [w, x, y, z], output TUM requires qx qy qz qw
            w, x, y, z = q[i]
            f.write(f"{t_str} {tx:.9f} {ty:.9f} {tz:.9f} {x:.9f} {y:.9f} {z:.9f} {w:.9f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate synthetic analytic IMU and ground truth trajectory.")
    parser.add_argument("--type", choices=TRAJECTORY_TYPES, default="static", help="Trajectory type")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration in seconds")
    parser.add_argument("--rate", type=float, default=200.0, help="IMU sampling rate in Hz")
    parser.add_argument("--g-mag", type=float, default=9.80665, help="Gravity magnitude in m/s^2")
    parser.add_argument("--yaw-rate", type=float, default=0.5, help="Yaw rate for constant_yaw (rad/s)")
    parser.add_argument("--seed", type=int, default=42, help="RNG seed for random smooth trajectory")
    parser.add_argument("--output-imu", type=str, default=None, help="Output path for EuRoC imu0/data.csv")
    parser.add_argument("--output-gt", type=str, default=None, help="Output path for groundtruth.txt (TUM format)")

    args = parser.parse_args()

    traj = generate_trajectory(
        traj_type=args.type,
        duration=args.duration,
        rate=args.rate,
        g_mag=args.g_mag,
        yaw_rate=args.yaw_rate,
        seed=args.seed,
    )

    if args.output_imu:
        write_euroc_imu_csv(Path(args.output_imu), traj["t_ns"], traj["omega"], traj["acc"])
        print(f"Wrote {len(traj['t_ns'])} IMU samples to {args.output_imu}")

    if args.output_gt:
        write_tum_trajectory(Path(args.output_gt), traj["t_ns"], traj["p"], traj["q"])
        print(f"Wrote {len(traj['t_ns'])} ground truth poses to {args.output_gt}")


if __name__ == "__main__":
    main()
