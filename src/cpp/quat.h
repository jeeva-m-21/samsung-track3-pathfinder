#ifndef QUAT_H
#define QUAT_H

#include "mat.h"
#include <cmath>

namespace math {

struct Quat {
    double w;
    double x;
    double y;
    double z;

    static constexpr Quat identity() {
        return Quat{1.0, 0.0, 0.0, 0.0};
    }

    // Hamilton product: q * p
    inline Quat operator*(const Quat& p) const {
        return Quat{
            w * p.w - x * p.x - y * p.y - z * p.z,
            w * p.x + x * p.w + y * p.z - z * p.y,
            w * p.y - x * p.z + y * p.w + z * p.x,
            w * p.z + x * p.y - y * p.x + z * p.w
        };
    }

    inline Quat operator-() const {
        return Quat{-w, -x, -y, -z};
    }

    inline double norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    inline Quat conjugate() const {
        return Quat{w, -x, -y, -z};
    }

    inline Quat inverse() const {
        double n2 = w * w + x * x + y * y + z * z;
        if (n2 < 1e-24) {
            n2 = 1e-24;
        }
        double inv_n2 = 1.0 / n2;
        return Quat{w * inv_n2, -x * inv_n2, -y * inv_n2, -z * inv_n2};
    }

    inline void normalize(double floor_val = 1e-12) {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < floor_val) {
            n = floor_val;
        }
        double inv_n = 1.0 / n;
        w *= inv_n;
        x *= inv_n;
        y *= inv_n;
        z *= inv_n;
    }

    inline Quat normalized(double floor_val = 1e-12) const {
        Quat res = *this;
        res.normalize(floor_val);
        return res;
    }

    // Convert Hamilton quaternion [w, x, y, z] to 3x3 rotation matrix R (body to world)
    // Matches SPEC §12.1 and src/ref/mathlib.py
    inline Mat3 to_rot_mat() const {
        Mat3 R;
        R.data[0] = 1.0 - 2.0 * (y * y + z * z);
        R.data[1] = 2.0 * (x * y - w * z);
        R.data[2] = 2.0 * (x * z + w * y);

        R.data[3] = 2.0 * (x * y + w * z);
        R.data[4] = 1.0 - 2.0 * (x * x + z * z);
        R.data[5] = 2.0 * (y * z - w * x);

        R.data[6] = 2.0 * (x * z - w * y);
        R.data[7] = 2.0 * (y * z + w * x);
        R.data[8] = 1.0 - 2.0 * (x * x + y * y);
        return R;
    }

    // Rotate vector: R(q) * v
    inline Vec3 rotate(const Vec3& v) const {
        return to_rot_mat() * v;
    }

    // Inverse rotate vector: R(q)^T * v = R(q*) * v
    inline Vec3 rotate_inv(const Vec3& v) const {
        return conjugate().to_rot_mat() * v;
    }
};

// Standalone functions
inline Quat normalize(const Quat& q, double floor_val = 1e-12) {
    return q.normalized(floor_val);
}

inline Quat conjugate(const Quat& q) {
    return q.conjugate();
}

inline Quat inverse(const Quat& q) {
    return q.inverse();
}

inline Mat3 to_rot_mat(const Quat& q) {
    return q.to_rot_mat();
}

inline Vec3 rotate(const Quat& q, const Vec3& v) {
    return q.rotate(v);
}

inline Vec3 rotate_inv(const Quat& q, const Vec3& v) {
    return q.rotate_inv(v);
}

// 3x3 skew-symmetric cross-product matrix: skew(v) * u == v x u
// Matches SPEC §12.1 and src/ref/mathlib.py
inline Mat3 skew(const Vec3& v) {
    double vx = v.data[0];
    double vy = v.data[1];
    double vz = v.data[2];
    Mat3 S;
    S.data[0] =  0.0; S.data[1] = -vz; S.data[2] =  vy;
    S.data[3] =  vz;  S.data[4] =  0.0; S.data[5] = -vx;
    S.data[6] = -vy;  S.data[7] =  vx;  S.data[8] =  0.0;
    return S;
}

// Convert 3x3 rotation matrix to unit Hamilton quaternion using Shepperd's algorithm
// Matches src/ref/mathlib.py rot_matrix_to_quat
inline Quat from_rot_mat(const Mat3& R) {
    double r00 = R.data[0], r01 = R.data[1], r02 = R.data[2];
    double r10 = R.data[3], r11 = R.data[4], r12 = R.data[5];
    double r20 = R.data[6], r21 = R.data[7], r22 = R.data[8];

    double trace = r00 + r11 + r22;
    double w, x, y, z;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (r21 - r12) * s;
        y = (r02 - r20) * s;
        z = (r10 - r01) * s;
    } else if ((r00 > r11) && (r00 > r22)) {
        double rad = 1.0 + r00 - r11 - r22;
        if (rad < 0.0) rad = 0.0;
        double s = 2.0 * std::sqrt(rad);
        if (s < 1e-12) s = 1e-12;
        w = (r21 - r12) / s;
        x = 0.25 * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        double rad = 1.0 + r11 - r00 - r22;
        if (rad < 0.0) rad = 0.0;
        double s = 2.0 * std::sqrt(rad);
        if (s < 1e-12) s = 1e-12;
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25 * s;
        z = (r12 + r21) / s;
    } else {
        double rad = 1.0 + r22 - r00 - r11;
        if (rad < 0.0) rad = 0.0;
        double s = 2.0 * std::sqrt(rad);
        if (s < 1e-12) s = 1e-12;
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25 * s;
    }

    return normalize(Quat{w, x, y, z});
}

// Exponential map of rotation vector phi to unit Hamilton quaternion [w, x, y, z]
// Angle a = |phi|: exp(phi) = [cos(a/2), sin(a/2)*phi/a].
// For a < 1e-8, uses [1, phi/2] then normalizes.
// Matches SPEC §12.1 and src/ref/mathlib.py
inline Quat exp_map(const Vec3& phi) {
    double px = phi.data[0];
    double py = phi.data[1];
    double pz = phi.data[2];
    double a = std::sqrt(px * px + py * py + pz * pz);

    if (a < 1e-8) {
        return normalize(Quat{1.0, 0.5 * px, 0.5 * py, 0.5 * pz});
    }

    double half_a = 0.5 * a;
    double factor = std::sin(half_a) / a;
    return Quat{
        std::cos(half_a),
        factor * px,
        factor * py,
        factor * pz
    };
}

// Logarithmic map of unit Hamilton quaternion q = [w, x, y, z] to rotation vector phi
// Matches src/ref/mathlib.py quat_log
inline Vec3 log_map(const Quat& q_in) {
    double w = q_in.w;
    double x = q_in.x;
    double y = q_in.y;
    double z = q_in.z;

    // Ensure w >= 0 so that angle is in [0, pi]
    if (w < 0.0) {
        w = -w;
        x = -x;
        y = -y;
        z = -z;
    }

    double v_norm = std::sqrt(x * x + y * y + z * z);
    Vec3 phi;
    if (v_norm < 1e-8) {
        double w_clamped = (w > 1e-12) ? w : 1e-12;
        double scale = 2.0 / w_clamped;
        phi.data[0] = scale * x;
        phi.data[1] = scale * y;
        phi.data[2] = scale * z;
        return phi;
    }

    double angle = 2.0 * std::atan2(v_norm, w);
    double scale = angle / v_norm;
    phi.data[0] = scale * x;
    phi.data[1] = scale * y;
    phi.data[2] = scale * z;
    return phi;
}

// Enforce quaternion sign continuity: dot(q, q_prev) >= 0
// Matches SPEC §12.1 and src/ref/mathlib.py quat_sign_continuity
inline void ensure_continuity(Quat& q, const Quat& q_prev) {
    double dot = q.w * q_prev.w + q.x * q_prev.x + q.y * q_prev.y + q.z * q_prev.z;
    if (dot < 0.0) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }
}

inline Quat sign_continuity(const Quat& q, const Quat& q_prev) {
    Quat res = q;
    ensure_continuity(res, q_prev);
    return res;
}

} // namespace math

// Common type alias in global scope
using math::Quat;

#endif // QUAT_H
