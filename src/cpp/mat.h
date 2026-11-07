#ifndef MAT_H
#define MAT_H

#include <cmath>
#include <cstddef>

namespace math {

template <size_t R, size_t C>
struct Mat {
    double data[R * C];

    constexpr Mat() : data{} {}

    static Mat zeros() {
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = 0.0;
        }
        return res;
    }

    static Mat identity() {
        Mat res;
        constexpr size_t n = (R < C) ? R : C;
        for (size_t i = 0; i < n; ++i) {
            res.data[i * C + i] = 1.0;
        }
        return res;
    }

    inline double& operator()(size_t r, size_t c = 0) {
        return data[r * C + c];
    }

    inline const double& operator()(size_t r, size_t c = 0) const {
        return data[r * C + c];
    }

    inline double* operator[](size_t r) {
        return &data[r * C];
    }

    inline const double* operator[](size_t r) const {
        return &data[r * C];
    }

    // Element-wise addition
    Mat operator+(const Mat& other) const {
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = data[i] + other.data[i];
        }
        return res;
    }

    Mat& operator+=(const Mat& other) {
        for (size_t i = 0; i < R * C; ++i) {
            data[i] += other.data[i];
        }
        return *this;
    }

    // Element-wise subtraction
    Mat operator-(const Mat& other) const {
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = data[i] - other.data[i];
        }
        return res;
    }

    Mat& operator-=(const Mat& other) {
        for (size_t i = 0; i < R * C; ++i) {
            data[i] -= other.data[i];
        }
        return *this;
    }

    Mat operator-() const {
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = -data[i];
        }
        return res;
    }

    // Scalar multiplication
    Mat operator*(double s) const {
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = data[i] * s;
        }
        return res;
    }

    Mat& operator*=(double s) {
        for (size_t i = 0; i < R * C; ++i) {
            data[i] *= s;
        }
        return *this;
    }

    friend Mat operator*(double s, const Mat& m) {
        return m * s;
    }

    // Scalar division
    Mat operator/(double s) const {
        double inv_s = 1.0 / s;
        Mat res;
        for (size_t i = 0; i < R * C; ++i) {
            res.data[i] = data[i] * inv_s;
        }
        return res;
    }

    Mat& operator/=(double s) {
        double inv_s = 1.0 / s;
        for (size_t i = 0; i < R * C; ++i) {
            data[i] *= inv_s;
        }
        return *this;
    }

    // Matrix-matrix multiplication
    template <size_t K>
    Mat<R, K> operator*(const Mat<C, K>& other) const {
        Mat<R, K> res;
        for (size_t r = 0; r < R; ++r) {
            for (size_t c = 0; c < C; ++c) {
                double a_rc = data[r * C + c];
                for (size_t k = 0; k < K; ++k) {
                    res.data[r * K + k] += a_rc * other.data[c * K + k];
                }
            }
        }
        return res;
    }

    // Transpose
    Mat<C, R> transpose() const {
        Mat<C, R> res;
        for (size_t r = 0; r < R; ++r) {
            for (size_t c = 0; c < C; ++c) {
                res.data[c * R + r] = data[r * C + c];
            }
        }
        return res;
    }

    // Submatrix block extraction and insertion
    template <size_t SubR, size_t SubC>
    Mat<SubR, SubC> block(size_t start_r, size_t start_c) const {
        Mat<SubR, SubC> res;
        for (size_t r = 0; r < SubR; ++r) {
            for (size_t c = 0; c < SubC; ++c) {
                res.data[r * SubC + c] = data[(start_r + r) * C + (start_c + c)];
            }
        }
        return res;
    }

    template <size_t SubR, size_t SubC>
    void set_block(size_t start_r, size_t start_c, const Mat<SubR, SubC>& sub) {
        for (size_t r = 0; r < SubR; ++r) {
            for (size_t c = 0; c < SubC; ++c) {
                data[(start_r + r) * C + (start_c + c)] = sub.data[r * SubC + c];
            }
        }
    }

    // Vector helpers (applicable when C == 1)
    double dot(const Mat& other) const {
        static_assert(C == 1, "dot product only defined for column vectors");
        double sum = 0.0;
        for (size_t i = 0; i < R; ++i) {
            sum += data[i] * other.data[i];
        }
        return sum;
    }

    double squared_norm() const {
        double sum = 0.0;
        for (size_t i = 0; i < R * C; ++i) {
            sum += data[i] * data[i];
        }
        return sum;
    }

    double norm() const {
        return std::sqrt(squared_norm());
    }
};

// Type aliases
using Vec3 = Mat<3, 1>;
using Mat3 = Mat<3, 3>;
using Vec15 = Mat<15, 1>;
using Mat15 = Mat<15, 15>;
using Mat3x15 = Mat<3, 15>;
using Mat15x3 = Mat<15, 3>;

// Helper to construct a Vec3
inline Vec3 make_vec3(double x, double y, double z) {
    Vec3 v;
    v.data[0] = x;
    v.data[1] = y;
    v.data[2] = z;
    return v;
}

// 3D vector cross product
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    Vec3 res;
    res.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1];
    res.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2];
    res.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0];
    return res;
}

// Determinant of 3x3 matrix
inline double det3(const Mat3& M) {
    double a00 = M.data[0], a01 = M.data[1], a02 = M.data[2];
    double a10 = M.data[3], a11 = M.data[4], a12 = M.data[5];
    double a20 = M.data[6], a21 = M.data[7], a22 = M.data[8];

    double c00 = a11 * a22 - a12 * a21;
    double c01 = -(a10 * a22 - a12 * a20);
    double c02 = a10 * a21 - a11 * a20;

    return a00 * c00 + a01 * c01 + a02 * c02;
}

// Analytic 3x3 matrix inversion with closed form + floor (SPEC §12.5)
inline bool inv(const Mat3& M, Mat3& out, double det_floor = 1e-12) {
    double a00 = M.data[0], a01 = M.data[1], a02 = M.data[2];
    double a10 = M.data[3], a11 = M.data[4], a12 = M.data[5];
    double a20 = M.data[6], a21 = M.data[7], a22 = M.data[8];

    double c00 = a11 * a22 - a12 * a21;
    double c01 = -(a10 * a22 - a12 * a20);
    double c02 = a10 * a21 - a11 * a20;

    double c10 = -(a01 * a22 - a02 * a21);
    double c11 = a00 * a22 - a02 * a20;
    double c12 = -(a00 * a21 - a01 * a20);

    double c20 = a01 * a12 - a02 * a11;
    double c21 = -(a00 * a12 - a02 * a10);
    double c22 = a00 * a11 - a01 * a10;

    double det = a00 * c00 + a01 * c01 + a02 * c02;
    bool is_invertible = (std::abs(det) >= det_floor);

    if (std::abs(det) < det_floor) {
        det = (det >= 0.0) ? det_floor : -det_floor;
    }

    double inv_det = 1.0 / det;

    out.data[0] = c00 * inv_det;
    out.data[1] = c10 * inv_det;
    out.data[2] = c20 * inv_det;

    out.data[3] = c01 * inv_det;
    out.data[4] = c11 * inv_det;
    out.data[5] = c21 * inv_det;

    out.data[6] = c02 * inv_det;
    out.data[7] = c12 * inv_det;
    out.data[8] = c22 * inv_det;

    return is_invertible;
}

inline Mat3 inv3(const Mat3& M, double det_floor = 1e-12) {
    Mat3 out;
    inv(M, out, det_floor);
    return out;
}

// Covariance helpers: symmetrise and floor/cap diagonal
template <size_t N>
inline void symmetrise(Mat<N, N>& P) {
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            double avg = 0.5 * (P.data[i * N + j] + P.data[j * N + i]);
            P.data[i * N + j] = avg;
            P.data[j * N + i] = avg;
        }
    }
}

template <size_t N>
inline void condition_covariance(Mat<N, N>& P, double diag_floor = 1e-12, double diag_cap = 1e6) {
    symmetrise(P);
    for (size_t i = 0; i < N; ++i) {
        if (P.data[i * N + i] < diag_floor) {
            P.data[i * N + i] = diag_floor;
        } else if (P.data[i * N + i] > diag_cap) {
            P.data[i * N + i] = diag_cap;
        }
    }
}

} // namespace math

// Common type aliases in global scope
using math::Mat;
using math::Vec3;
using math::Mat3;
using math::Vec15;
using math::Mat15;
using math::Mat3x15;
using math::Mat15x3;

#endif // MAT_H
