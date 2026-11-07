#include "src/cpp/mat.h"
#include "src/cpp/quat.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_tests_run = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (!(cond)) { \
        std::printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
        g_tests_failed++; \
    } \
} while(0)

#define TEST_ASSERT_NEAR(val, exp, tol, msg) do { \
    g_tests_run++; \
    double diff = std::abs((val) - (exp)); \
    if (diff > (tol)) { \
        std::printf("FAIL [%s:%d]: %s (got %.15e, expected %.15e, diff %.15e > tol %.15e)\n", \
                    __FILE__, __LINE__, msg, (double)(val), (double)(exp), diff, (double)(tol)); \
        g_tests_failed++; \
    } \
} while(0)

using namespace math;

void test_mat3_basic() {
    std::printf("Running test_mat3_basic...\n");

    Mat3 Z = Mat3::zeros();
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(Z.data[i], 0.0, 1e-15, "Mat3::zeros() must be 0");
    }

    Mat3 I = Mat3::identity();
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            TEST_ASSERT_NEAR(I(r, c), expected, 1e-15, "Mat3::identity() diagonal check");
            TEST_ASSERT_NEAR(I[r][c], expected, 1e-15, "Mat3 operator[][] check");
        }
    }

    Mat3 A;
    A(0, 0) = 1.0; A(0, 1) = 2.0; A(0, 2) = 3.0;
    A(1, 0) = 4.0; A(1, 1) = 5.0; A(1, 2) = 6.0;
    A(2, 0) = 7.0; A(2, 1) = 8.0; A(2, 2) = 10.0; // det = -3.0

    Mat3 B;
    B(0, 0) = 9.0; B(0, 1) = 8.0; B(0, 2) = 7.0;
    B(1, 0) = 6.0; B(1, 1) = 5.0; B(1, 2) = 4.0;
    B(2, 0) = 3.0; B(2, 1) = 2.0; B(2, 2) = 1.0;

    // Addition & subtraction
    Mat3 S = A + B;
    Mat3 D = S - B;
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(D.data[i], A.data[i], 1e-14, "A + B - B == A");
    }

    // Scalar mult
    Mat3 Sm = A * 2.5;
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(Sm.data[i], A.data[i] * 2.5, 1e-14, "A * scalar");
    }

    // Transpose
    Mat3 At = A.transpose();
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            TEST_ASSERT_NEAR(At(c, r), A(r, c), 1e-15, "A.transpose()");
        }
    }

    // Multiplication with identity
    Mat3 AI = A * I;
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(AI.data[i], A.data[i], 1e-15, "A * I == A");
    }

    // Transpose product property: (A * B)^T == B^T * A^T
    Mat3 AB = A * B;
    Mat3 AB_t = AB.transpose();
    Mat3 Bt_At = B.transpose() * A.transpose();
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(AB_t.data[i], Bt_At.data[i], 1e-13, "(A * B)^T == B^T * A^T");
    }
}

void test_mat3_inv() {
    std::printf("Running test_mat3_inv...\n");

    Mat3 A;
    A(0, 0) = 1.0; A(0, 1) = 2.0; A(0, 2) = 3.0;
    A(1, 0) = 0.0; A(1, 1) = 1.0; A(1, 2) = 4.0;
    A(2, 0) = 5.0; A(2, 1) = 6.0; A(2, 2) = 0.0;

    // Determinant: 1*(0 - 24) - 2*(0 - 20) + 3*(0 - 5) = -24 + 40 - 15 = 1.0
    double det = det3(A);
    TEST_ASSERT_NEAR(det, 1.0, 1e-12, "det3(A) must equal 1.0");

    Mat3 A_inv;
    bool ok = inv(A, A_inv);
    TEST_ASSERT(ok, "inv(A) must report invertible");

    Mat3 prod = A * A_inv;
    Mat3 I = Mat3::identity();
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            TEST_ASSERT_NEAR(prod(r, c), I(r, c), 1e-12, "A * A_inv == I");
        }
    }

    // Singular matrix test: all zeros
    Mat3 Z = Mat3::zeros();
    Mat3 Z_inv;
    bool ok_z = inv(Z, Z_inv);
    TEST_ASSERT(!ok_z, "inv(Z) must report singular");
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT(!std::isnan(Z_inv.data[i]), "Z_inv must not have NaNs due to det floor");
        TEST_ASSERT(!std::isinf(Z_inv.data[i]), "Z_inv must not have Infs due to det floor");
    }
}

void test_mat15_ops() {
    std::printf("Running test_mat15_ops...\n");

    Mat15 I15 = Mat15::identity();
    Mat15 P = Mat15::zeros();
    for (size_t i = 0; i < 15; ++i) {
        for (size_t j = 0; j < 15; ++j) {
            P(i, j) = (double)(i * 15 + j) * 0.01;
        }
    }

    // P * I == P
    Mat15 PI = P * I15;
    for (size_t i = 0; i < 225; ++i) {
        TEST_ASSERT_NEAR(PI.data[i], P.data[i], 1e-14, "Mat15 P * I == P");
    }

    // Submatrix block test
    Mat3 block_in;
    block_in(0, 0) = 1.1; block_in(0, 1) = 2.2; block_in(0, 2) = 3.3;
    block_in(1, 0) = 4.4; block_in(1, 1) = 5.5; block_in(1, 2) = 6.6;
    block_in(2, 0) = 7.7; block_in(2, 1) = 8.8; block_in(2, 2) = 9.9;

    P.set_block<3, 3>(3, 6, block_in);
    Mat3 block_out = P.block<3, 3>(3, 6);
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            TEST_ASSERT_NEAR(block_out(r, c), block_in(r, c), 1e-15, "block<3, 3> read/write");
        }
    }

    // Covariance symmetrise and conditioning
    Mat15 cov = Mat15::identity() * 0.5;
    cov(0, 1) = 0.1;
    cov(1, 0) = 0.2; // asymmetric
    cov(2, 2) = 1e-15; // below floor

    condition_covariance(cov, 1e-12, 1e6);
    TEST_ASSERT_NEAR(cov(0, 1), 0.15, 1e-15, "Covariance symmetrised element (0, 1)");
    TEST_ASSERT_NEAR(cov(1, 0), 0.15, 1e-15, "Covariance symmetrised element (1, 0)");
    TEST_ASSERT_NEAR(cov(2, 2), 1e-12, 1e-15, "Covariance diagonal floored");
}

void test_skew() {
    std::printf("Running test_skew...\n");

    Vec3 v = make_vec3(1.0, 2.0, 3.0);
    Vec3 u = make_vec3(4.0, 5.0, 6.0);

    Mat3 S = skew(v);

    // Antisymmetry: S^T == -S
    Mat3 St = S.transpose();
    Mat3 neg_S = -S;
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(St.data[i], neg_S.data[i], 1e-15, "skew(v)^T == -skew(v)");
    }

    // Cross product equivalence: skew(v) * u == v x u
    Vec3 Su = S * u;
    Vec3 v_x_u = cross(v, u);
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(Su(i), v_x_u(i), 1e-15, "skew(v) * u == v x u");
    }

    // S(v) * v == 0
    Vec3 Sv = S * v;
    for (size_t i = 0; i < 3; ++i) {
        TEST_ASSERT_NEAR(Sv(i), 0.0, 1e-15, "skew(v) * v == 0");
    }
}

void test_quat_mult_hamilton() {
    std::printf("Running test_quat_mult_hamilton...\n");

    Quat q_id = Quat::identity();
    Quat q{0.5, 0.5, 0.5, 0.5};

    Quat q_res1 = q * q_id;
    Quat q_res2 = q_id * q;
    TEST_ASSERT_NEAR(q_res1.w, q.w, 1e-15, "q * I == q (w)");
    TEST_ASSERT_NEAR(q_res1.x, q.x, 1e-15, "q * I == q (x)");
    TEST_ASSERT_NEAR(q_res1.y, q.y, 1e-15, "q * I == q (y)");
    TEST_ASSERT_NEAR(q_res1.z, q.z, 1e-15, "q * I == q (z)");
    TEST_ASSERT_NEAR(q_res2.w, q.w, 1e-15, "I * q == q (w)");

    // Conjugate inverse: q * q* == I
    Quat q_c = conjugate(q);
    Quat prod_c = q * q_c;
    TEST_ASSERT_NEAR(prod_c.w, 1.0, 1e-15, "q * q* == I (w)");
    TEST_ASSERT_NEAR(prod_c.x, 0.0, 1e-15, "q * q* == I (x)");
    TEST_ASSERT_NEAR(prod_c.y, 0.0, 1e-15, "q * q* == I (y)");
    TEST_ASSERT_NEAR(prod_c.z, 0.0, 1e-15, "q * q* == I (z)");

    // Basis rules: i*j = k, j*k = i, k*i = j, i*i = -1
    Quat qi{0.0, 1.0, 0.0, 0.0};
    Quat qj{0.0, 0.0, 1.0, 0.0};
    Quat qk{0.0, 0.0, 0.0, 1.0};

    Quat ij = qi * qj;
    TEST_ASSERT_NEAR(ij.w, qk.w, 1e-15, "i * j == k (w)");
    TEST_ASSERT_NEAR(ij.x, qk.x, 1e-15, "i * j == k (x)");
    TEST_ASSERT_NEAR(ij.y, qk.y, 1e-15, "i * j == k (y)");
    TEST_ASSERT_NEAR(ij.z, qk.z, 1e-15, "i * j == k (z)");

    Quat jk = qj * qk;
    TEST_ASSERT_NEAR(jk.x, qi.x, 1e-15, "j * k == i (x)");

    Quat ki = qk * qi;
    TEST_ASSERT_NEAR(ki.y, qj.y, 1e-15, "k * i == j (y)");

    Quat ii = qi * qi;
    TEST_ASSERT_NEAR(ii.w, -1.0, 1e-15, "i * i == -1 (w)");
    TEST_ASSERT_NEAR(ii.x, 0.0, 1e-15, "i * i == -1 (x)");

    // Associativity: (q1 * q2) * q3 == q1 * (q2 * q3)
    Quat q1 = normalize(Quat{1.0, 2.0, 3.0, 4.0});
    Quat q2 = normalize(Quat{0.5, -0.5, 1.0, -0.2});
    Quat q3 = normalize(Quat{2.0, 0.1, -0.3, 0.4});

    Quat lhs = (q1 * q2) * q3;
    Quat rhs = q1 * (q2 * q3);
    TEST_ASSERT_NEAR(lhs.w, rhs.w, 1e-14, "Quat mult associative (w)");
    TEST_ASSERT_NEAR(lhs.x, rhs.x, 1e-14, "Quat mult associative (x)");
    TEST_ASSERT_NEAR(lhs.y, rhs.y, 1e-14, "Quat mult associative (y)");
    TEST_ASSERT_NEAR(lhs.z, rhs.z, 1e-14, "Quat mult associative (z)");

    // Norm multiplicativity: |q1 * q2| == |q1| * |q2|
    Quat raw1{1.2, -0.3, 0.5, 0.8};
    Quat raw2{0.4, 1.1, -0.9, 0.2};
    Quat raw_prod = raw1 * raw2;
    TEST_ASSERT_NEAR(raw_prod.norm(), raw1.norm() * raw2.norm(), 1e-14, "|q1 * q2| == |q1| * |q2|");
}

void test_quat_normalize() {
    std::printf("Running test_quat_normalize...\n");

    Quat q{2.0, 0.0, 0.0, 0.0};
    Quat q_n = normalize(q);
    TEST_ASSERT_NEAR(q_n.norm(), 1.0, 1e-15, "norm == 1");
    TEST_ASSERT_NEAR(q_n.w, 1.0, 1e-15, "q_n.w == 1");

    // Tiny quaternion: floor protection
    Quat q_tiny{1e-15, 0.0, 0.0, 0.0};
    Quat q_floor = normalize(q_tiny);
    TEST_ASSERT(!std::isnan(q_floor.w), "normalize floor must not produce NaN");
    TEST_ASSERT(!std::isinf(q_floor.w), "normalize floor must not produce Inf");
    TEST_ASSERT(q_floor.norm() > 0.0, "normalize floor norm > 0");
}

void test_quat_exp_and_log() {
    std::printf("Running test_quat_exp_and_log...\n");

    // Zero vector
    Vec3 zero = make_vec3(0.0, 0.0, 0.0);
    Quat q_zero = exp_map(zero);
    TEST_ASSERT_NEAR(q_zero.w, 1.0, 1e-15, "exp(0).w == 1");
    TEST_ASSERT_NEAR(q_zero.x, 0.0, 1e-15, "exp(0).x == 0");
    TEST_ASSERT_NEAR(q_zero.y, 0.0, 1e-15, "exp(0).y == 0");
    TEST_ASSERT_NEAR(q_zero.z, 0.0, 1e-15, "exp(0).z == 0");

    // Near-zero vector < 1e-8 (Taylor branch)
    Vec3 tiny = make_vec3(1e-9, 2e-9, -1e-9);
    Quat q_tiny = exp_map(tiny);
    TEST_ASSERT(!std::isnan(q_tiny.w), "exp(tiny) finite");
    TEST_ASSERT_NEAR(q_tiny.norm(), 1.0, 1e-15, "exp(tiny) unit norm");

    // 90 degrees around z
    Vec3 phi_z = make_vec3(0.0, 0.0, M_PI / 2.0);
    Quat q_z = exp_map(phi_z);
    TEST_ASSERT_NEAR(q_z.w, std::cos(M_PI / 4.0), 1e-15, "exp(90 deg z).w");
    TEST_ASSERT_NEAR(q_z.x, 0.0, 1e-15, "exp(90 deg z).x");
    TEST_ASSERT_NEAR(q_z.y, 0.0, 1e-15, "exp(90 deg z).y");
    TEST_ASSERT_NEAR(q_z.z, std::sin(M_PI / 4.0), 1e-15, "exp(90 deg z).z");

    // Exp - Log roundtrip
    Vec3 phi = make_vec3(0.2, -0.3, 0.4);
    Quat q_phi = exp_map(phi);
    Vec3 phi_rec = log_map(q_phi);
    TEST_ASSERT_NEAR(phi_rec(0), phi(0), 1e-12, "exp-log roundtrip (x)");
    TEST_ASSERT_NEAR(phi_rec(1), phi(1), 1e-12, "exp-log roundtrip (y)");
    TEST_ASSERT_NEAR(phi_rec(2), phi(2), 1e-12, "exp-log roundtrip (z)");

    // Log of -q: same rotation vector
    Vec3 phi_neg = log_map(-q_phi);
    TEST_ASSERT_NEAR(phi_neg(0), phi(0), 1e-12, "log(-q) == log(q) (x)");
    TEST_ASSERT_NEAR(phi_neg(1), phi(1), 1e-12, "log(-q) == log(q) (y)");
    TEST_ASSERT_NEAR(phi_neg(2), phi(2), 1e-12, "log(-q) == log(q) (z)");
}

void test_rot_matrix_cookbook_spec() {
    std::printf("Running test_rot_matrix_cookbook_spec...\n");

    // SPEC §12.1: 90° about z gives q=[0.70710678,0,0,0.70710678]
    double val = std::sqrt(0.5);
    Quat q_90z{val, 0.0, 0.0, val};
    Mat3 R = to_rot_mat(q_90z);

    // R^T * R == I
    Mat3 RtR = R.transpose() * R;
    Mat3 I = Mat3::identity();
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(RtR.data[i], I.data[i], 1e-14, "R^T * R == I");
    }
    TEST_ASSERT_NEAR(det3(R), 1.0, 1e-14, "det(R) == 1");

    // R(q) * v == q (x) [0, v] (x) q*
    Vec3 v = make_vec3(1.0, 2.0, 3.0);
    Vec3 Rv = R * v;
    Vec3 Rv_method = rotate(q_90z, v);
    TEST_ASSERT_NEAR(Rv(0), Rv_method(0), 1e-15, "rotate() == R * v (x)");
    TEST_ASSERT_NEAR(Rv(1), Rv_method(1), 1e-15, "rotate() == R * v (y)");
    TEST_ASSERT_NEAR(Rv(2), Rv_method(2), 1e-15, "rotate() == R * v (z)");

    // Hamilton sandwich computation
    Quat qv{0.0, v(0), v(1), v(2)};
    Quat q_rot_v = q_90z * qv * conjugate(q_90z);
    TEST_ASSERT_NEAR(Rv(0), q_rot_v.x, 1e-14, "R(q) v == q [0,v] q* (x)");
    TEST_ASSERT_NEAR(Rv(1), q_rot_v.y, 1e-14, "R(q) v == q [0,v] q* (y)");
    TEST_ASSERT_NEAR(Rv(2), q_rot_v.z, 1e-14, "R(q) v == q [0,v] q* (z)");

    // Shepperd roundtrip
    Quat q_rec = from_rot_mat(R);
    double dot = q_rec.w * q_90z.w + q_rec.x * q_90z.x + q_rec.y * q_90z.y + q_rec.z * q_90z.z;
    double sign = (dot >= 0.0) ? 1.0 : -1.0;
    TEST_ASSERT_NEAR(q_rec.w * sign, q_90z.w, 1e-14, "Shepperd roundtrip (w)");
    TEST_ASSERT_NEAR(q_rec.x * sign, q_90z.x, 1e-14, "Shepperd roundtrip (x)");
    TEST_ASSERT_NEAR(q_rec.y * sign, q_90z.y, 1e-14, "Shepperd roundtrip (y)");
    TEST_ASSERT_NEAR(q_rec.z * sign, q_90z.z, 1e-14, "Shepperd roundtrip (z)");
}

void test_rot_matrix_shepperd_branches() {
    std::printf("Running test_rot_matrix_shepperd_branches...\n");

    // Branch 1: trace > 0 (identity)
    Mat3 R_id = Mat3::identity();
    Quat q_id = from_rot_mat(R_id);
    TEST_ASSERT_NEAR(q_id.w, 1.0, 1e-14, "Branch 1: q_id.w");
    TEST_ASSERT_NEAR(q_id.x, 0.0, 1e-14, "Branch 1: q_id.x");
    TEST_ASSERT_NEAR(q_id.y, 0.0, 1e-14, "Branch 1: q_id.y");
    TEST_ASSERT_NEAR(q_id.z, 0.0, 1e-14, "Branch 1: q_id.z");

    // Branch 2: r00 dominant (180 deg rotation about x)
    // R = diag(1, -1, -1)
    Mat3 R_x180 = Mat3::zeros();
    R_x180(0, 0) =  1.0;
    R_x180(1, 1) = -1.0;
    R_x180(2, 2) = -1.0;
    Quat q_x180 = from_rot_mat(R_x180);
    TEST_ASSERT_NEAR(std::abs(q_x180.x), 1.0, 1e-14, "Branch 2: |q.x| == 1");
    TEST_ASSERT_NEAR(std::abs(q_x180.w), 0.0, 1e-14, "Branch 2: |q.w| == 0");
    Mat3 R_x180_rec = to_rot_mat(q_x180);
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(R_x180_rec.data[i], R_x180.data[i], 1e-14, "Branch 2: R roundtrip");
    }

    // Branch 3: r11 dominant (180 deg rotation about y)
    // R = diag(-1, 1, -1)
    Mat3 R_y180 = Mat3::zeros();
    R_y180(0, 0) = -1.0;
    R_y180(1, 1) =  1.0;
    R_y180(2, 2) = -1.0;
    Quat q_y180 = from_rot_mat(R_y180);
    TEST_ASSERT_NEAR(std::abs(q_y180.y), 1.0, 1e-14, "Branch 3: |q.y| == 1");
    TEST_ASSERT_NEAR(std::abs(q_y180.w), 0.0, 1e-14, "Branch 3: |q.w| == 0");
    Mat3 R_y180_rec = to_rot_mat(q_y180);
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(R_y180_rec.data[i], R_y180.data[i], 1e-14, "Branch 3: R roundtrip");
    }

    // Branch 4: r22 dominant (180 deg rotation about z)
    // R = diag(-1, -1, 1)
    Mat3 R_z180 = Mat3::zeros();
    R_z180(0, 0) = -1.0;
    R_z180(1, 1) = -1.0;
    R_z180(2, 2) =  1.0;
    Quat q_z180 = from_rot_mat(R_z180);
    TEST_ASSERT_NEAR(std::abs(q_z180.z), 1.0, 1e-14, "Branch 4: |q.z| == 1");
    TEST_ASSERT_NEAR(std::abs(q_z180.w), 0.0, 1e-14, "Branch 4: |q.w| == 0");
    Mat3 R_z180_rec = to_rot_mat(q_z180);
    for (size_t i = 0; i < 9; ++i) {
        TEST_ASSERT_NEAR(R_z180_rec.data[i], R_z180.data[i], 1e-14, "Branch 4: R roundtrip");
    }
}

void test_quat_sign_continuity() {
    std::printf("Running test_quat_sign_continuity...\n");

    Quat q_prev{1.0, 0.0, 0.0, 0.0};
    Quat q_flipped{-0.999, 0.01, 0.0, 0.0};

    ensure_continuity(q_flipped, q_prev);

    double dot = q_flipped.w * q_prev.w + q_flipped.x * q_prev.x +
                 q_flipped.y * q_prev.y + q_flipped.z * q_prev.z;
    TEST_ASSERT(dot > 0.0, "ensure_continuity dot product must be > 0");
    TEST_ASSERT_NEAR(q_flipped.w, 0.999, 1e-15, "flipped sign preserved (w)");
    TEST_ASSERT_NEAR(q_flipped.x, -0.01, 1e-15, "flipped sign preserved (x)");
}

int main() {
    std::printf("=== Starting C++ Math Unit Tests ===\n");

    test_mat3_basic();
    test_mat3_inv();
    test_mat15_ops();
    test_skew();
    test_quat_mult_hamilton();
    test_quat_normalize();
    test_quat_exp_and_log();
    test_rot_matrix_cookbook_spec();
    test_rot_matrix_shepperd_branches();
    test_quat_sign_continuity();

    std::printf("=== Summary: %d checks run, %d failures ===\n", g_tests_run, g_tests_failed);

    if (g_tests_failed == 0) {
        std::printf("ALL MATH TESTS PASSED.\n");
        return 0;
    } else {
        std::printf("TESTS FAILED!\n");
        return 1;
    }
}
