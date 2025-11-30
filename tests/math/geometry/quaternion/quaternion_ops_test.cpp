#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion Ops Tests (quaternion_ops.hpp)
// ============================================================================

TEST_CASE("Quaternion - Binary Addition", "[quaternion][ops][binary]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion result = q1 + q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(10.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(12.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Addition Commutativity", "[quaternion][ops][binary]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion r1 = q1 + q2;
    Quaternion r2 = q2 + q1;
    
    REQUIRE(r1 == r2);
}

TEST_CASE("Quaternion - Binary Subtraction", "[quaternion][ops][binary]") {
    Quaternion q1(5.0f, 7.0f, 9.0f, 11.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = q1 - q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(7.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Subtraction Self", "[quaternion][ops][binary]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = q - q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Multiplication", "[quaternion][ops][binary]") {
    Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);  // j
    
    Quaternion result = qi * qj;  // i * j = k
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Multiplication Quaternion * Scalar", "[quaternion][ops][binary]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = q * 2.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Multiplication Scalar * Quaternion", "[quaternion][ops][binary]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = 2.0f * q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Division Quaternion / Scalar", "[quaternion][ops][binary]") {
    Quaternion q(4.0f, 6.0f, 8.0f, 10.0f);
    
    Quaternion result = q / 2.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Binary Division Scalar / Quaternion", "[quaternion][ops][binary]") {
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);  // Real quaternion
    
    Quaternion result = 2.0f / q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Unary Negation", "[quaternion][ops][unary]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = -q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-2.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-3.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-4.0f, 1e-6));
}

TEST_CASE("Quaternion - Double Negation", "[quaternion][ops][unary]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = -(-q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Equality True", "[quaternion][ops][equality]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE(q1 == q2);
}

TEST_CASE("Quaternion - Equality False", "[quaternion][ops][equality]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 5.0f);
    
    REQUIRE_FALSE(q1 == q2);
}

TEST_CASE("Quaternion - Inequality", "[quaternion][ops][equality]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 5.0f);
    
    REQUIRE(q1 != q2);
}

TEST_CASE("Quaternion - Conjugate Basic", "[quaternion][ops][conj]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = conj(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-2.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-3.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-4.0f, 1e-6));
}

TEST_CASE("Quaternion - Double Conjugate", "[quaternion][ops][conj]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = conj(conj(q));
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Conjugate of Real", "[quaternion][ops][conj]") {
    Quaternion q(5.0f, 0.0f, 0.0f, 0.0f);
    
    Quaternion result = conj(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Norm Unit Quaternion", "[quaternion][ops][norm]") {
    Quaternion q = Quaternion::eye();
    
    float result = norm(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Norm Basic", "[quaternion][ops][norm]") {
    Quaternion q(1.0f, 2.0f, 2.0f, 0.0f);  // sqrt(1+4+4) = 3
    
    float result = norm(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Quaternion - Norm Squared", "[quaternion][ops][norm]") {
    Quaternion q(1.0f, 2.0f, 2.0f, 0.0f);
    
    float result = normSqr(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(9.0f, 1e-6));
}

TEST_CASE("Quaternion - Norm Squared Consistency", "[quaternion][ops][norm]") {
    Quaternion q(2.0f, 3.0f, 4.0f, 5.0f);
    
    float n = norm(q);
    float nSq = normSqr(q);
    
    REQUIRE_THAT(n * n, Catch::Matchers::WithinAbs(nSq, 1e-6));
}

TEST_CASE("Quaternion - Normalize Unit", "[quaternion][ops][normalize]") {
    Quaternion q = Quaternion::eye();
    
    Quaternion result = normalize(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Normalize Creates Unit", "[quaternion][ops][normalize]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion result = normalize(q);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Normalize Zero Returns Identity", "[quaternion][ops][normalize]") {
    Quaternion q = Quaternion::zero();
    
    Quaternion result = normalize(q);
    
    REQUIRE(result == Quaternion::eye());
}

TEST_CASE("Quaternion - Inverse Identity", "[quaternion][ops][inv]") {
    Quaternion q = Quaternion::eye();
    
    Quaternion result = inv(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Inverse Multiplication Property", "[quaternion][ops][inv]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion qInv = inv(q);
    
    Quaternion result = q * qInv;
    
    // q * q^-1 ≈ 1
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Quaternion - Double Inverse", "[quaternion][ops][inv]") {
    Quaternion q(2.0f, 3.0f, 4.0f, 5.0f);
    
    Quaternion result = inv(inv(q));
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-4f));
}

TEST_CASE("Quaternion - Dot Product Same", "[quaternion][ops][dot]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    float result = dot(q, q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(normSqr(q), 1e-6));
}

TEST_CASE("Quaternion - Dot Product Orthogonal", "[quaternion][ops][dot]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(0.0f, 1.0f, 0.0f, 0.0f);
    
    float result = dot(q1, q2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Dot Product Commutativity", "[quaternion][ops][dot]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    float d1 = dot(q1, q2);
    float d2 = dot(q2, q1);
    
    REQUIRE_THAT(d1, Catch::Matchers::WithinAbs(d2, 1e-6));
}

TEST_CASE("Quaternion - Log of Identity", "[quaternion][ops][log]") {
    Quaternion q = Quaternion::eye();
    
    Quaternion result = log(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Exp of Zero", "[quaternion][ops][exp]") {
    Quaternion q = Quaternion::zero();
    
    Quaternion result = exp(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Exp Log Inverse", "[quaternion][ops][exp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    
    Quaternion result = exp(log(q));
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-4f));
}

TEST_CASE("Quaternion - Power Zero", "[quaternion][ops][pow]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = pow(q, 0.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("Quaternion - Power One", "[quaternion][ops][pow]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = pow(q, 1.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-4f));
}

TEST_CASE("Quaternion - Power Two Matches Multiplication", "[quaternion][ops][pow]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    
    Quaternion squared = q * q;
    Quaternion powered = pow(q, 2.0f);
    
    REQUIRE_THAT(squared.w(), Catch::Matchers::WithinAbs(powered.w(), 1e-4f));
    REQUIRE_THAT(squared.x(), Catch::Matchers::WithinAbs(powered.x(), 1e-4f));
    REQUIRE_THAT(squared.y(), Catch::Matchers::WithinAbs(powered.y(), 1e-4f));
    REQUIRE_THAT(squared.z(), Catch::Matchers::WithinAbs(powered.z(), 1e-4f));
}


TEST_CASE("Quaternion - Rotate Vector Identity", "[quaternion][ops][rotate]") {
    Quaternion q = Quaternion::eye();
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(v.x(), 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(v.y(), 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(v.z(), 1e-6));
}

TEST_CASE("Quaternion - Rotate Vector 90-deg Z-Axis", "[quaternion][ops][rotate]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = rotate(q, v);
    
    // (1,0,0) rotated 90° around z should be (0,1,0)
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Quaternion - Rotate Vector 180-deg X-Axis", "[quaternion][ops][rotate]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI);
    Vector<3> v = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = rotate(q, v);
    
    // (0,1,0) rotated 180° around x should be (0,-1,0)
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-1.0f, 1e-5f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Quaternion - Rotate Preserves Magnitude", "[quaternion][ops][rotate]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, M_PI / 3.0f);
    Vector<3> v = {3.0f, 4.0f, 5.0f};
    
    Vector<3> result = rotate(q, v);
    
    float origMag = std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
    float rotMag = std::sqrt(result.x()*result.x() + result.y()*result.y() + result.z()*result.z());
    
    REQUIRE_THAT(rotMag, Catch::Matchers::WithinAbs(origMag, 1e-4f));
}

TEST_CASE("Quaternion - Conjugate Multiplication Property", "[quaternion][integration]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion qConj = conj(q);
    
    Quaternion result = q * qConj;
    
    // q * q̄ = |q|²
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(normSqr(q), 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Quaternion - Normalized Rotation Preserves Norm", "[quaternion][integration]") {
    Quaternion q(2.0f, 3.0f, 4.0f, 5.0f);
    Quaternion qNorm = normalize(q);
    
    REQUIRE_THAT(norm(qNorm), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Composition of Rotations", "[quaternion][integration]") {
    // 90° around Z, then 90° around X
    Quaternion qZ = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Quaternion qX = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 2.0f);
    
    Quaternion combined = qX * qZ;
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> rotated = rotate(combined, v);
    
    // Should end up at (0, 0, 1)
    REQUIRE_THAT(rotated.z(), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("Quaternion - Inverse Rotation", "[quaternion][integration]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    Quaternion qInv = inv(q);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> rotated = rotate(q, v);
    Vector<3> unrotated = rotate(qInv, rotated);
    
    REQUIRE_THAT(unrotated.x(), Catch::Matchers::WithinAbs(v.x(), 1e-4f));
    REQUIRE_THAT(unrotated.y(), Catch::Matchers::WithinAbs(v.y(), 1e-4f));
    REQUIRE_THAT(unrotated.z(), Catch::Matchers::WithinAbs(v.z(), 1e-4f));
}