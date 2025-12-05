#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion Core Tests (quaternion.hpp)
// ============================================================================

TEST_CASE("Quaternion - Default Zero-Constructor", "[quaternion][core]") {
    Quaternion q;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Scalar Only Constructor", "[quaternion][core]") {
    Quaternion q(2.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Full Constructor", "[quaternion][core]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Quaternion - Negative Values Constructor", "[quaternion][core]") {
    Quaternion q(-1.0f, -2.0f, -3.0f, -4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(-2.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(-3.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(-4.0f, 1e-6));
}

TEST_CASE("Quaternion - Zero Factory", "[quaternion][core][factory]") {
    Quaternion q = Quaternion::zero();
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Identity Factory", "[quaternion][core][factory]") {
    Quaternion q = Quaternion::eye();
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Pure Quaternion Factory", "[quaternion][core][factory]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Quaternion q = Quaternion::pure(v);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Quaternion - From Axis-Angle X-Axis 90-deg", "[quaternion][core][factory]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 2.0f;
    
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    
    // q = cos(π/4) + sin(π/4)i = √2/2 + √2/2 i
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Axis-Angle Y-Axis 180-deg", "[quaternion][core][factory]") {
    Vector<3> axis = {0.0f, 1.0f, 0.0f};
    float angle = M_PI;
    
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    
    // q = cos(π/2) + sin(π/2)j = 0 + j
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Axis-Angle Zero Rotation", "[quaternion][core][factory]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = 0.0f;
    
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Axis-Angle Unnormalized Axis", "[quaternion][core][factory]") {
    Vector<3> axis = {2.0f, 0.0f, 0.0f};  // Not normalized
    float angle = M_PI / 2.0f;
    
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    
    // Should auto-normalize axis
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(M_PI / 4.0f), 1e-6));
}

TEST_CASE("Quaternion - From Rotation Vector Zero", "[quaternion][core][factory]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    Quaternion q = Quaternion::fromRotationVector(v);
    
    // Should return identity
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Rotation Vector X-Axis", "[quaternion][core][factory]") {
    Vector<3> v = {M_PI / 2.0f, 0.0f, 0.0f};  // 90° around x-axis
    
    Quaternion q = Quaternion::fromRotationVector(v);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Euler Angles Zero", "[quaternion][core][factory]") {
    Quaternion q = Quaternion::fromEuler(0.0f, 0.0f, 0.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - From Euler Angles Roll Only", "[quaternion][core][factory]") {
    Quaternion q = Quaternion::fromEuler(M_PI / 2.0f, 0.0f, 0.0f);
    
    // 90° roll around x-axis
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(M_PI / 4.0f), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Set W Component", "[quaternion][core][accessor]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    q.w(5.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
}

TEST_CASE("Quaternion - Set X Component", "[quaternion][core][accessor]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    q.x(5.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Set Y Component", "[quaternion][core][accessor]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    q.y(5.0f);
    
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Set Z Component", "[quaternion][core][accessor]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    q.z(5.0f);
    
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Set All Components", "[quaternion][core][accessor]") {
    Quaternion q;
    q.w(1.0f);
    q.x(2.0f);
    q.y(3.0f);
    q.z(4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Quaternion - Addition", "[quaternion][core][ops]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    q1 += q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(10.0f, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(12.0f, 1e-6));
}

TEST_CASE("Quaternion - Addition With Zero", "[quaternion][core][ops]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion zero = Quaternion::zero();
    
    q1 += zero;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Quaternion - Subtraction", "[quaternion][core][ops]") {
    Quaternion q1(5.0f, 7.0f, 9.0f, 11.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    q1 -= q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(7.0f, 1e-6));
}

TEST_CASE("Quaternion - Subtraction Self", "[quaternion][core][ops]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    q1 -= q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication With Identity", "[quaternion][core][ops]") {
    Quaternion q(2.0f, 3.0f, 4.0f, 5.0f);
    Quaternion identity = Quaternion::eye();
    
    q *= identity;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication i*i = -1", "[quaternion][core][ops]") {
    Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);  // i
    
    qi *= Quaternion(0.0f, 1.0f, 0.0f, 0.0f);  // i * i
    
    // i² = -1
    REQUIRE_THAT(qi.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(qi.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication j*j = -1", "[quaternion][core][ops]") {
    Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);  // j
    
    qj *= Quaternion(0.0f, 0.0f, 1.0f, 0.0f);  // j * j
    
    REQUIRE_THAT(qj.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(qj.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qj.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qj.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication k*k = -1", "[quaternion][core][ops]") {
    Quaternion qk(0.0f, 0.0f, 0.0f, 1.0f);  // k
    
    qk *= Quaternion(0.0f, 0.0f, 0.0f, 1.0f);  // k * k
    
    REQUIRE_THAT(qk.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(qk.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qk.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qk.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication i*j = k", "[quaternion][core][ops]") {
    Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);  // j
    
    qi *= qj;  // i * j
    
    // i * j = k
    REQUIRE_THAT(qi.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.z(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication j*i = -k", "[quaternion][core][ops]") {
    Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);  // j
    Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);  // i
    
    qj *= qi;  // j * i
    
    // j * i = -k (non-commutative!)
    REQUIRE_THAT(qj.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qj.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qj.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qj.z(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Scalar", "[quaternion][core][ops]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q *= 2.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Scalar Zero", "[quaternion][core][ops]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q *= 0.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Division Scalar", "[quaternion][core][ops]") {
    Quaternion q(4.0f, 6.0f, 8.0f, 10.0f);
    
    q /= 2.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Quaternion - Division By One", "[quaternion][core][ops]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q /= 1.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Quaternion - Chained Operations", "[quaternion][integration]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(0.0f, 1.0f, 0.0f, 0.0f);
    
    q1 += q2;
    q1 *= 2.0f;
    q1 -= Quaternion(1.0f, 1.0f, 0.0f, 0.0f);
    
    // (1+i)*2 - (1+i) = 2+2i - 1-i = 1+i
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Hamilton Product i*j*k = -1", "[quaternion][integration]") {
    Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);  // j
    Quaternion qk(0.0f, 0.0f, 0.0f, 1.0f);  // k
    
    qi *= qj;  // i*j = k
    qi *= qk;  // k*k = -1
    
    REQUIRE_THAT(qi.w(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(qi.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(qi.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Quaternion - Non-Commutativity", "[quaternion][integration]") {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion a = q1;
    Quaternion b = q2;
    
    a *= q2;  // q1 * q2
    b *= q1;  // q2 * q1
    
    // q1*q2 ≠ q2*q1 in general
    bool areEqual = 
        std::abs(a.w() - b.w()) < 1e-6 &&
        std::abs(a.x() - b.x()) < 1e-6 &&
        std::abs(a.y() - b.y()) < 1e-6 &&
        std::abs(a.z() - b.z()) < 1e-6;
    
    REQUIRE_FALSE(areEqual);
}

TEST_CASE("Quaternion - Axis-Angle Round-Trip", "[quaternion][integration]") {
    Vector<3> axis = {1.0f, 1.0f, 1.0f};
    float angle = M_PI / 3.0f;
    
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    
    // Verify it creates a unit quaternion
    float normSq = q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();
    REQUIRE_THAT(normSq, Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Quaternion - Double Rotation Equivalence", "[quaternion][integration]") {
    // 90° + 90° = 180°
    Vector<3> axis = {0.0f, 0.0f, 1.0f};
    
    Quaternion q1 = Quaternion::fromAxisAngle(axis, M_PI / 2.0f);
    Quaternion q2 = Quaternion::fromAxisAngle(axis, M_PI / 2.0f);
    Quaternion q180 = Quaternion::fromAxisAngle(axis, M_PI);
    
    q1 *= q2;
    
    // Results should be equivalent (up to sign)
    bool equiv = 
        (std::abs(q1.w() - q180.w()) < 1e-6 &&
         std::abs(q1.x() - q180.x()) < 1e-6 &&
         std::abs(q1.y() - q180.y()) < 1e-6 &&
         std::abs(q1.z() - q180.z()) < 1e-6) ||
        (std::abs(q1.w() + q180.w()) < 1e-6 &&
         std::abs(q1.x() + q180.x()) < 1e-6 &&
         std::abs(q1.y() + q180.y()) < 1e-6 &&
         std::abs(q1.z() + q180.z()) < 1e-6);
    
    REQUIRE(equiv);
}