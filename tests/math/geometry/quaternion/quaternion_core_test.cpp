#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion Core Tests (quaternion.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Construction Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Default Construction Zeros All Components", "[quaternion][core][construction]") {
    Quaternion<> q;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Single Parameter Construction", "[quaternion][core][construction]") {
    Quaternion<> q(2.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Two Parameter Construction", "[quaternion][core][construction]") {
    Quaternion<> q(1.0f, 2.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Three Parameter Construction", "[quaternion][core][construction]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Full Construction", "[quaternion][core][construction]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Construction With Negative Values", "[quaternion][core][construction]") {
    Quaternion<> q(-1.0f, -2.0f, -3.0f, -4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Quaternion - Construction From Complex Number", "[quaternion][core][construction]") {
    cobalt::math::algebra::Complex c(3.0f, 4.0f);
    Quaternion<> q(c);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Factory Method Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Zero Factory", "[quaternion][core][factory]") {
    Quaternion<> q = Quaternion<>::zero();
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Identity Factory", "[quaternion][core][factory]") {
    Quaternion<> q = Quaternion<>::eye();
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Pure Vector", "[quaternion][core][factory]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Quaternion<> q = Quaternion<>::fromPureVector(v);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Quaternion - From Axis Angle X-Axis 90 Degrees", "[quaternion][core][factory]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 2.0f;
    
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Axis Angle Y-Axis 180 Degrees", "[quaternion][core][factory]") {
    Vector<3> axis = {0.0f, 1.0f, 0.0f};
    float angle = M_PI;
    
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Axis Angle Z-Axis 270 Degrees", "[quaternion][core][factory]") {
    Vector<3> axis = {0.0f, 0.0f, 1.0f};
    float angle = 3.0f * M_PI / 2.0f;
    
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-6));
}

TEST_CASE("Quaternion - From Axis Angle Zero Angle", "[quaternion][core][factory]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = 0.0f;
    
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Axis Angle Unnormalized Axis", "[quaternion][core][factory]") {
    Vector<3> axis = {2.0f, 0.0f, 0.0f};  // Not normalized
    float angle = M_PI / 2.0f;
    
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    // Should normalize the axis internally
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-6));
}

TEST_CASE("Quaternion - From Rotation Vector Zero", "[quaternion][core][factory]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    Quaternion<> q = Quaternion<>::fromRotationVector(v);
    
    // Should return identity
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Rotation Vector X-Axis", "[quaternion][core][factory]") {
    Vector<3> v = {M_PI / 2.0f, 0.0f, 0.0f};
    
    Quaternion<> q = Quaternion<>::fromRotationVector(v);
    
    float angle = M_PI / 2.0f;
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Rotation Vector Arbitrary Direction", "[quaternion][core][factory]") {
    Vector<3> v = {1.0f, 1.0f, 1.0f};
    
    Quaternion<> q = Quaternion<>::fromRotationVector(v);
    
    // Should create valid quaternion
    float normV = norm(v);
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(std::cos(normV/2), 1e-6));
}

TEST_CASE("Quaternion - From Rotation Matrix Identity", "[quaternion][core][factory]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    
    Quaternion<> q = Quaternion<>::fromRotationMatrix(R);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Quaternion - From Rotation Matrix X-Axis 90", "[quaternion][core][factory]") {
    Matrix<3, 3> R;
    R(0, 0) = 1.0f; R(0, 1) = 0.0f;  R(0, 2) = 0.0f;
    R(1, 0) = 0.0f; R(1, 1) = 0.0f;  R(1, 2) = -1.0f;
    R(2, 0) = 0.0f; R(2, 1) = 1.0f;  R(2, 2) = 0.0f;
    
    Quaternion<> q = Quaternion<>::fromRotationMatrix(R);
    
    // Should represent 90 degree rotation about X
    float angle = M_PI / 2.0f;
    REQUIRE_THAT(std::abs(q.w()), Catch::Matchers::WithinAbs(std::cos(angle/2), 1e-5));
    REQUIRE_THAT(std::abs(q.x()), Catch::Matchers::WithinAbs(std::sin(angle/2), 1e-5));
}

TEST_CASE("Quaternion - From Euler Angles Zero", "[quaternion][core][factory]") {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - From Euler Angles Roll Only", "[quaternion][core][factory]") {
    float roll = M_PI / 2.0f, pitch = 0.0f, yaw = 0.0f;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    
    // Should be primarily X rotation
    REQUIRE(std::abs(q.x()) > std::abs(q.y()));
    REQUIRE(std::abs(q.x()) > std::abs(q.z()));
}

TEST_CASE("Quaternion - From Euler Angles Pitch Only", "[quaternion][core][factory]") {
    float roll = 0.0f, pitch = M_PI / 2.0f, yaw = 0.0f;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    
    // Should be primarily Y rotation
    REQUIRE(std::abs(q.y()) > std::abs(q.x()));
    REQUIRE(std::abs(q.y()) > std::abs(q.z()));
}

TEST_CASE("Quaternion - From Euler Angles Yaw Only", "[quaternion][core][factory]") {
    float roll = 0.0f, pitch = 0.0f, yaw = M_PI / 2.0f;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    
    // Should be primarily Z rotation
    REQUIRE(std::abs(q.z()) > std::abs(q.x()));
    REQUIRE(std::abs(q.z()) > std::abs(q.y()));
}

TEST_CASE("Quaternion - From Euler Angles Combined", "[quaternion][core][factory]") {
    float roll = M_PI / 4.0f, pitch = M_PI / 6.0f, yaw = M_PI / 3.0f;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    
    // Should create valid quaternion
    float normSqr = q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();
    REQUIRE_THAT(normSqr, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

// ----------------------------------------------------------------------------
// Accessor Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - W Accessor", "[quaternion][core][accessor]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - X Accessor", "[quaternion][core][accessor]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Quaternion - Y Accessor", "[quaternion][core][accessor]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Quaternion - Z Accessor", "[quaternion][core][accessor]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - W Accessor Modification", "[quaternion][core][accessor]") {
    Quaternion<> q;
    
    q.w() = 5.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Quaternion - X Accessor Modification", "[quaternion][core][accessor]") {
    Quaternion<> q;
    
    q.x() = 6.0f;
    
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Quaternion - Y Accessor Modification", "[quaternion][core][accessor]") {
    Quaternion<> q;
    
    q.y() = 7.0f;
    
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Quaternion - Z Accessor Modification", "[quaternion][core][accessor]") {
    Quaternion<> q;
    
    q.z() = 8.0f;
    
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Quaternion - Const W Accessor", "[quaternion][core][accessor]") {
    const Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - Const X Accessor", "[quaternion][core][accessor]") {
    const Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Quaternion - Const Y Accessor", "[quaternion][core][accessor]") {
    const Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Quaternion - Const Z Accessor", "[quaternion][core][accessor]") {
    const Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Vector Accessor", "[quaternion][core][accessor]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Vector<3> v = q.vector();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Vector Accessor Pure Imaginary", "[quaternion][core][accessor]") {
    Quaternion<> q(0.0f, 5.0f, 6.0f, 7.0f);
    
    Vector<3> v = q.vector();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(7.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Arithmetic Assignment Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Addition Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    q1 += q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Quaternion - Addition Assignment With Zero", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2 = Quaternion<>::zero();
    
    Quaternion<> original(q1);
    q1 += q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(original.w(), 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(original.x(), 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(original.y(), 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(original.z(), 1e-6));
}

TEST_CASE("Quaternion - Subtraction Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(5.0f, 6.0f, 7.0f, 8.0f);
    Quaternion<> q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    q1 -= q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Subtraction Assignment Self Zeros", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(q1);
    
    q1 -= q2;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 0.0f, 0.0f, 0.0f);  // Real
    Quaternion<> q2(0.0f, 1.0f, 0.0f, 0.0f);  // i
    
    q1 *= q2;
    
    // 1 * i = i
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Assignment i*i", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion<> q2(0.0f, 1.0f, 0.0f, 0.0f);  // i
    
    q1 *= q2;
    
    // i * i = -1
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Assignment i*j", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion<> q2(0.0f, 0.0f, 1.0f, 0.0f);  // j
    
    q1 *= q2;
    
    // i * j = k
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Assignment j*k", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(0.0f, 0.0f, 1.0f, 0.0f);  // j
    Quaternion<> q2(0.0f, 0.0f, 0.0f, 1.0f);  // k
    
    q1 *= q2;
    
    // j * k = i
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Assignment Identity", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> identity = Quaternion<>::eye();
    
    Quaternion<> original(q1);
    q1 *= identity;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(original.w(), 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(original.x(), 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(original.y(), 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(original.z(), 1e-6));
}

TEST_CASE("Quaternion - Division Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(2.0f, 0.0f, 0.0f, 0.0f);
    Quaternion<> q2(2.0f, 0.0f, 0.0f, 0.0f);
    
    q1 /= q2;
    
    // Should equal 1
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Division Assignment By Identity", "[quaternion][core][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> identity = Quaternion<>::eye();
    
    Quaternion<> original(q1);
    q1 /= identity;
    
    REQUIRE_THAT(q1.w(), Catch::Matchers::WithinAbs(original.w(), 1e-6));
    REQUIRE_THAT(q1.x(), Catch::Matchers::WithinAbs(original.x(), 1e-6));
    REQUIRE_THAT(q1.y(), Catch::Matchers::WithinAbs(original.y(), 1e-6));
    REQUIRE_THAT(q1.z(), Catch::Matchers::WithinAbs(original.z(), 1e-6));
}

TEST_CASE("Quaternion - Scalar Multiplication Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q *= 2.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Quaternion - Scalar Multiplication Assignment By Zero", "[quaternion][core][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q *= 0.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Scalar Multiplication Assignment By One", "[quaternion][core][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> original(q);
    
    q *= 1.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(original.w(), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(original.x(), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(original.y(), 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(original.z(), 1e-6));
}

TEST_CASE("Quaternion - Scalar Division Assignment", "[quaternion][core][arithmetic]") {
    Quaternion<> q(2.0f, 4.0f, 6.0f, 8.0f);
    
    q /= 2.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Scalar Division Assignment By One", "[quaternion][core][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> original(q);
    
    q /= 1.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(original.w(), 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(original.x(), 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(original.y(), 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(original.z(), 1e-6));
}

TEST_CASE("Quaternion - Scalar Multiplication Assignment Negative", "[quaternion][core][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    q *= -1.0f;
    
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Double Type Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Double Type Construction", "[quaternion][core][type]") {
    Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
    
    REQUIRE(q.w() == 1.0);
    REQUIRE(q.x() == 2.0);
    REQUIRE(q.y() == 3.0);
    REQUIRE(q.z() == 4.0);
}

TEST_CASE("Quaternion - Double Type Operations", "[quaternion][core][type]") {
    Quaternion<double> q1(1.0, 2.0, 3.0, 4.0);
    Quaternion<double> q2(5.0, 6.0, 7.0, 8.0);
    
    q1 += q2;
    
    REQUIRE(q1.w() == 6.0);
    REQUIRE(q1.x() == 8.0);
    REQUIRE(q1.y() == 10.0);
    REQUIRE(q1.z() == 12.0);
}