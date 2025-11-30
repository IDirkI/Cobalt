#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_CASE("Transform - Default Constructor Identity", "[transform][core]") {
    Transform<> T;
    
    Matrix<3, 3> R = T.rotation();
    Vector<3> t = T.translation();
    
    // Should be identity rotation
    REQUIRE_THAT(R(0,0), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(R(1,1), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(R(2,2), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    
    // Should be zero translation
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Constructor from R and t", "[transform][core]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(R, t);
    
    REQUIRE(T.rotation() == R);
    REQUIRE(T.translation() == t);
}

// ============================================================================
// Static Factory Tests
// ============================================================================

TEST_CASE("Transform - Identity Factory", "[transform][core][factory]") {
    Transform<> T = Transform<>::eye();
    
    Matrix<3, 3> R = T.rotation();
    Vector<3> t = T.translation();
    
    REQUIRE(R == Matrix<3, 3>::eye());
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - From Quaternion Identity", "[transform][core][factory]") {
    Quaternion q = Quaternion::eye();
    Transform<> T = Transform<>::quatenrion(q);
    
    Matrix<3, 3> R = T.rotation();
    
    REQUIRE_THAT(R(0,0), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(R(1,1), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(R(2,2), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("Transform - From Quaternion 90-deg Z-Axis", "[transform][core][factory]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Transform<> T = Transform<>::quatenrion(q);
    
    // 90° rotation around Z should map X to Y
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Transform - From Pose", "[transform][core][factory]") {
    Vector<3> position = {1.0f, 2.0f, 3.0f};
    Quaternion orientation = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    
    Transform<> T = Transform<>::pose(position, orientation);
    
    REQUIRE(T.translation() == position);
}

TEST_CASE("Transform - From Rotation Vector", "[transform][core][factory]") {
    Vector<3> rotVec = {0.0f, 0.0f, M_PI / 2.0f};  // 90° around Z
    Transform<> T = Transform<>::rotationVector(rotVec);
    
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("Transform - From Translation", "[transform][core][factory]") {
    Vector<3> t = {5.0f, 10.0f, 15.0f};
    Transform<> T = Transform<>::translationVector(t);
    
    REQUIRE(T.rotation() == Matrix<3, 3>::eye());
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Rotation X", "[transform][core][factory]") {
    Transform<> T = Transform<>::rotationX(M_PI / 2.0f);
    
    // 90° rotation around X should map Y to Z
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Y", "[transform][core][factory]") {
    Transform<> T = Transform<>::rotationY(M_PI / 2.0f);
    
    // 90° rotation around Y should map Z to X
    Vector<3> z_axis = {0.0f, 0.0f, 1.0f};
    Vector<3> result = T.apply(z_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Z", "[transform][core][factory]") {
    Transform<> T = Transform<>::rotationZ(M_PI / 2.0f);
    
    // 90° rotation around Z should map X to Y
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

// ============================================================================
// Accessor Tests
// ============================================================================

TEST_CASE("Transform - Get Rotation", "[transform][core][accessor]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    R(0,1) = 0.5f;
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(R, t);
    
    Matrix<3, 3> R_out = T.rotation();
    REQUIRE_THAT(R_out(0,1), Catch::Matchers::WithinAbs(0.5f, 1e-6));
}

TEST_CASE("Transform - Get Translation", "[transform][core][accessor]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(R, t);
    
    Vector<3> t_out = T.translation();
    REQUIRE(t_out == t);
}

TEST_CASE("Transform - Set Rotation", "[transform][core][accessor]") {
    Transform<> T;
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    R(1,2) = 0.7f;
    
    T.rotation() = R;
    
    REQUIRE_THAT(T.rotation()(1,2), Catch::Matchers::WithinAbs(0.7f, 1e-6));
}

TEST_CASE("Transform - Set Translation", "[transform][core][accessor]") {
    Transform<> T;
    Vector<3> t = {4.0f, 5.0f, 6.0f};
    
    T.translation() = t;
    
    REQUIRE(T.translation() == t);
}

// ============================================================================
// Transform Point/Vector Tests
// ============================================================================

TEST_CASE("Transform - Transform Point Identity", "[transform][core][transform]") {
    Transform<> T = Transform<>::eye();
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE(result == p);
}

TEST_CASE("Transform - Transform Point Translation Only", "[transform][core][transform]") {
    Vector<3> t = {5.0f, 10.0f, 15.0f};
    Transform<> T = Transform<>::translationVector(t);
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(12.0f, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(18.0f, 1e-6));
}

TEST_CASE("Transform - Transform Point Rotation Only", "[transform][core][transform]") {
    Transform<> T = Transform<>::rotationZ(M_PI / 2.0f);
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Transform Vector No Translation", "[transform][core][transform]") {
    Vector<3> t = {0.0f, 0.0f, 0.0f};
    Transform<> T = Transform<>::translationVector(t);
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = T.apply(v);
    
    // Vector should not be translated
    REQUIRE(result == v);
}

TEST_CASE("Transform - Transform Vector Rotation", "[transform][core][transform]") {
    Transform<> T = Transform<>::rotationZ(M_PI / 2.0f);
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = T.apply(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

// ============================================================================
// Composition Tests
// ============================================================================

TEST_CASE("Transform - Composition With Identity", "[transform][core][compose]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T = Transform<>::translationVector(t);
    Transform<> I = Transform<>::eye();
    
    T *= I;
    
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Composition Two Translations", "[transform][core][compose]") {
    Transform<> T1 = Transform<>::translationVector(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T2 = Transform<>::translationVector(Vector<3>{0.0f, 2.0f, 0.0f});
    
    T1 *= T2;
    
    Vector<3> t = T1.translation();
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(2.0f, 1e-6));
}

TEST_CASE("Transform - Composition Two Rotations", "[transform][core][compose]") {
    Transform<> T1 = Transform<>::rotationZ(M_PI / 4.0f);
    Transform<> T2 = Transform<>::rotationZ(M_PI / 4.0f);
    
    T1 *= T2;
    
    // Should be 90° total
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T1.apply(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Composition Order Matters", "[transform][core][compose]") {
    Transform<> T_rot = Transform<>::rotationZ(M_PI / 2.0f);
    Transform<> T_trans = Transform<>::translationVector(Vector<3>{1.0f, 0.0f, 0.0f});
    
    // Rotate then translate
    Transform<> T1 = T_trans;
    T1 *= T_rot;
    
    // Translate then rotate
    Transform<> T2 = T_rot;
    T2 *= T_trans;
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> result1 = T1.apply(p);
    Vector<3> result2 = T2.apply(p);
    
    // Results should be different
    bool different = std::abs(result1[0] - result2[0]) > 1e-4f ||
                     std::abs(result1[1] - result2[1]) > 1e-4f;
    
    REQUIRE(different);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("Transform - Robot Arm Forward Kinematics", "[transform][integration]") {
    // Simple 2-joint planar arm
    Transform<> joint1 = Transform<>::rotationZ(M_PI / 4.0f);  // 45°
    Transform<> link1 = Transform<>::translationVector(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> joint2 = Transform<>::rotationZ(M_PI / 4.0f);  // 45°
    Transform<> link2 = Transform<>::translationVector(Vector<3>{1.0f, 0.0f, 0.0f});
    
    // Chain: base -> joint1 -> link1 -> joint2 -> link2 -> end-effector
    Transform<> T = joint1;
    T *= link1;
    T *= joint2;
    T *= link2;
    
    Vector<3> end_effector = T.apply(Vector<3>::zero());
    
    // End effector should be at specific position
    REQUIRE_THAT(norm(end_effector), Catch::Matchers::WithinAbs(std::sqrt(2.0f - 2.0f*std::cos(M_PI_4*3.0f)), 1e-4f));
}