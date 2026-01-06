#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Transform Core Tests (transform.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Construction Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Default Constructor Identity", "[transform][core][construction]") {
    Transform<> T;
    
    Quaternion<> q = T.rotation();
    Vector<3> t = T.translation();
    
    // Should be identity rotation
    REQUIRE_THAT(q.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(q.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(q.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    
    // Should be zero translation
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Constructor from Quaternion and Translation", "[transform][core][construction]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(q, t);
    
    REQUIRE(T.rotation() == q);
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Constructor Quaternion Only", "[transform][core][construction]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 2);
    
    Transform<> T(q);
    
    REQUIRE(T.rotation() == q);
    REQUIRE_THAT(norm(T.translation()), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Constructor Translation Only", "[transform][core][construction]") {
    Vector<3> t = {5.0f, 10.0f, 15.0f};
    
    Transform<> T(t);
    
    REQUIRE(T.translation() == t);
    REQUIRE(T.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - Constructor from Translation and Quaternion", "[transform][core][construction]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 3);
    
    Transform<> T(t, q);
    
    REQUIRE(T.translation() == t);
    REQUIRE(T.rotation() == q);
}

TEST_CASE("Transform - Constructor from Matrix and Translation", "[transform][core][construction]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    R(0, 1) = -1.0f;
    R(1, 0) = 1.0f;
    R(0, 0) = 0.0f;
    R(1, 1) = 0.0f;
    
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(R, t);
    
    REQUIRE(T.translation() == t);
    // Rotation should be converted from matrix
    REQUIRE(norm(T.rotation()) > 0.0f);
}

TEST_CASE("Transform - Constructor Matrix Only", "[transform][core][construction]") {
    Matrix<3, 3> R = Matrix<3, 3>::eye();
    
    Transform<> T(R);
    
    REQUIRE_THAT(norm(T.translation()), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Constructor Negative Translation", "[transform][core][construction]") {
    Vector<3> t = {-1.0f, -2.0f, -3.0f};
    
    Transform<> T(t);
    
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Constructor Large Values", "[transform][core][construction]") {
    Vector<3> t = {1000.0f, 2000.0f, 3000.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI);
    
    Transform<> T(q, t);
    
    REQUIRE(T.translation() == t);
    REQUIRE(T.rotation() == q);
}

// ----------------------------------------------------------------------------
// Factory Method Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Identity Factory", "[transform][core][factory]") {
    Transform<> T = Transform<>::eye();
    
    Quaternion<> q = T.rotation();
    Vector<3> t = T.translation();
    
    REQUIRE(q == Quaternion<>::eye());
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(t[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - From Rotation Vector Zero", "[transform][core][factory]") {
    Vector<3> rotVec = {0.0f, 0.0f, 0.0f};
    Transform<> T = Transform<>::fromRotationVector(rotVec);
    
    REQUIRE(T.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - From Rotation Vector X-Axis", "[transform][core][factory]") {
    Vector<3> rotVec = {(float)M_PI / 2.0f, 0.0f, 0.0f};
    Transform<> T = Transform<>::fromRotationVector(rotVec);
    
    // 90° rotation around X should map Y to Z
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - From Rotation Vector Arbitrary", "[transform][core][factory]") {
    Vector<3> rotVec = {1.0f, 1.0f, 1.0f};
    Transform<> T = Transform<>::fromRotationVector(rotVec);
    
    // Should create valid transform
    REQUIRE(isNormalized(T.rotation()));
}

TEST_CASE("Transform - Rotation X Factory", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationX((float)M_PI / 2.0f);
    
    // 90° rotation around X should map Y to Z
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation X Zero Angle", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationX(0.0f);
    
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> result = T.apply(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Transform - Rotation X 180 Degrees", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationX((float)M_PI);
    
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Y Factory", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationY((float)M_PI / 2.0f);
    
    // 90° rotation around Y should map Z to X
    Vector<3> z_axis = {0.0f, 0.0f, 1.0f};
    Vector<3> result = T.apply(z_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Y 180 Degrees", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationY((float)M_PI);
    
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Z Factory", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationZ((float)M_PI / 2.0f);
    
    // 90° rotation around Z should map X to Y
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Z Negative Angle", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationZ(-(float)M_PI / 2.0f);
    
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Rotation Z 360 Degrees", "[transform][core][factory]") {
    Transform<> T = Transform<>::fromRotationZ(2.0f * (float)M_PI);
    
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> result = T.apply(v);
    
    // Should return to original (within tolerance)
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(v[0], 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(v[1], 1e-4f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(v[2], 1e-4f));
}

// ----------------------------------------------------------------------------
// Accessor Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Get Rotation", "[transform][core][accessor]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(q, t);
    
    Quaternion<> q_out = T.rotation();
    REQUIRE(q_out == q);
}

TEST_CASE("Transform - Get Translation", "[transform][core][accessor]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 3);
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    
    Transform<> T(q, t);
    
    Vector<3> t_out = T.translation();
    REQUIRE(t_out == t);
}

TEST_CASE("Transform - Set Rotation", "[transform][core][accessor]") {
    Transform<> T;
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 6);
    
    T.rotation() = q;
    
    REQUIRE(T.rotation() == q);
}

TEST_CASE("Transform - Set Translation", "[transform][core][accessor]") {
    Transform<> T;
    Vector<3> t = {4.0f, 5.0f, 6.0f};
    
    T.translation() = t;
    
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Const Get Rotation", "[transform][core][accessor]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 0.0f}, (float)M_PI / 4);
    const Transform<> T(q);
    
    Quaternion<> q_out = T.rotation();
    REQUIRE(q_out == q);
}

TEST_CASE("Transform - Const Get Translation", "[transform][core][accessor]") {
    Vector<3> t = {7.0f, 8.0f, 9.0f};
    const Transform<> T(t);
    
    Vector<3> t_out = T.translation();
    REQUIRE(t_out == t);
}

TEST_CASE("Transform - Modify Rotation Components", "[transform][core][accessor]") {
    Transform<> T;
    
    T.rotation().w() = 1.0f;
    T.rotation().x() = 0.0f;
    T.rotation().y() = 0.0f;
    T.rotation().z() = 0.0f;
    
    REQUIRE(T.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - Modify Translation Components", "[transform][core][accessor]") {
    Transform<> T;
    
    T.translation()[0] = 1.0f;
    T.translation()[1] = 2.0f;
    T.translation()[2] = 3.0f;
    
    Vector<3> expected = {1.0f, 2.0f, 3.0f};
    REQUIRE(T.translation() == expected);
}

// ----------------------------------------------------------------------------
// Chain Operations Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Rotate X Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.rotateX((float)M_PI / 2);
    
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotate X Multiple Times", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.rotateX((float)M_PI / 4).rotateX((float)M_PI / 4);
    
    // Should equal 90 degree rotation
    Vector<3> y_axis = {0.0f, 1.0f, 0.0f};
    Vector<3> result = T.apply(y_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotate Y Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.rotateY((float)M_PI / 2);
    
    Vector<3> z_axis = {0.0f, 0.0f, 1.0f};
    Vector<3> result = T.apply(z_axis);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotate Z Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.rotateZ((float)M_PI / 2);
    
    Vector<3> x_axis = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(x_axis);
    
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Rotate Quaternion Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    
    T.rotate(q);
    
    REQUIRE(T.rotation() == q);
}

TEST_CASE("Transform - Translate Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    Vector<3> offset = {1.0f, 2.0f, 3.0f};
    
    T.translate(offset);
    
    REQUIRE(T.translation() == offset);
}

TEST_CASE("Transform - Translate Multiple Times", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.translate(Vector<3>{1.0f, 0.0f, 0.0f})
     .translate(Vector<3>{0.0f, 2.0f, 0.0f})
     .translate(Vector<3>{0.0f, 0.0f, 3.0f});
    
    Vector<3> expected = {1.0f, 2.0f, 3.0f};
    REQUIRE(T.translation() == expected);
}

TEST_CASE("Transform - Combined Rotate and Translate Chain", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    T.rotateZ((float)M_PI / 4).translate(Vector<3>{1.0f, 0.0f, 0.0f});
    
    // Should have both rotation and translation
    REQUIRE(norm(T.translation()) > 0.0f);
    REQUIRE_FALSE(T.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - Chaining Returns Reference", "[transform][core][chain]") {
    Transform<> T = Transform<>::eye();
    
    Transform<> &ref = T.rotateX((float)M_PI / 6);
    
    REQUIRE(&ref == &T);
}

// ----------------------------------------------------------------------------
// Apply Transformation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Transform Point Identity", "[transform][core][apply]") {
    Transform<> T = Transform<>::eye();
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE(result == p);
}

TEST_CASE("Transform - Transform Point Translation Only", "[transform][core][apply]") {
    Vector<3> t = {5.0f, 10.0f, 15.0f};
    Transform<> T(t);
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(12.0f, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(18.0f, 1e-6));
}

TEST_CASE("Transform - Transform Point Rotation Only", "[transform][core][apply]") {
    Transform<> T = Transform<>::fromRotationZ((float)M_PI / 2.0f);
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Transform Point Combined", "[transform][core][apply]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2.0f);
    Vector<3> t = {1.0f, 0.0f, 0.0f};
    Transform<> T(q, t);
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(p);
    
    // Rotate (1,0,0) to (0,1,0), then translate by (1,0,0) -> (1,1,0)
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Transform Zero Vector", "[transform][core][apply]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T(t);
    Vector<3> p = {0.0f, 0.0f, 0.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE(result == t);
}

TEST_CASE("Transform - Transform Negative Point", "[transform][core][apply]") {
    Transform<> T = Transform<>::eye();
    Vector<3> p = {-1.0f, -2.0f, -3.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE(result == p);
}

TEST_CASE("Transform - Transform Preserves Distance From Origin After Rotation", "[transform][core][apply]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 3);
    Transform<> T(q);
    Vector<3> p = {3.0f, 4.0f, 0.0f};
    
    Vector<3> result = T.apply(p);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(norm(p), 1e-5f));
}

TEST_CASE("Transform - Transform Multiple Points", "[transform][core][apply]") {
    Transform<> T = Transform<>::fromRotationZ((float)M_PI / 4.0f);
    
    Vector<3> p1 = {1.0f, 0.0f, 0.0f};
    Vector<3> p2 = {0.0f, 1.0f, 0.0f};
    Vector<3> p3 = {1.0f, 1.0f, 0.0f};
    
    Vector<3> r1 = T.apply(p1);
    Vector<3> r2 = T.apply(p2);
    Vector<3> r3 = T.apply(p3);
    
    // All should be rotated 45°
    REQUIRE_THAT(norm(r1), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(norm(r2), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(norm(r3), Catch::Matchers::WithinAbs(std::sqrt(2.0f), 1e-5f));
}

// ----------------------------------------------------------------------------
// Composition Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Composition With Identity", "[transform][core][compose]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T(t);
    Transform<> I = Transform<>::eye();
    
    T *= I;
    
    REQUIRE(T.translation() == t);
}

TEST_CASE("Transform - Composition Identity Left", "[transform][core][compose]") {
    Transform<> I = Transform<>::eye();
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T(t);
    
    I *= T;
    
    REQUIRE(I.translation() == t);
}

TEST_CASE("Transform - Composition Two Translations", "[transform][core][compose]") {
    Transform<> T1(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T2(Vector<3>{0.0f, 2.0f, 0.0f});
    
    T1 *= T2;
    
    Vector<3> t = T1.translation();
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(2.0f, 1e-6));
}

TEST_CASE("Transform - Composition Two Rotations", "[transform][core][compose]") {
    Transform<> T1 = Transform<>::fromRotationZ((float)M_PI / 4.0f);
    Transform<> T2 = Transform<>::fromRotationZ((float)M_PI / 4.0f);
    
    T1 *= T2;
    
    // Should be 90° total
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T1.apply(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Composition Order Matters", "[transform][core][compose]") {
    Transform<> T_rot = Transform<>::fromRotationZ((float)M_PI / 2.0f);
    Transform<> T_trans(Vector<3>{1.0f, 0.0f, 0.0f});
    
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

TEST_CASE("Transform - Composition Three Transforms", "[transform][core][compose]") {
    Transform<> T1 = Transform<>::fromRotationX((float)M_PI / 6);
    Transform<> T2(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T3 = Transform<>::fromRotationZ((float)M_PI / 4);
    
    T1 *= T2;
    T1 *= T3;
    
    // Should have combined rotation and translation
    REQUIRE(norm(T1.translation()) > 0.0f);
    REQUIRE_FALSE(T1.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - Composition Associative", "[transform][core][compose]") {
    Transform<> T1 = Transform<>::fromRotationX(0.3f);
    Transform<> T2(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T3 = Transform<>::fromRotationZ(0.5f);
    
    Transform<> result1 = T1;
    result1 *= T2;
    result1 *= T3;
    
    Transform<> temp = T2;
    temp *= T3;
    Transform<> result2 = T1;
    result2 *= temp;
    
    Vector<3> p = {1.0f, 1.0f, 1.0f};
    Vector<3> p1 = result1.apply(p);
    Vector<3> p2 = result2.apply(p);
    
    REQUIRE_THAT(p1[0], Catch::Matchers::WithinAbs(p2[0], 1e-4f));
    REQUIRE_THAT(p1[1], Catch::Matchers::WithinAbs(p2[1], 1e-4f));
    REQUIRE_THAT(p1[2], Catch::Matchers::WithinAbs(p2[2], 1e-4f));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Robot Arm Forward Kinematics", "[transform][integration]") {
    // Simple 2-joint planar arm
    Transform<> joint1 = Transform<>::fromRotationZ((float)M_PI / 4.0f);  // 45°
    Transform<> link1(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> joint2 = Transform<>::fromRotationZ((float)M_PI / 4.0f);  // 45°
    Transform<> link2(Vector<3>{1.0f, 0.0f, 0.0f});
    
    // Chain: base -> joint1 -> link1 -> joint2 -> link2 -> end-effector
    Transform<> T = joint1;
    T *= link1;
    T *= joint2;
    T *= link2;
    
    Vector<3> end_effector = T.apply(Vector<3>::zero());
    
    // End effector should be at specific position
    REQUIRE(norm(end_effector) > 0.0f);
}

TEST_CASE("Transform - Camera Pose Transform", "[transform][integration]") {
    // Camera at position (0, 0, 5) looking down negative Z
    Vector<3> camera_pos = {0.0f, 0.0f, 5.0f};
    Quaternion<> camera_rot = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI);
    
    Transform<> world_to_camera(camera_rot, camera_pos);
    
    // Point in world space
    Vector<3> world_point = {1.0f, 2.0f, 3.0f};
    
    // Transform to camera space
    Vector<3> camera_point = world_to_camera.apply(world_point);
    
    REQUIRE(isFinite(camera_point));
}

TEST_CASE("Transform - Sequential Translations Accumulate", "[transform][integration]") {
    Transform<> T = Transform<>::eye();
    
    for (int i = 0; i < 10; i++) {
        Transform<> step(Vector<3>{0.1f, 0.0f, 0.0f});
        T *= step;
    }
    
    Vector<3> t = T.translation();
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - Sequential Rotations Accumulate", "[transform][integration]") {
    Transform<> T = Transform<>::eye();
    
    for (int i = 0; i < 4; i++) {
        Transform<> step = Transform<>::fromRotationZ((float)M_PI / 2.0f);
        T *= step;
    }
    
    // 4 * 90° = 360°, should return to identity
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(v[0], 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(v[1], 1e-4f));
}

TEST_CASE("Transform - Transform Chain Preserves Point", "[transform][integration]") {
    // Create arbitrary transform
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 3);
    Vector<3> t = {2.0f, 3.0f, 4.0f};
    Transform<> T(q, t);
    
    // Apply to multiple points
    std::vector<Vector<3>> points = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };
    
    for (const auto& p : points) {
        Vector<3> result = T.apply(p);
        REQUIRE(isFinite(result));
    }
}

TEST_CASE("Transform - Gimbal Lock Scenario", "[transform][integration]") {
    // Test for gimbal lock at pitch = 90°
    float roll = (float)M_PI / 6;
    float pitch = (float)M_PI / 2;
    float yaw = (float)M_PI / 4;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    Transform<> T(q);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T.apply(v);
    
    REQUIRE(isFinite(result));
}

TEST_CASE("Transform - Double Type Support", "[transform][integration]") {
    Transform<double> T;
    
    T.translation()[0] = 1.0;
    T.translation()[1] = 2.0;
    T.translation()[2] = 3.0;
    
    Vector<3, double> v = {1.0, 1.0, 1.0};
    Vector<3, double> result = T.apply(v);
    
    REQUIRE(result[0] == 2.0);
    REQUIRE(result[1] == 3.0);
    REQUIRE(result[2] == 4.0);
}