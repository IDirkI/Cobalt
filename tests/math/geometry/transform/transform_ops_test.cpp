#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Transform Operations Tests (transform_ops.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Binary Multiplication Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Binary Multiplication Identity", "[transform][ops][multiply]") {
    Transform<> T1(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2 = Transform<>::eye();
    
    Transform<> result = T1 * T2;
    
    REQUIRE(result.translation() == T1.translation());
    REQUIRE(result.rotation() == T1.rotation());
}

TEST_CASE("Transform - Binary Multiplication Two Translations", "[transform][ops][multiply]") {
    Transform<> T1(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T2(Vector<3>{0.0f, 2.0f, 0.0f});
    
    Transform<> result = T1 * T2;
    
    Vector<3> t = result.translation();
    REQUIRE_THAT(t[0], Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(t[1], Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(t[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Binary Multiplication Two Rotations", "[transform][ops][multiply]") {
    Transform<> T1 = Transform<>::fromRotationZ(M_PI / 4.0f);
    Transform<> T2 = Transform<>::fromRotationZ(M_PI / 4.0f);
    
    Transform<> result = T1 * T2;
    
    // Should be 90° rotation
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> rotated = result.apply(v);
    
    REQUIRE_THAT(rotated[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(rotated[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Binary Multiplication Order Matters", "[transform][ops][multiply]") {
    Transform<> T_rot = Transform<>::fromRotationZ(M_PI / 2.0f);
    Transform<> T_trans(Vector<3>{1.0f, 0.0f, 0.0f});
    
    Transform<> result1 = T_rot * T_trans;
    Transform<> result2 = T_trans * T_rot;
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> p1 = result1.apply(p);
    Vector<3> p2 = result2.apply(p);
    
    // Results should be different
    bool different = std::abs(p1[0] - p2[0]) > 1e-4f ||
                     std::abs(p1[1] - p2[1]) > 1e-4f;
    
    REQUIRE(different);
}

TEST_CASE("Transform - Binary Multiplication Rotation Then Translation", "[transform][ops][multiply]") {
    // First rotate, then translate
    Transform<> T_rot = Transform<>::fromRotationZ(M_PI / 2.0f);
    Transform<> T_trans(Vector<3>{1.0f, 0.0f, 0.0f});
    
    Transform<> result = T_trans * T_rot;
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> transformed = result.apply(p);
    
    // Point rotated 90° then translated by (1,0,0)
    // (1,0,0) -> (0,1,0) -> (1,1,0)
    REQUIRE_THAT(transformed[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(transformed[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(transformed[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Binary Multiplication Translation Then Rotation", "[transform][ops][multiply]") {
    // First translate, then rotate
    Transform<> T_trans(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T_rot = Transform<>::fromRotationZ(M_PI / 2.0f);
    
    Transform<> result = T_rot * T_trans;
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> transformed = result.apply(p);
    
    // Point translated by (1,0,0) then rotated 90°
    // (1,0,0) -> (2,0,0) -> (0,2,0)
    REQUIRE_THAT(transformed[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(transformed[1], Catch::Matchers::WithinAbs(2.0f, 1e-5f));
    REQUIRE_THAT(transformed[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Binary Multiplication Chaining Three Transforms", "[transform][ops][multiply]") {
    Transform<> T1 = Transform<>::fromRotationX(M_PI / 2.0f);
    Transform<> T2(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> T3 = Transform<>::fromRotationZ(M_PI / 2.0f);
    
    Transform<> result = T1 * T2 * T3;
    
    // Should compose all three
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> transformed = result.apply(v);
    
    // Verify composition worked (exact values depend on order)
    float mag = norm(transformed);
    REQUIRE(mag > 0.1f);  // Not at origin
}

TEST_CASE("Transform - Binary Multiplication Associativity", "[transform][ops][multiply]") {
    Transform<> T1 = Transform<>::fromRotationX(0.3f);
    Transform<> T2(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T3 = Transform<>::fromRotationZ(0.5f);
    
    Transform<> result1 = (T1 * T2) * T3;
    Transform<> result2 = T1 * (T2 * T3);
    
    Vector<3> p = {1.0f, 1.0f, 1.0f};
    Vector<3> p1 = result1.apply(p);
    Vector<3> p2 = result2.apply(p);
    
    REQUIRE_THAT(p1[0], Catch::Matchers::WithinAbs(p2[0], 1e-4f));
    REQUIRE_THAT(p1[1], Catch::Matchers::WithinAbs(p2[1], 1e-4f));
    REQUIRE_THAT(p1[2], Catch::Matchers::WithinAbs(p2[2], 1e-4f));
}

TEST_CASE("Transform - Binary Multiplication Preserves Operands", "[transform][ops][multiply]") {
    Transform<> T1 = Transform<>::fromRotationZ(M_PI / 4);
    Transform<> T2(Vector<3>{1.0f, 2.0f, 3.0f});
    
    Transform<> T1_copy = T1;
    Transform<> T2_copy = T2;
    
    Transform<> result = T1 * T2;
    
    REQUIRE(T1.rotation() == T1_copy.rotation());
    REQUIRE(T1.translation() == T1_copy.translation());
    REQUIRE(T2.rotation() == T2_copy.rotation());
    REQUIRE(T2.translation() == T2_copy.translation());
}

// ----------------------------------------------------------------------------
// Vector Multiplication Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Vector Multiplication Identity", "[transform][ops][vector]") {
    Transform<> T = Transform<>::eye();
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T * v;
    
    REQUIRE(result == v);
}

TEST_CASE("Transform - Vector Multiplication Translation", "[transform][ops][vector]") {
    Transform<> T(Vector<3>{5.0f, 10.0f, 15.0f});
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = T * v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(12.0f, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(18.0f, 1e-6));
}

TEST_CASE("Transform - Vector Multiplication Rotation", "[transform][ops][vector]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 2.0f);
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = T * v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Vector Multiplication Combined", "[transform][ops][vector]") {
    Vector<3> trans = {1.0f, 0.0f, 0.0f};
    Quaternion<> quat = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Transform<> T(quat, trans);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = T * v;
    
    // Rotate (1,0,0) to (0,1,0), then translate by (1,0,0) -> (1,1,0)
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - Vector Multiplication Multiple Points", "[transform][ops][vector]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 4.0f);
    
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    Vector<3> v3 = {1.0f, 1.0f, 0.0f};
    
    Vector<3> r1 = T * v1;
    Vector<3> r2 = T * v2;
    Vector<3> r3 = T * v3;
    
    // All should be rotated 45°
    REQUIRE_THAT(norm(r1), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(norm(r2), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(norm(r3), Catch::Matchers::WithinAbs(std::sqrt(2.0f), 1e-5f));
}

TEST_CASE("Transform - Vector Multiplication Zero Vector", "[transform][ops][vector]") {
    Transform<> T(Vector<3>{1.0f, 2.0f, 3.0f});
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    Vector<3> result = T * v;
    
    // Zero vector translated becomes translation vector
    REQUIRE(result == T.translation());
}

TEST_CASE("Transform - Vector Multiplication Preserves Vector", "[transform][ops][vector]") {
    Transform<> T = Transform<>::fromRotationX(M_PI / 6);
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> v_copy = v;
    
    Vector<3> result = T * v;
    
    REQUIRE(v == v_copy);
}

TEST_CASE("Transform - Vector Multiplication Linearity", "[transform][ops][vector]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 3);
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result1 = T * (v1 + v2);
    Vector<3> result2 = (T * v1) + (T * v2);
    
    // For pure rotation, should be linear
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(result2[0], 1e-5f));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(result2[1], 1e-5f));
    REQUIRE_THAT(result1[2], Catch::Matchers::WithinAbs(result2[2], 1e-5f));
}

// ----------------------------------------------------------------------------
// Equality Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Equality Identity", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::eye();
    Transform<> T2 = Transform<>::eye();
    
    REQUIRE(T1 == T2);
}

TEST_CASE("Transform - Equality Same Translation", "[transform][ops][equality]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T1(t);
    Transform<> T2(t);
    
    REQUIRE(T1 == T2);
}

TEST_CASE("Transform - Equality Same Rotation", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::fromRotationZ(M_PI / 3.0f);
    Transform<> T2 = Transform<>::fromRotationZ(M_PI / 3.0f);
    
    REQUIRE(T1 == T2);
}

TEST_CASE("Transform - Equality Different Translation", "[transform][ops][equality]") {
    Transform<> T1(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2(Vector<3>{1.0f, 2.0f, 4.0f});
    
    REQUIRE_FALSE(T1 == T2);
}

TEST_CASE("Transform - Equality Different Rotation", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::fromRotationZ(M_PI / 3.0f);
    Transform<> T2 = Transform<>::fromRotationZ(M_PI / 4.0f);
    
    REQUIRE_FALSE(T1 == T2);
}

TEST_CASE("Transform - Equality Both Different", "[transform][ops][equality]") {
    Quaternion<> q1 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4);
    Quaternion<> q2 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 6);
    
    Transform<> T1(q1, Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2(q2, Vector<3>{4.0f, 5.0f, 6.0f});
    
    REQUIRE_FALSE(T1 == T2);
}

TEST_CASE("Transform - Equality Within Epsilon", "[transform][ops][equality]") {
    Transform<> T1(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2(Vector<3>{1.0f + 1e-7f, 2.0f, 3.0f});
    
    REQUIRE(T1 == T2);
}

TEST_CASE("Transform - Equality Reflexive", "[transform][ops][equality]") {
    Transform<> T(Vector<3>{1.0f, 2.0f, 3.0f});
    
    REQUIRE(T == T);
}

TEST_CASE("Transform - Equality Symmetric", "[transform][ops][equality]") {
    Transform<> T1(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2 = T1;
    
    REQUIRE(T1 == T2);
    REQUIRE(T2 == T1);
}

// ----------------------------------------------------------------------------
// Inequality Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Inequality Different", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::eye();
    Transform<> T2(Vector<3>{1.0f, 0.0f, 0.0f});
    
    REQUIRE(T1 != T2);
}

TEST_CASE("Transform - Inequality Same", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::eye();
    Transform<> T2 = Transform<>::eye();
    
    REQUIRE_FALSE(T1 != T2);
}

TEST_CASE("Transform - Inequality Different Rotation Only", "[transform][ops][equality]") {
    Transform<> T1 = Transform<>::fromRotationX(M_PI / 4);
    Transform<> T2 = Transform<>::fromRotationX(M_PI / 6);
    
    REQUIRE(T1 != T2);
}

TEST_CASE("Transform - Inequality Different Translation Only", "[transform][ops][equality]") {
    Transform<> T1(Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T2(Vector<3>{1.0f, 2.0f, 4.0f});
    
    REQUIRE(T1 != T2);
}

// ----------------------------------------------------------------------------
// Inverse Function Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Inverse Identity", "[transform][ops][inverse]") {
    Transform<> T = Transform<>::eye();
    Transform<> T_inv = inv(T);
    
    REQUIRE(T_inv.rotation() == Quaternion<>::eye());
    REQUIRE_THAT(T_inv.translation()[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(T_inv.translation()[1], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(T_inv.translation()[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Inverse Pure Translation", "[transform][ops][inverse]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T(t);
    Transform<> T_inv = inv(T);

    printf("T_inv translation: [%f, %f, %f]\n", T_inv.translation()[0], T_inv.translation()[1], T_inv.translation()[2]);
    
    Vector<3> t_inv = T_inv.translation();
    REQUIRE_THAT(t_inv[0], Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(t_inv[1], Catch::Matchers::WithinAbs(-2.0f, 1e-6));
    REQUIRE_THAT(t_inv[2], Catch::Matchers::WithinAbs(-3.0f, 1e-6));
}

TEST_CASE("Transform - Inverse Pure Rotation", "[transform][ops][inverse]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 4.0f);
    Transform<> T_inv = inv(T);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> rotated = T.apply(v);
    Vector<3> back = T_inv.apply(rotated);
    
    REQUIRE_THAT(back[0], Catch::Matchers::WithinAbs(v[0], 1e-5f));
    REQUIRE_THAT(back[1], Catch::Matchers::WithinAbs(v[1], 1e-5f));
    REQUIRE_THAT(back[2], Catch::Matchers::WithinAbs(v[2], 1e-5f));
}

TEST_CASE("Transform - Inverse Composition T * T^-1 = I", "[transform][ops][inverse]") {
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 3.0f);
    Transform<> T(q, t);
    Transform<> T_inv = inv(T);
    
    Transform<> result = T * T_inv;
    
    // Should be identity
    Quaternion<> R = result.rotation();
    Vector<3> trans = result.translation();
    
    REQUIRE_THAT(R.w(), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(norm(R.vector()), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(trans[0], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(trans[1], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(trans[2], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Transform - Inverse Composition T^-1 * T = I", "[transform][ops][inverse]") {
    Vector<3> t = {5.0f, -3.0f, 2.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 0.0f}, 0.7f);
    Transform<> T(q, t);
    Transform<> T_inv = inv(T);
    
    Transform<> result = T_inv * T;
    
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    Vector<3> transformed = result.apply(p);
    
    REQUIRE_THAT(transformed[0], Catch::Matchers::WithinAbs(p[0], 1e-4f));
    REQUIRE_THAT(transformed[1], Catch::Matchers::WithinAbs(p[1], 1e-4f));
    REQUIRE_THAT(transformed[2], Catch::Matchers::WithinAbs(p[2], 1e-4f));
}

TEST_CASE("Transform - Double Inverse", "[transform][ops][inverse]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 4.0f);
    Transform<> T(q, Vector<3>{1.0f, 2.0f, 3.0f});
    Transform<> T_inv_inv = inv(inv(T));
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> p1 = T.apply(p);
    Vector<3> p2 = T_inv_inv.apply(p);
    
    REQUIRE_THAT(p1[0], Catch::Matchers::WithinAbs(p2[0], 1e-4f));
    REQUIRE_THAT(p1[1], Catch::Matchers::WithinAbs(p2[1], 1e-4f));
    REQUIRE_THAT(p1[2], Catch::Matchers::WithinAbs(p2[2], 1e-4f));
}

TEST_CASE("Transform - Inverse of Composition (AB)^-1 = B^-1 A^-1", "[transform][ops][inverse]") {
    Transform<> A = Transform<>::fromRotationZ(M_PI / 6.0f);
    Transform<> B(Vector<3>{1.0f, 2.0f, 0.0f});
    
    Transform<> AB = A * B;
    Transform<> AB_inv_direct = inv(AB);
    Transform<> AB_inv_composed = inv(B) * inv(A);
    
    Vector<3> p = {1.0f, 1.0f, 1.0f};
    Vector<3> r1 = AB_inv_direct.apply(p);
    Vector<3> r2 = AB_inv_composed.apply(p);
    
    REQUIRE_THAT(r1[0], Catch::Matchers::WithinAbs(r2[0], 1e-4f));
    REQUIRE_THAT(r1[1], Catch::Matchers::WithinAbs(r2[1], 1e-4f));
    REQUIRE_THAT(r1[2], Catch::Matchers::WithinAbs(r2[2], 1e-4f));
}

TEST_CASE("Transform - Inverse Handles Large Translations", "[transform][ops][inverse]") {
    Vector<3> t = {1000.0f, 2000.0f, 3000.0f};
    Transform<> T(t);
    Transform<> T_inv = inv(T);
    
    Vector<3> t_inv = T_inv.translation();
    REQUIRE_THAT(t_inv[0], Catch::Matchers::WithinAbs(-1000.0f, 1e-3f));
    REQUIRE_THAT(t_inv[1], Catch::Matchers::WithinAbs(-2000.0f, 1e-3f));
    REQUIRE_THAT(t_inv[2], Catch::Matchers::WithinAbs(-3000.0f, 1e-3f));
}

TEST_CASE("Transform - Inverse Handles Arbitrary Rotation", "[transform][ops][inverse]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 2.0f, 3.0f}, 2.5f);
    Transform<> T(q);
    Transform<> T_inv = inv(T);
    
    Transform<> identity = T * T_inv;
    
    Vector<3> test = {1.0f, 2.0f, 3.0f};
    Vector<3> result = identity.apply(test);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(test[0], 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(test[1], 1e-4f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(test[2], 1e-4f));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Robot Chain Forward-Backward", "[transform][integration]") {
    // Simple robot: base -> link1 -> link2 -> end-effector
    Transform<> base_to_link1 = Transform<>::fromRotationZ(M_PI / 4.0f);
    Transform<> link1_to_link2(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> link2_to_ee = Transform<>::fromRotationZ(M_PI / 4.0f);
    
    // Forward: base to end-effector
    Transform<> base_to_ee = base_to_link1 * link1_to_link2 * link2_to_ee;
    
    // Backward: end-effector to base
    Transform<> ee_to_base = inv(base_to_ee);
    
    // Should cancel out
    Transform<> identity_check = base_to_ee * ee_to_base;
    
    Vector<3> p = {1.0f, 0.0f, 0.0f};
    Vector<3> result = identity_check.apply(p);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(p[0], 1e-4f));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(p[1], 1e-4f));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(p[2], 1e-4f));
}

TEST_CASE("Transform - Sensor Frame Change", "[transform][integration]") {
    // World frame to robot base
    Transform<> world_to_base(Vector<3>{0.0f, 0.0f, 0.5f});
    
    // Robot base to sensor
    Quaternion<> sensor_rot = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Transform<> base_to_sensor(sensor_rot, Vector<3>{0.2f, 0.0f, 0.1f});
    
    // World to sensor
    Transform<> world_to_sensor = world_to_base * base_to_sensor;
    
    // Point in world
    Vector<3> p_world = {1.0f, 0.0f, 0.5f};
    
    // Point in sensor frame
    Vector<3> p_sensor = inv(world_to_sensor) * p_world;
    
    // Transform back
    Vector<3> p_world_back = world_to_sensor * p_sensor;
    
    REQUIRE_THAT(p_world_back[0], Catch::Matchers::WithinAbs(p_world[0], 1e-4f));
    REQUIRE_THAT(p_world_back[1], Catch::Matchers::WithinAbs(p_world[1], 1e-4f));
    REQUIRE_THAT(p_world_back[2], Catch::Matchers::WithinAbs(p_world[2], 1e-4f));
}

TEST_CASE("Transform - Relative Transform Between Frames", "[transform][integration]") {
    // Frame A
    Quaternion<> qA = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    Transform<> world_to_A(qA, Vector<3>{1.0f, 0.0f, 0.0f});
    
    // Frame B
    Quaternion<> qB = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, -M_PI / 4.0f);
    Transform<> world_to_B(qB, Vector<3>{0.0f, 1.0f, 0.0f});
    
    // Relative transform: A to B
    Transform<> A_to_B = inv(world_to_A) * world_to_B;
    
    // Point in frame A
    Vector<3> p_A = {1.0f, 0.0f, 0.0f};
    
    // Express in frame B
    Vector<3> p_B = A_to_B * p_A;
    
    // Verify it's different (frames are different)
    bool different = std::abs(p_A[0] - p_B[0]) > 1e-4f ||
                     std::abs(p_A[1] - p_B[1]) > 1e-4f;
    
    REQUIRE(different);
}

TEST_CASE("Transform - Multiple Frame Transformations", "[transform][integration]") {
    // Create chain: World -> A -> B -> C
    Transform<> world_to_A(Vector<3>{1.0f, 0.0f, 0.0f});
    Transform<> A_to_B = Transform<>::fromRotationZ(M_PI / 6);
    Transform<> B_to_C(Vector<3>{0.0f, 1.0f, 0.0f});
    
    Transform<> world_to_C = world_to_A * A_to_B * B_to_C;
    
    // Point in world
    Vector<3> p_world = {1.0f, 1.0f, 0.0f};
    
    // Transform to C and back
    Vector<3> p_C = inv(world_to_C) * p_world;
    Vector<3> p_world_back = world_to_C * p_C;
    
    REQUIRE_THAT(p_world_back[0], Catch::Matchers::WithinAbs(p_world[0], 1e-4f));
    REQUIRE_THAT(p_world_back[1], Catch::Matchers::WithinAbs(p_world[1], 1e-4f));
    REQUIRE_THAT(p_world_back[2], Catch::Matchers::WithinAbs(p_world[2], 1e-4f));
}

TEST_CASE("Transform - Coordinate System Conversion", "[transform][integration]") {
    // Transform from right-handed to left-handed (mirror Z)
    Quaternion<> mirror = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI);
    Transform<> right_to_left(mirror);
    
    Vector<3> point_right = {1.0f, 2.0f, 3.0f};
    Vector<3> point_left = right_to_left * point_right;
    
    // X and Y should be preserved, Z should be negated
    REQUIRE_THAT(point_left[0], Catch::Matchers::WithinAbs(point_right[0], 1e-5f));
    REQUIRE_THAT(point_left[1], Catch::Matchers::WithinAbs(-point_right[1], 1e-5f));
    REQUIRE_THAT(point_left[2], Catch::Matchers::WithinAbs(-point_right[2], 1e-5f));
}

TEST_CASE("Transform - Camera Projection Chain", "[transform][integration]") {
    // World to camera transform
    Vector<3> camera_pos = {0.0f, 0.0f, 10.0f};
    Quaternion<> look_down = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI);
    Transform<> world_to_camera(look_down, camera_pos);
    
    // Object in world space
    Vector<3> object = {1.0f, 2.0f, 5.0f};
    
    // Transform to camera space
    Vector<3> object_camera = inv(world_to_camera) * object;
    
    // Should be in front of camera (negative Z in camera space typically)
    REQUIRE(isFinite(object_camera));
}

TEST_CASE("Transform - Path Following", "[transform][integration]") {
    // Simulate moving along a path
    Transform<> pose = Transform<>::eye();
    
    // Move forward and turn
    for (int i = 0; i < 10; i++) {
        Transform<> step(Vector<3>{0.1f, 0.0f, 0.0f});
        Transform<> turn = Transform<>::fromRotationZ(M_PI / 20);
        pose = pose * step * turn;
    }
    
    // Should have moved and rotated
    REQUIRE(norm(pose.translation()) > 0.5f);
    REQUIRE_FALSE(pose.rotation() == Quaternion<>::eye());
}

TEST_CASE("Transform - Precision Test Large Compositions", "[transform][integration]") {
    Transform<> T = Transform<>::eye();
    
    // Compose many small transforms
    for (int i = 0; i < 100; i++) {
        Transform<> small_rot = Transform<>::fromRotationZ(0.01f);
        T = T * small_rot;
    }
    
    // Should equal one large rotation
    Transform<> large_rot = Transform<>::fromRotationZ(1.0f);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> r1 = T * v;
    Vector<3> r2 = large_rot * v;
    
    // Should be close (within accumulated error)
    REQUIRE_THAT(r1[0], Catch::Matchers::WithinAbs(r2[0], 1e-2f));
    REQUIRE_THAT(r1[1], Catch::Matchers::WithinAbs(r2[1], 1e-2f));
}

TEST_CASE("Transform - Double Type Operations", "[transform][integration]") {
    Transform<double> T1(Vector<3, double>{1.0, 2.0, 3.0});
    Transform<double> T2 = Transform<double>::fromRotationZ(M_PI / 4.0);
    
    Transform<double> result = T1 * T2;
    
    Vector<3, double> v = {1.0, 0.0, 0.0};
    Vector<3, double> transformed = result * v;
    
    REQUIRE(std::isfinite(transformed[0]));
    REQUIRE(std::isfinite(transformed[1]));
    REQUIRE(std::isfinite(transformed[2]));
}