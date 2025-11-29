#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion Util Tests (quaternion_util.hpp)
// ============================================================================

TEST_CASE("Quaternion - isZero True", "[quaternion][util][check]") {
    Quaternion q = Quaternion::zero();
    
    REQUIRE(isZero(q));
}

TEST_CASE("Quaternion - isZero False", "[quaternion][util][check]") {
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isZero(q));
}

TEST_CASE("Quaternion - isZero Near Zero", "[quaternion][util][check]") {
    Quaternion q(1e-6f, 1e-6f, 1e-6f, 1e-6f);
    
    REQUIRE(isZero(q));
}

TEST_CASE("Quaternion - isIdentity True", "[quaternion][util][check]") {
    Quaternion q = Quaternion::eye();
    
    REQUIRE(isIdentity(q));
}

TEST_CASE("Quaternion - isIdentity False", "[quaternion][util][check]") {
    Quaternion q(1.0f, 0.1f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isIdentity(q));
}

TEST_CASE("Quaternion - isNormalized True", "[quaternion][util][check]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 4.0f);
    
    REQUIRE(isNormalized(q));
}

TEST_CASE("Quaternion - isNormalized False", "[quaternion][util][check]") {
    Quaternion q(2.0f, 3.0f, 4.0f, 5.0f);
    
    REQUIRE_FALSE(isNormalized(q));
}

TEST_CASE("Quaternion - isPure True", "[quaternion][util][check]") {
    Quaternion q = Quaternion::pure(Vector<3>{1.0f, 2.0f, 3.0f});
    
    REQUIRE(isPure(q));
}

TEST_CASE("Quaternion - isPure False", "[quaternion][util][check]") {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_FALSE(isPure(q));
}

TEST_CASE("Quaternion - isReal True", "[quaternion][util][check]") {
    Quaternion q(5.0f, 0.0f, 0.0f, 0.0f);
    
    REQUIRE(isReal(q));
}

TEST_CASE("Quaternion - isReal False", "[quaternion][util][check]") {
    Quaternion q(5.0f, 0.1f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isReal(q));
}

TEST_CASE("Quaternion - isSameRotation Identical", "[quaternion][util][check]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    
    REQUIRE(isSameRotation(q1, q2));
}

TEST_CASE("Quaternion - isSameRotation Opposite Sign", "[quaternion][util][check]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    Quaternion q2 = -q1;
    
    REQUIRE(isSameRotation(q1, q2));
}

TEST_CASE("Quaternion - isSameRotation Different", "[quaternion][util][check]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 4.0f);
    
    REQUIRE_FALSE(isSameRotation(q1, q2));
}

TEST_CASE("Quaternion - toRotationVector Zero Rotation", "[quaternion][util][convert]") {
    Quaternion q = Quaternion::eye();
    
    Vector<3> result = toRotationVector(q);
    
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - toRotationVector Round-Trip", "[quaternion][util][convert]") {
    Vector<3> origVec = {0.0f, M_PI / 3.0f, 0.0f};
    
    Quaternion q = Quaternion::rotationVector(origVec);
    Vector<3> result = toRotationVector(q);
    
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(origVec.x(), 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(origVec.y(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(origVec.z(), 1e-4f));
}

TEST_CASE("Quaternion - toAngle Identity", "[quaternion][util][convert]") {
    Quaternion q = Quaternion::eye();
    
    float angle = toAngle(q);
    
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - toAngle 90 Degrees", "[quaternion][util][convert]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 2.0f);
    
    float angle = toAngle(q);
    
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(M_PI / 2.0f, 1e-4f));
}

TEST_CASE("Quaternion - toAxis X-Axis", "[quaternion][util][convert]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 3.0f);
    
    Vector<3> axis = toAxis(q);
    
    REQUIRE_THAT(axis.x(), Catch::Matchers::WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(axis.y(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(axis.z(), Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Quaternion - toAxis Identity Returns Arbitrary", "[quaternion][util][convert]") {
    Quaternion q = Quaternion::eye();
    
    Vector<3> axis = toAxis(q);
    
    // Should return some normalized axis (arbitrary for zero rotation)
    float axisNorm = std::sqrt(axis.x()*axis.x() + axis.y()*axis.y() + axis.z()*axis.z());
    REQUIRE_THAT(axisNorm, Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - clamp Within Range", "[quaternion][util][clamp]") {
    Quaternion q(0.5f, 0.3f, 0.2f, 0.1f);
    
    Quaternion result = clamp(q, 1.0f);
    
    REQUIRE(result == q);  // Should be unchanged
}

TEST_CASE("Quaternion - clamp Outside Range", "[quaternion][util][clamp]") {
    Quaternion q(2.0f, -3.0f, 4.0f, -5.0f);
    
    Quaternion result = clamp(q, 1.5f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.5f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-1.5f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(1.5f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-1.5f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - round Basic", "[quaternion][util][round]") {
    Quaternion q(1.7f, 2.3f, 3.5f, 4.1f);
    
    Quaternion result = round(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(4.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - cleanZero All Below Threshold", "[quaternion][util][round]") {
    Quaternion q(1e-6f, 1e-7f, 1e-8f, 1e-9f);
    
    Quaternion result = cleanZero(q);
    
    REQUIRE(isZero(result));
}

TEST_CASE("Quaternion - cleanZero Some Above Threshold", "[quaternion][util][round]") {
    Quaternion q(1e-3f, 1e-7f, 2e-3f, 1e-9f);
    
    Quaternion result = cleanZero(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1e-3f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(2e-3f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - angledDistance Same Rotation", "[quaternion][util][distance]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 4.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 4.0f);
    
    float dist = angledDistance(q1, q2);
    
    REQUIRE_THAT(dist, Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Quaternion - angledDistance 90 Degrees", "[quaternion][util][distance]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    float dist = angledDistance(q1, q2);
    
    REQUIRE_THAT(dist, Catch::Matchers::WithinAbs(M_PI / 2.0f, 1e-4f));
}

TEST_CASE("Quaternion - difference Identity to Rotation", "[quaternion][util][distance]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    
    Quaternion diff = difference(q1, q2);
    
    // diff * q1 should equal q2
    Quaternion result = diff * q1;
    REQUIRE(isSameRotation(result, q2));
}

TEST_CASE("Quaternion - shortestPath Same Sign", "[quaternion][util][rotate]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(0.9f, 0.1f, 0.0f, 0.0f);
    
    Quaternion result = shortestPath(q1, q2);
    
    // Should return q2 unchanged (dot product > 0)
    REQUIRE(result == q2);
}

TEST_CASE("Quaternion - shortestPath Opposite Sign", "[quaternion][util][rotate]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(-0.9f, -0.1f, 0.0f, 0.0f);
    
    Quaternion result = shortestPath(q1, q2);
    
    // Should return -q2 (dot product < 0)
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.9f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.1f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - swingTwist Z-Axis", "[quaternion][util][rotate]") {
    Vector<3> axis = {0.0f, 0.0f, 1.0f};
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, M_PI / 4.0f);
    
    Quaternion swing, twist;
    swingTwist(q, axis, swing, twist);
    
    // Recomposing should give original
    Quaternion recomposed = swing * twist;
    REQUIRE(isSameRotation(recomposed, q));
}

TEST_CASE("Quaternion - toAngularVelocity Static", "[quaternion][util][rotate]") {
    Quaternion q = Quaternion::eye();
    Quaternion qDot = Quaternion::zero();
    
    Vector<3> omega = angularVelocity(q, qDot);
    
    // No rotation, zero velocity
    REQUIRE_THAT(omega.x(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(omega.y(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(omega.z(), Catch::Matchers::WithinAbs(0.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - Full Rotation Cycle Check", "[quaternion][integration]") {
    Vector<3> axis = {1.0f, 1.0f, 1.0f};
    float angle = M_PI / 3.0f;
    
    // Create quaternion
    Quaternion q = Quaternion::axisAngle(axis, angle);
    
    // Verify it's normalized
    REQUIRE(isNormalized(q));
    
    // Extract and verify
    float extractedAngle = toAngle(q);
    Vector<3> extractedAxis = toAxis(q);
    
    REQUIRE_THAT(extractedAngle, Catch::Matchers::WithinAbs(angle, 1e-4f));
}

TEST_CASE("Quaternion - Angular Distance Symmetry", "[quaternion][integration]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 6.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 4.0f);
    
    float d1 = angledDistance(q1, q2);
    float d2 = angledDistance(q2, q1);
    
    REQUIRE_THAT(d1, Catch::Matchers::WithinAbs(d2, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - Difference Composition", "[quaternion][integration]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 6.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    
    Quaternion diff = difference(q1, q2);
    Quaternion composed = diff * q1;
    
    REQUIRE(isSameRotation(composed, q2));
}

TEST_CASE("Quaternion - SLERP Start", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = slerp(q1, q2, 0.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q1.w(), QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q1.x(), QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q1.y(), QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q1.z(), QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - SLERP End", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = slerp(q1, q2, 1.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q2.w(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q2.z(), 1e-4f));
}

TEST_CASE("Quaternion - SLERP Midpoint", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = slerp(q1, q2, 0.5f);
    Quaternion expected = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(expected.w(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(expected.z(), 1e-4f));
}

TEST_CASE("Quaternion - SLERP Unit Quaternion Output", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 6.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    
    Quaternion result = slerp(q1, q2, 0.3f);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - SLERP Handles Opposite Quaternions", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = -Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    
    Quaternion result = slerp(q1, q2, 0.5f);
    
    // Should still produce valid unit quaternion
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - SLERP Very Close Quaternions", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, 0.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, 0.001f);
    
    Quaternion result = slerp(q1, q2, 0.5f);
    
    // Should still work (falls back to lerp)
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - SLERP Constant Angular Velocity", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    // Sample at quarter points
    Quaternion at25 = slerp(q1, q2, 0.25f);
    Quaternion at50 = slerp(q1, q2, 0.50f);
    Quaternion at75 = slerp(q1, q2, 0.75f);
    
    // Angular distances should be roughly equal
    float dist1 = angledDistance(q1, at25);
    float dist2 = angledDistance(at25, at50);
    float dist3 = angledDistance(at50, at75);
    
    REQUIRE_THAT(dist1, Catch::Matchers::WithinAbs(dist2, 1e-4f));
    REQUIRE_THAT(dist2, Catch::Matchers::WithinAbs(dist3, 1e-4f));
}

TEST_CASE("Quaternion - NLERP Start", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = nlerp(q1, q2, 0.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q1.w(), QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - NLERP End", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = nlerp(q1, q2, 1.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q2.w(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q2.z(), 1e-4f));
}

TEST_CASE("Quaternion - NLERP Produces Unit Quaternion", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Quaternion result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - NLERP Similar to SLERP for Small Angles", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 8.0f);
    
    Quaternion slerped = slerp(q1, q2, 0.5f);
    Quaternion nlerped = nlerp(q1, q2, 0.5f);
    
    // Should be reasonably close for small angles
    float diff = std::abs(slerped.w() - nlerped.w()) + 
                 std::abs(slerped.x() - nlerped.x()) +
                 std::abs(slerped.y() - nlerped.y()) +
                 std::abs(slerped.z() - nlerped.z());
    
    REQUIRE(diff < 0.05f);
}

TEST_CASE("Quaternion - NLERP Handles Opposite Sign", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = -Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 4.0f);
    
    Quaternion result = nlerp(q1, q2, 0.5f);
    
    // Should take shorter path
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - NLERP Multiple Steps", "[quaternion][util][interp]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    // Interpolate in steps
    Quaternion result = q1;
    for(int i = 0; i < 10; i++) {
        result = nlerp(result, q2, 0.1f*i);
    }
    
    // Should be close to q2
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q2.w(), 1e-2f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q2.x(), 1e-2f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q2.y(), 1e-2f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q2.z(), 1e-2f));
}

TEST_CASE("Quaternion - shortestPath Same Sign", "[quaternion][util][path]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(0.9f, 0.1f, 0.0f, 0.0f);
    
    Quaternion result = shortestPath(q1, q2);
    
    // Should return q2 unchanged (dot product > 0)
    REQUIRE(result == q2);
}

TEST_CASE("Quaternion - shortestPath Opposite Sign", "[quaternion][util][path]") {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(-0.9f, -0.1f, 0.0f, 0.0f);
    
    Quaternion result = shortestPath(q1, q2);
    
    // Should return -q2 (dot product < 0)
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.9f, QUATERNION_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.1f, QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - shortestPath Prevents 360-deg Rotation", "[quaternion][util][path]") {
    Quaternion q1 = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.1f);
    Quaternion q2 = -q1;  // Same rotation, opposite representation
    
    Quaternion adjusted = shortestPath(q1, q2);
    
    // Should negate to avoid long path
    float d1 = dot(q1, q2);
    float d2 = dot(q1, adjusted);
    
    REQUIRE(d2 > d1);  // Adjusted should be closer
}

TEST_CASE("Quaternion - shortestPath Preserves Magnitude", "[quaternion][util][path]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, M_PI);
    
    Quaternion result = shortestPath(q1, q2);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(norm(q2), QUATERNION_EQUAL_THRESHOLD));
}

TEST_CASE("Quaternion - shortestPath Idempotent", "[quaternion][util][path]") {
    Quaternion q1 = Quaternion::eye();
    Quaternion q2 = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 3.0f);
    
    Quaternion once = shortestPath(q1, q2);
    Quaternion twice = shortestPath(q1, once);
    
    REQUIRE(once == twice);
}

TEST_CASE("Quaternion - shortestPath Critical for Control", "[quaternion][util][path]") {
    // Simulate servo control scenario
    Quaternion current = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI - 0.1f);
    Quaternion target = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, -M_PI + 0.1f);
    
    // Without shortestPath, these might choose the long way around
    Quaternion adjusted = shortestPath(current, target);
    
    float distDirect = angledDistance(current, target);
    float distAdjusted = angledDistance(current, adjusted);
    
    // Adjusted path should be shorter or equal
    REQUIRE(distAdjusted <= distDirect + 1e-4f);
}

TEST_CASE("Quaternion - clampRotation Within Limit", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, M_PI / 6.0f);
    
    Quaternion result = clampRotation(q, M_PI / 3.0f);
    
    // Should be unchanged
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-4f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-4f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-4f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-4f));
}

TEST_CASE("Quaternion - clampRotation Exceeds Limit", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI);
    float maxAngle = M_PI / 2.0f;
    
    Quaternion result = clampRotation(q, maxAngle);
    
    // Extract angle to verify it's clamped
    float resultAngle = 2.0f * std::acos(std::clamp(result.w(), -1.0f, 1.0f));
    
    REQUIRE_THAT(resultAngle, Catch::Matchers::WithinAbs(maxAngle, 1e-4f));
}

TEST_CASE("Quaternion - clampRotation Preserves Axis", "[quaternion][util][clamp]") {
    Vector<3> axis = {1.0f, 1.0f, 1.0f};
    Quaternion q = Quaternion::axisAngle(axis, M_PI);
    
    Quaternion result = clampRotation(q, M_PI / 4.0f);
    
    // Extract axis from result
    float sinHalf = std::sqrt(1.0f - result.w() * result.w());
    
    if(sinHalf > QUATERNION_EQUAL_THRESHOLD) {
        Vector<3> resultAxis{
            result.x() / sinHalf,
            result.y() / sinHalf,
            result.z() / sinHalf
        };
        
        // Normalize original axis for comparison
        float axisNorm = std::sqrt(axis.x()*axis.x() + axis.y()*axis.y() + axis.z()*axis.z());
        Vector<3> axisNorm_vec = axis / axisNorm;
        
        // Axes should be parallel
        REQUIRE_THAT(resultAxis.x(), Catch::Matchers::WithinAbs(axisNorm_vec.x(), 1e-4f));
        REQUIRE_THAT(resultAxis.y(), Catch::Matchers::WithinAbs(axisNorm_vec.y(), 1e-4f));
        REQUIRE_THAT(resultAxis.z(), Catch::Matchers::WithinAbs(axisNorm_vec.z(), 1e-4f));
    }
}

TEST_CASE("Quaternion - clampRotation Zero Limit", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 4.0f);
    
    Quaternion result = clampRotation(q, 0.0f);
    
    // Should return identity
    REQUIRE(isIdentity(result));
}

TEST_CASE("Quaternion - clampRotation Unit Quaternion Output", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, 2.0f * M_PI / 3.0f);
    
    Quaternion result = clampRotation(q, M_PI / 6.0f);
    
    REQUIRE(isNormalized(result));
}

TEST_CASE("Quaternion - clampRotation Large Limit", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::axisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 4.0f);
    
    Quaternion result = clampRotation(q, 10.0f * M_PI);
    
    // Should be unchanged (within limit)
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-4f));
}

TEST_CASE("Quaternion - clampRotation Identity Input", "[quaternion][util][clamp]") {
    Quaternion q = Quaternion::eye();
    
    Quaternion result = clampRotation(q, M_PI / 4.0f);
    
    // Should remain identity
    REQUIRE(isIdentity(result));
}

TEST_CASE("Quaternion - clampRotation Rate Limiting Use Case", "[quaternion][util][clamp]") {
    // Simulate rate limiting in control loop
    float maxRatePerSec = M_PI / 4.0f;  // 45°/sec
    float dt = 0.1f;  // 100ms
    float maxAngle = maxRatePerSec * dt;  // 4.5°
    
    Quaternion desired = Quaternion::axisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    Quaternion limited = clampRotation(desired, maxAngle);
    
    float resultAngle = 2.0f * std::acos(std::clamp(limited.w(), -1.0f, 1.0f));
    
    REQUIRE(resultAngle <= maxAngle + 1e-4f);
}