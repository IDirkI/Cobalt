#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion<> Utility Tests (quaternion_util.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Clamping Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Clamp Within Range", "[quaternion][util][clamp]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = clamp(q, 0.0f, 5.0f);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Clamp Above Max", "[quaternion][util][clamp]") {
    Quaternion<> q(1.0f, 7.0f, 3.0f, 9.0f);
    
    Quaternion<> result = clamp(q, 0.0f, 5.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Quaternion - Clamp Below Min", "[quaternion][util][clamp]") {
    Quaternion<> q(-5.0f, 2.0f, -1.0f, 4.0f);
    
    Quaternion<> result = clamp(q, 0.0f, 5.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Clamp Mixed Values", "[quaternion][util][clamp]") {
    Quaternion<> q(-2.0f, 3.0f, 8.0f, 1.0f);
    
    Quaternion<> result = clamp(q, 0.0f, 5.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - Clamp Symmetric Absolute", "[quaternion][util][clamp]") {
    Quaternion<> q(-7.0f, 2.0f, 8.0f, -1.0f);
    
    Quaternion<> result = clamp(q, 5.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(-5.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Quaternion - Clamp Zero Quaternion", "[quaternion][util][clamp]") {
    Quaternion<> zero = Quaternion<>::zero();
    
    Quaternion<> result = clamp(zero, -1.0f, 1.0f);
    
    REQUIRE(result == zero);
}

TEST_CASE("Quaternion - Clamp Rotation Angle Within Range", "[quaternion][util][clamp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    
    Quaternion<> result = clampRotation(q, 0.0f, (float)M_PI);
    
    // Should remain unchanged as angle is within range
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-6));
}

TEST_CASE("Quaternion - Clamp Rotation Angle Above Max", "[quaternion][util][clamp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI);
    
    Quaternion<> result = clampRotation(q, 0.0f, (float)M_PI / 2);
    
    // Angle should be clamped to PI/2
    float angle = toAngle(result);
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs((float)M_PI / 2, 1e-5));
}

TEST_CASE("Quaternion - Clamp Rotation Angle Below Min", "[quaternion][util][clamp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 6);
    
    Quaternion<> result = clampRotation(q, (float)M_PI / 4, (float)M_PI);
    
    // Angle should be clamped to PI/4
    float angle = toAngle(result);
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs((float)M_PI / 4, 1e-5));
}

TEST_CASE("Quaternion - Clamp Rotation Symmetric", "[quaternion][util][clamp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI);
    
    Quaternion<> result = clampRotation(q, (float)M_PI / 2);
    
    float angle = toAngle(result);
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs((float)M_PI / 2, 1e-5));
}

TEST_CASE("Quaternion - Clamp Rotation Preserves Axis", "[quaternion][util][clamp]") {
    Vector<3> axis = normalize(Vector<3>{1.0f, 1.0f, 0.0f});
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, (float)M_PI);
    
    Quaternion<> result = clampRotation(q, (float)M_PI / 4);
    
    Vector<3> resultAxis = toAxis(result);
    
    // Axis should remain the same
    REQUIRE_THAT(dot(axis, resultAxis), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

// ----------------------------------------------------------------------------
// Element-wise Operation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Round Elements", "[quaternion][util][element]") {
    Quaternion<> q(1.4f, 2.6f, -3.5f, 4.2f);
    
    Quaternion<> result = round(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-4.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Round All Integers", "[quaternion][util][element]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = round(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Ceil Elements", "[quaternion][util][element]") {
    Quaternion<> q(1.2f, -2.7f, 0.5f, 3.1f);
    
    Quaternion<> result = ceil(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Floor Elements", "[quaternion][util][element]") {
    Quaternion<> q(1.7f, -2.3f, 0.5f, 3.9f);
    
    Quaternion<> result = floor(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Quaternion - Clean Zero", "[quaternion][util][element]") {
    Quaternion<> q(1e-10f, 2.0f, -1e-10f, 3.0f);
    
    Quaternion<> result = cleanZero(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Quaternion - Clean Zero Preserves Significant", "[quaternion][util][element]") {
    Quaternion<> q(0.001f, 2.0f, 0.0001f, 3.0f);
    
    Quaternion<> result = cleanZero(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.001, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0001, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Angles & Difference Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Angled Distance Identity", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::eye();
    Quaternion<> p = Quaternion<>::eye();
    
    float distance = angledDistance(q, p);
    
    REQUIRE_THAT(distance, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Angled Distance 90 Degrees", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2);
    
    float distance = angledDistance(q, p);
    
    REQUIRE_THAT(distance, Catch::Matchers::WithinAbs((float)M_PI / 2, 1e-5));
}

TEST_CASE("Quaternion - Angled Distance 180 Degrees", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI);
    
    float distance = angledDistance(q, p);
    
    REQUIRE_THAT(distance, Catch::Matchers::WithinAbs((float)M_PI, 1e-5));
}

TEST_CASE("Quaternion - Angled Distance Symmetric", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 4);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 3);
    
    float dist1 = angledDistance(q, p);
    float dist2 = angledDistance(p, q);
    
    REQUIRE_THAT(dist1, Catch::Matchers::WithinAbs(dist2, 1e-6));
}

TEST_CASE("Quaternion - Angled Distance Non-Negative", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 6);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{-1.0f, 0.0f, 1.0f}, (float)M_PI / 3);
    
    float distance = angledDistance(q, p);
    
    REQUIRE(distance >= 0.0f);
}

TEST_CASE("Quaternion - Difference Identity", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Quaternion<> diff = difference(q, q);
    
    REQUIRE(isIdentity(diff));
}

TEST_CASE("Quaternion - Difference Returns Rotation", "[quaternion][util][angle]") {
    Quaternion<> q1 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> q2 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2);
    
    Quaternion<> diff = difference(q1, q2);
    
    // diff should represent the rotation from q1 to q2
    Quaternion<> result = q1 * diff;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q2.w(), 1e-5));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q2.x(), 1e-5));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q2.y(), 1e-5));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q2.z(), 1e-5));
}

TEST_CASE("Quaternion - Shortest Path Same Quaternion", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 4);
    
    Quaternion<> result = shortestPath(q, q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Quaternion - Shortest Path Negative Dot Product", "[quaternion][util][angle]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 6);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, -5.0f * (float)M_PI / 6);
    
    Quaternion<> result = shortestPath(q, p);
    
    // Result should ensure shortest path is taken
    float dotProd = dot(normalize(q), normalize(result));
    REQUIRE(dotProd >= 0.0f);
}

// ----------------------------------------------------------------------------
// Dynamics & Motion Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Angular Velocity Zero Rate", "[quaternion][util][dynamics]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Quaternion<> qDot = Quaternion<>::zero();
    
    Vector<3> omega = angularVelocity(q, qDot);
    
    REQUIRE_THAT(norm(omega), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Angular Velocity Non-Zero Rate", "[quaternion][util][dynamics]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Quaternion<> qDot(0.1f, 0.0f, 0.0f, 0.5f);
    
    Vector<3> omega = angularVelocity(q, qDot);
    
    // Angular velocity should be non-zero
    REQUIRE(norm(omega) > 0.0f);
}

TEST_CASE("Quaternion - Swing Twist Decomposition Z-Axis", "[quaternion][util][dynamics]") {
    Vector<3> axis = Vector<3>{0.0f, 0.0f, 1.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 4);
    
    Quaternion<> swing, twist;
    swingTwist(q, axis, swing, twist);
    
    // Reconstruct should equal original
    Quaternion<> reconstructed = swing * twist;
    
    REQUIRE_THAT(reconstructed.w(), Catch::Matchers::WithinAbs(q.w(), 1e-5));
    REQUIRE_THAT(reconstructed.x(), Catch::Matchers::WithinAbs(q.x(), 1e-5));
    REQUIRE_THAT(reconstructed.y(), Catch::Matchers::WithinAbs(q.y(), 1e-5));
    REQUIRE_THAT(reconstructed.z(), Catch::Matchers::WithinAbs(q.z(), 1e-5));
}

TEST_CASE("Quaternion - Swing Twist Decomposition Pure Twist", "[quaternion][util][dynamics]") {
    Vector<3> axis = Vector<3>{0.0f, 0.0f, 1.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, (float)M_PI / 4);
    
    Quaternion<> swing, twist;
    swingTwist(q, axis, swing, twist);
    
    // Swing should be identity for pure twist
    REQUIRE(isIdentity(swing));
}

TEST_CASE("Quaternion - Swing Twist Decomposition Pure Swing", "[quaternion][util][dynamics]") {
    Vector<3> twistAxis = Vector<3>{0.0f, 0.0f, 1.0f};
    Vector<3> swingAxis = Vector<3>{1.0f, 0.0f, 0.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(swingAxis, (float)M_PI / 4);
    
    Quaternion<> swing, twist;
    swingTwist(q, twistAxis, swing, twist);
    
    // Twist should be identity for pure swing
    REQUIRE(isIdentity(twist));
}

TEST_CASE("Quaternion - Swing Twist Twist Axis Alignment", "[quaternion][util][dynamics]") {
    Vector<3> axis = Vector<3>{0.0f, 1.0f, 0.0f};
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 0.0f}, (float)M_PI / 3);
    
    Quaternion<> swing, twist;
    swingTwist(q, axis, swing, twist);
    
    // Twist component should be aligned with axis
    Vector<3> twistVec = twist.vector();
    Vector<3> twistDir = normalize(twistVec);
    
    if (norm(twistVec) > 1e-5) {
        REQUIRE_THAT(std::abs(dot(twistDir, axis)), Catch::Matchers::WithinAbs(1.0, 1e-4));
    }
}

// ----------------------------------------------------------------------------
// Interpolation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Slerp Boundaries", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 2);
    
    Quaternion<> result0 = slerp(q, p, 0.0f);
    Quaternion<> result1 = slerp(q, p, 1.0f);
    
    REQUIRE_THAT(result0.w(), Catch::Matchers::WithinAbs(q.w(), 1e-5));
    REQUIRE_THAT(result1.w(), Catch::Matchers::WithinAbs(p.w(), 1e-5));
}

TEST_CASE("Quaternion - Slerp Midpoint", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2);
    
    Quaternion<> result = slerp(q, p, 0.5f);
    
    // At t=0.5, should be at PI/4 rotation
    float angle = toAngle(result);
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs((float)M_PI / 4, 1e-5));
}

TEST_CASE("Quaternion - Slerp Maintains Unit Norm", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 3);
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quaternion<> result = slerp(q, p, t);
        float n = norm(result);
        REQUIRE_THAT(n, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

TEST_CASE("Quaternion - Slerp Opposite Quaternions", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> p = -q;
    
    Quaternion<> result = slerp(q, p, 0.5f);
    
    // Should handle negative dot product case
    REQUIRE(isNormalized(result));
}

TEST_CASE("Quaternion - Nlerp Boundaries", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 2);
    
    Quaternion<> result0 = nlerp(q, p, 0.0f);
    Quaternion<> result1 = nlerp(q, p, 1.0f);
    
    REQUIRE_THAT(result0.w(), Catch::Matchers::WithinAbs(q.w(), 1e-5));
    REQUIRE_THAT(result1.w(), Catch::Matchers::WithinAbs(p.w(), 1e-5));
}

TEST_CASE("Quaternion - Nlerp Maintains Unit Norm", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 3);
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quaternion<> result = nlerp(q, p, t);
        float n = norm(result);
        REQUIRE_THAT(n, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

TEST_CASE("Quaternion - Lerp Boundaries", "[quaternion][util][interp]") {
    Quaternion<> q(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion<> p(0.0f, 1.0f, 0.0f, 0.0f);
    
    Quaternion<> result0 = lerp(q, p, 0.0f);
    Quaternion<> result1 = lerp(q, p, 1.0f);
    
    REQUIRE(result0 == q);
    REQUIRE(result1 == p);
}

TEST_CASE("Quaternion - Lerp Not Normalized", "[quaternion][util][interp]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, 0.0f);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 2);
    
    Quaternion<> result = lerp(q, p, 0.5f);
    
    // Lerp result is not necessarily normalized
    REQUIRE(norm(result) != 1.0f);
}

// ----------------------------------------------------------------------------
// Conversion Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - To Array", "[quaternion][util][conversion]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    std::array<float, 4> arr = toArray(q);
    
    REQUIRE_THAT(arr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(arr[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(arr[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(arr[3], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - To Matrix Identity", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Matrix<3, 3> R = toMatrix(q);
    
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(R(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5));
        }
    }
}

TEST_CASE("Quaternion - To Matrix 90 Degree Z Rotation", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2);
    
    Matrix<3, 3> R = toMatrix(q);
    
    // 90 degree Z rotation: x -> y, y -> -x, z -> z
    REQUIRE_THAT(R(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(R(0, 1), Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(R(1, 0), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(R(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(R(2, 2), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Quaternion - To Matrix Orthogonal", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 3);
    
    Matrix<3, 3> R = toMatrix(q);
    
    // R should be orthogonal: R^T * R = I
    Matrix<3, 3> RtR = transpose(R) * R;
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(RtR(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5));
        }
    }
}

TEST_CASE("Quaternion - To Matrix Determinant One", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 1.0f, 0.0f}, (float)M_PI / 4);
    
    Matrix<3, 3> R = toMatrix(q);
    
    float determinant = det(R);
    REQUIRE_THAT(determinant, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Quaternion - To Axis Angle Z-Axis", "[quaternion][util][conversion]") {
    Vector<3> axis = Vector<3>{0.0f, 0.0f, 1.0f};
    float angle = (float)M_PI / 4;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> result = toRotationVector(q);
    
    // Result should be axis scaled by angle
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(angle, 1e-5));
    REQUIRE_THAT(dot(normalize(result), axis), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Quaternion - To Axis Angle Identity", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Vector<3> result = toRotationVector(q);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - To Angle", "[quaternion][util][conversion]") {
    float angle = (float)M_PI / 3;
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, angle);
    
    float resultAngle = toAngle(q);
    
    REQUIRE_THAT(resultAngle, Catch::Matchers::WithinAbs(angle, 1e-5));
}

TEST_CASE("Quaternion - To Angle Identity", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::eye();
    
    float angle = toAngle(q);
    
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - To Axis", "[quaternion][util][conversion]") {
    Vector<3> axis = normalize(Vector<3>{1.0f, 1.0f, 0.0f});
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, (float)M_PI / 4);
    
    Vector<3> resultAxis = toAxis(q);
    
    REQUIRE_THAT(dot(resultAxis, axis), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(norm(resultAxis), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - To Axis Identity Returns Default", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Vector<3> axis = toAxis(q);
    
    // Should return a default axis (implementation returns +x)
    REQUIRE_THAT(norm(axis), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - To Euler ZYX Sequence", "[quaternion][util][conversion]") {
    float expectedRoll = (float)M_PI / 6;
    float expectedPitch = (float)M_PI / 4;
    float expectedYaw = (float)M_PI / 3;
    
    Quaternion<> q = Quaternion<>::fromEuler(expectedRoll, expectedPitch, expectedYaw);
    
    float roll, pitch, yaw;
    toEuler(q, roll, pitch, yaw);
    
    REQUIRE_THAT(roll, Catch::Matchers::WithinAbs(expectedRoll, 1e-5));
    REQUIRE_THAT(pitch, Catch::Matchers::WithinAbs(expectedPitch, 1e-5));
    REQUIRE_THAT(yaw, Catch::Matchers::WithinAbs(expectedYaw, 1e-5));
}

TEST_CASE("Quaternion - To Euler Identity", "[quaternion][util][conversion]") {
    Quaternion<> q = Quaternion<>::eye();
    
    float roll, pitch, yaw;
    toEuler(q, roll, pitch, yaw);
    
    REQUIRE_THAT(roll, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(pitch, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(yaw, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

// ----------------------------------------------------------------------------
// Check Function Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Is Zero True", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::zero();
    
    REQUIRE(isZero(q));
}

TEST_CASE("Quaternion - Is Zero False", "[quaternion][util][check]") {
    Quaternion<> q(0.01f, 0.0f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isZero(q));
}

TEST_CASE("Quaternion - Is Zero Within Epsilon", "[quaternion][util][check]") {
    Quaternion<> q(1e-10f, 1e-10f, 1e-10f, 1e-10f);
    
    REQUIRE(isZero(q));
}

TEST_CASE("Quaternion - Is Normalized True", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, (float)M_PI / 4);
    
    REQUIRE(isNormalized(q));
}

TEST_CASE("Quaternion - Is Normalized False", "[quaternion][util][check]") {
    Quaternion<> q(2.0f, 0.0f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isNormalized(q));
}

TEST_CASE("Quaternion - Is Normalized After Normalization", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> normalized = normalize(q);
    
    REQUIRE(isNormalized(normalized));
}

TEST_CASE("Quaternion - Is Identity True", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::eye();
    
    REQUIRE(isIdentity(q));
}

TEST_CASE("Quaternion - Is Identity False", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, 0.01f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isIdentity(q));
}

TEST_CASE("Quaternion - Is Identity Within Epsilon", "[quaternion][util][check]") {
    Quaternion<> q(1.0f + 1e-7f, 1e-10f, 1e-10f, 1e-10f);
    
    REQUIRE(isIdentity(q));
}

TEST_CASE("Quaternion - Is Real True", "[quaternion][util][check]") {
    Quaternion<> q(5.0f, 0.0f, 0.0f, 0.0f);
    
    REQUIRE(isReal(q));
}

TEST_CASE("Quaternion - Is Real False", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, 0.01f, 0.0f, 0.0f);
    
    REQUIRE_FALSE(isReal(q));
}

TEST_CASE("Quaternion - Is Real Within Epsilon", "[quaternion][util][check]") {
    Quaternion<> q(2.0f, 1e-10f, 1e-10f, 1e-10f);
    
    REQUIRE(isReal(q));
}

TEST_CASE("Quaternion - Is Pure True", "[quaternion][util][check]") {
    Quaternion<> q(0.0f, 1.0f, 2.0f, 3.0f);
    
    REQUIRE(isPure(q));
}

TEST_CASE("Quaternion - Is Pure False", "[quaternion][util][check]") {
    Quaternion<> q(0.01f, 1.0f, 2.0f, 3.0f);
    
    REQUIRE_FALSE(isPure(q));
}

TEST_CASE("Quaternion - Is Pure Within Epsilon", "[quaternion][util][check]") {
    Quaternion<> q(1e-10f, 1.0f, 2.0f, 3.0f);
    
    REQUIRE(isPure(q));
}

TEST_CASE("Quaternion - Is Finite True", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE(isFinite(q));
}

TEST_CASE("Quaternion - Is Finite False Infinity", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, std::numeric_limits<float>::infinity(), 3.0f, 4.0f);
    
    REQUIRE_FALSE(isFinite(q));
}

TEST_CASE("Quaternion - Is Finite False NaN", "[quaternion][util][check]") {
    Quaternion<> q(1.0f, 2.0f, std::numeric_limits<float>::quiet_NaN(), 4.0f);
    
    REQUIRE_FALSE(isFinite(q));
}

TEST_CASE("Quaternion - Is Same Rotation True", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Quaternion<> p = q;
    
    REQUIRE(isSameRotation(q, p));
}

TEST_CASE("Quaternion - Is Same Rotation True Opposite Signs", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Quaternion<> p = -q;
    
    REQUIRE(isSameRotation(q, p));
}

TEST_CASE("Quaternion - Is Same Rotation False", "[quaternion][util][check]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 4);
    Quaternion<> p = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 3);
    
    REQUIRE_FALSE(isSameRotation(q, p));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Rotate Vector and Convert to Matrix", "[quaternion][integration]") {
    Vector<3> axis = Vector<3>{0.0f, 0.0f, 1.0f};
    float angle = (float)M_PI / 2;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = Vector<3>{1.0f, 0.0f, 0.0f};
    
    // Rotate using quaternion
    Vector<3> rotated1 = rotate(q, v);
    
    // Rotate using matrix
    Matrix<3, 3> R = toMatrix(q);
    Vector<3> rotated2 = R * v;
    
    REQUIRE_THAT(rotated1[0], Catch::Matchers::WithinAbs(rotated2[0], 1e-5));
    REQUIRE_THAT(rotated1[1], Catch::Matchers::WithinAbs(rotated2[1], 1e-5));
    REQUIRE_THAT(rotated1[2], Catch::Matchers::WithinAbs(rotated2[2], 1e-5));
}

TEST_CASE("Quaternion - Slerp Chain Rotation", "[quaternion][integration]") {
    Quaternion<> q0 = Quaternion<>::eye();
    Quaternion<> q1 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI / 2);
    Quaternion<> q2 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, (float)M_PI);
    
    Quaternion<> mid1 = slerp(q0, q1, 0.5f);
    Quaternion<> mid2 = slerp(q1, q2, 0.5f);
    
    // Check that mid1 and mid2 represent proper intermediate rotations
    float angle1 = toAngle(mid1);
    float angle2 = toAngle(mid2);
    
    REQUIRE_THAT(angle1, Catch::Matchers::WithinAbs((float)M_PI / 4, 1e-5));
    REQUIRE_THAT(angle2, Catch::Matchers::WithinAbs(3.0f * (float)M_PI / 4, 1e-5));
}

TEST_CASE("Quaternion - Round Trip Conversions", "[quaternion][integration]") {
    // Original quaternion
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, (float)M_PI / 3);
    
    // Convert to matrix and back
    Matrix<3, 3> R = toMatrix(q);
    Quaternion<> qFromMatrix = Quaternion<>::fromRotationMatrix(R);
    
    // Should represent the same rotation
    REQUIRE(isSameRotation(q, qFromMatrix));
}

TEST_CASE("Quaternion - Swing Twist Reconstruction", "[quaternion][integration]") {
    Vector<3> axis = normalize(Vector<3>{0.0f, 1.0f, 0.0f});
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 2.0f, 0.5f}, 0.8f);
    
    Quaternion<> swing, twist;
    swingTwist(q, axis, swing, twist);
    
    Quaternion<> reconstructed = swing * twist;
    
    REQUIRE(isSameRotation(q, reconstructed));
}

TEST_CASE("Quaternion - Clamp and Normalize Stability", "[quaternion][integration]") {
    Quaternion<> q(10.0f, -15.0f, 20.0f, -5.0f);
    
    Quaternion<> clamped = clamp(q, 5.0f);
    Quaternion<> normalized = normalize(clamped);
    
    REQUIRE(isNormalized(normalized));
    REQUIRE(isFinite(normalized));
}