#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector_util.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Vector Utility Tests (vector_util.hpp)
// ============================================================================

TEST_CASE("Vector - Min Element", "[vector][util]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    float result = min(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Max Element", "[vector][util]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    float result = max(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Abs", "[vector][util]") {
    Vector<3> v = {-1.0f, 2.0f, -3.0f};
    
    Vector<3> result = abs(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Sign", "[vector][util]") {
    Vector<4> v = {-2.0f, 0.0f, 3.0f, -0.5f};
    
    Vector<4> result = sign(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[3], Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Clamp", "[vector][util]") {
    Vector<3> v = {-2.0f, 0.5f, 5.0f};
    
    Vector<3> result = clamp(v, -1.0f, 2.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Floor", "[vector][util]") {
    Vector<3> v = {1.7f, -2.3f, 0.5f};
    
    Vector<3> result = floor(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Ceil", "[vector][util]") {
    Vector<3> v = {1.2f, -2.7f, 0.5f};
    
    Vector<3> result = ceil(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Round", "[vector][util]") {
    Vector<3> v = {1.4f, 2.6f, -3.5f};
    
    Vector<3> result = round(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Vector - Min Elements", "[vector][util]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result = minElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Max Elements", "[vector][util]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result = maxElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Argmin", "[vector][util]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    uint8_t result = argmin(v);
    
    REQUIRE(result == 1);
}

TEST_CASE("Vector - Argmax", "[vector][util]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    uint8_t result = argmax(v);
    
    REQUIRE(result == 2);
}

TEST_CASE("Vector - Project Plane", "[vector][util]") {
    Vector<3> v = {1.0f, 1.0f, 1.0f};
    Vector<3> n = {0.0f, 0.0f, 1.0f};  // Z-axis normal
    
    Vector<3> result = projectPlane(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Ortho (Rejection)", "[vector][util]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = ortho(v, u);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Reflect", "[vector][util]") {
    Vector<3> v = {1.0f, -1.0f, 0.0f};  // 45° downward
    Vector<3> n = {0.0f, 1.0f, 0.0f};   // Up normal
    
    Vector<3> result = reflect(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Sum", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = sum(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Vector - Product", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = product(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(24.0, 1e-6));
}

TEST_CASE("Vector - Mean", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = mean(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(2.5, 1e-6));
}

TEST_CASE("Vector - Variance", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = variance(v);
    
    // Mean = 2.5, variance = ((1-2.5)^2 + (2-2.5)^2 + (3-2.5)^2 + (4-2.5)^2)/4
    // = (2.25 + 0.25 + 0.25 + 2.25)/4 = 5/4 = 1.25
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.25, 1e-6));
}

TEST_CASE("Vector - Standard Deviation", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = stdDev(v);
    
    // Variance = 1.25, stdDev = sqrt(1.25) ≈ 1.118
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.118033988, 1e-6));
}

TEST_CASE("Vector - Linear Interpolation", "[vector][util]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {10.0f, 10.0f, 10.0f};
    
    Vector<3> result = lerp(v1, v2, 0.5f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Linear Interpolation Boundaries", "[vector][util]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result0 = lerp(v1, v2, 0.0f);
    Vector<3> result1 = lerp(v1, v2, 1.0f);
    
    REQUIRE(result0 == v1);
    REQUIRE(result1 == v2);
}

TEST_CASE("Vector - Spherical Interpolation", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = slerp(v1, v2, 0.5f);
    
    // Should be normalized and at 45° angle
    float magnitude = norm(result);
    REQUIRE_THAT(magnitude, Catch::Matchers::WithinAbs(1.0, 1e-6));
    
    // At t=0.5, should be roughly at 45° (equal x and y components)
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(result[1], 1e-5));
}

TEST_CASE("Vector - Spherical Interpolation Boundaries", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result0 = slerp(v1, v2, 0.0f);
    Vector<3> result1 = slerp(v1, v2, 1.0f);
    
    // Should return normalized versions of v1 and v2
    REQUIRE_THAT(result0[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result0[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - To Array Conversion", "[vector][util]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    std::array<float, 3> arr = toArray(v);
    
    REQUIRE_THAT(arr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(arr[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(arr[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - To Skew Symmetric Matrix", "[vector][util]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Matrix<3, 3> skewMat = toSkew(v);
    
    // Diagonal should be zero
    REQUIRE_THAT(skewMat(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(skewMat(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(skewMat(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    
    // Check specific elements
    REQUIRE_THAT(skewMat(0, 1), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(skewMat(0, 2), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(skewMat(1, 2), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    
    // Check antisymmetry
    REQUIRE_THAT(skewMat(1, 0), Catch::Matchers::WithinAbs(-skewMat(0, 1), 1e-6));
    REQUIRE_THAT(skewMat(2, 0), Catch::Matchers::WithinAbs(-skewMat(0, 2), 1e-6));
    REQUIRE_THAT(skewMat(2, 1), Catch::Matchers::WithinAbs(-skewMat(1, 2), 1e-6));
}

TEST_CASE("Vector - Is Normalized True", "[vector][util]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    REQUIRE(isNormalized(v));
}

TEST_CASE("Vector - Is Normalized False", "[vector][util]") {
    Vector<3> v = {2.0f, 0.0f, 0.0f};
    
    REQUIRE_FALSE(isNormalized(v));
}

TEST_CASE("Vector - Is Normalized After Normalization", "[vector][util]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> normalized = normalize(v);
    
    REQUIRE(isNormalized(normalized));
}

TEST_CASE("Vector - Is Zero True", "[vector][util]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    REQUIRE(isZero(v));
}

TEST_CASE("Vector - Is Zero False", "[vector][util]") {
    Vector<3> v = {0.01f, 0.0f, 0.0f};
    
    REQUIRE_FALSE(isZero(v));
}

TEST_CASE("Vector - Is Parallel True Same Direction", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {2.0f, 0.0f, 0.0f};
    
    REQUIRE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Parallel True Opposite Direction", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {-2.0f, 0.0f, 0.0f};
    
    REQUIRE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Parallel False", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    REQUIRE_FALSE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Orthogonal True", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    REQUIRE(isOrthogonal(v1, v2));
}

TEST_CASE("Vector - Is Orthogonal False", "[vector][util]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {1.0f, 1.0f, 0.0f};
    
    REQUIRE_FALSE(isOrthogonal(v1, v2));
}

TEST_CASE("Vector - Is Finite True", "[vector][util]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE(isFinite(v));
}

TEST_CASE("Vector - Is Finite False Infinity", "[vector][util]") {
    Vector<3> v = {1.0f, std::numeric_limits<float>::infinity(), 3.0f};
    
    REQUIRE_FALSE(isFinite(v));
}

TEST_CASE("Vector - Is Finite False NaN", "[vector][util]") {
    Vector<3> v = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f};
    
    REQUIRE_FALSE(isFinite(v));
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("Vector - Complex Expression", "[vector][integration]") {
    Vector<3> a = {1.0f, 0.0f, 0.0f};
    Vector<3> b = {0.0f, 1.0f, 0.0f};
    Vector<3> c = {0.0f, 0.0f, 1.0f};
    
    // Test complex expression: 2a + 3b - c
    Vector<3> result = 2.0f * a + 3.0f * b - c;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Physics Velocity Calculation", "[vector][integration]") {
    Vector<3> position1 = {0.0f, 0.0f, 0.0f};
    Vector<3> position2 = {10.0f, 5.0f, 0.0f};
    float deltaTime = 2.0f;
    
    Vector<3> velocity = (position2 - position1) / deltaTime;
    
    REQUIRE_THAT(velocity[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(velocity[1], Catch::Matchers::WithinAbs(2.5, 1e-6));
    REQUIRE_THAT(velocity[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Gram-Schmidt Orthogonalization", "[vector][integration]") {
    Vector<3> v1 = {1.0f, 1.0f, 0.0f};
    Vector<3> v2 = {1.0f, 0.0f, 1.0f};
    
    // Orthogonalize v2 with respect to v1
    Vector<3> u1 = normalize(v1);
    Vector<3> u2 = normalize(ortho(v2, u1));
    
    // Check they are orthogonal
    REQUIRE(isOrthogonal(u1, u2));
    
    // Check they are normalized
    REQUIRE(isNormalized(u1));
    REQUIRE(isNormalized(u2));
}

TEST_CASE("Vector - Triangle Area from Cross Product", "[vector][integration]") {
    Vector<3> p1 = {0.0f, 0.0f, 0.0f};
    Vector<3> p2 = {1.0f, 0.0f, 0.0f};
    Vector<3> p3 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> edge1 = p2 - p1;
    Vector<3> edge2 = p3 - p1;
    
    Vector<3> crossProd = cross(edge1, edge2);
    float area = norm(crossProd) / 2.0f;
    
    REQUIRE_THAT(area, Catch::Matchers::WithinAbs(0.5, 1e-6));
}

TEST_CASE("Vector - Centroid Calculation", "[vector][integration]") {
    std::array<Vector<3>, 4> points = {
        Vector<3>{0.0f, 0.0f, 0.0f},
        Vector<3>{2.0f, 0.0f, 0.0f},
        Vector<3>{2.0f, 2.0f, 0.0f},
        Vector<3>{0.0f, 2.0f, 0.0f}
    };
    
    Vector<3> centroid = Vector<3>::zero();
    for(const auto& p : points) {
        centroid += p;
    }
    centroid /= 4.0f;
    
    REQUIRE_THAT(centroid[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(centroid[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(centroid[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Barycentric Coordinates", "[vector][integration]") {
    // Triangle vertices
    Vector<3> a = {0.0f, 0.0f, 0.0f};
    Vector<3> b = {1.0f, 0.0f, 0.0f};
    Vector<3> c = {0.0f, 1.0f, 0.0f};
    
    // Point in triangle (center)
    Vector<3> p = {0.25f, 0.25f, 0.0f};
    
    // Calculate barycentric coordinates
    Vector<3> v0 = b - a;
    Vector<3> v1 = c - a;
    Vector<3> v2 = p - a;
    
    float dot00 = dot(v0, v0);
    float dot01 = dot(v0, v1);
    float dot02 = dot(v0, v2);
    float dot11 = dot(v1, v1);
    float dot12 = dot(v1, v2);
    
    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    float w = 1.0f - u - v;
    
    // Verify point is inside triangle
    REQUIRE(u >= 0.0f);
    REQUIRE(v >= 0.0f);
    REQUIRE(w >= 0.0f);
    REQUIRE_THAT(u + v + w, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Distance Optimization Using DistanceSqr", "[vector][integration]") {
    Vector<3> target = {5.0f, 5.0f, 0.0f};
    std::array<Vector<3>, 3> candidates = {
        Vector<3>{1.0f, 1.0f, 0.0f},
        Vector<3>{4.0f, 6.0f, 0.0f},
        Vector<3>{10.0f, 10.0f, 0.0f}
    };
    
    // Find closest point using distanceSqr (cheaper than distance)
    uint8_t closestIdx = 0;
    float minDistSqr = distanceSqr(target, candidates[0]);
    
    for(uint8_t i = 1; i < 3; i++) {
        float dSqr = distanceSqr(target, candidates[i]);
        if(dSqr < minDistSqr) {
            minDistSqr = dSqr;
            closestIdx = i;
        }
    }
    
    REQUIRE(closestIdx == 1);  // candidates[1] is closest
}

TEST_CASE("Vector - Bounding Box Calculation", "[vector][integration]") {
    std::array<Vector<3>, 4> points = {
        Vector<3>{1.0f, 2.0f, 3.0f},
        Vector<3>{-1.0f, 5.0f, 0.0f},
        Vector<3>{3.0f, -2.0f, 4.0f},
        Vector<3>{0.0f, 0.0f, 1.0f}
    };
    
    Vector<3> bboxMin = points[0];
    Vector<3> bboxMax = points[0];
    
    for(const auto& p : points) {
        bboxMin = minElements(bboxMin, p);
        bboxMax = maxElements(bboxMax, p);
    }
    
    REQUIRE_THAT(bboxMin[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(bboxMin[1], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(bboxMin[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    
    REQUIRE_THAT(bboxMax[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(bboxMax[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(bboxMax[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Different Dimensions", "[vector][integration]") {
    Vector<2> v2 = {1.0f, 2.0f};
    Vector<4> v4 = {1.0f, 2.0f, 3.0f, 4.0f};
    Vector<6> v6 = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    
    REQUIRE(v2.size() == 2);
    REQUIRE(v4.size() == 4);
    REQUIRE(v6.size() == 6);
    
    REQUIRE_THAT(norm(v2), Catch::Matchers::WithinAbs(std::sqrt(5.0f), 1e-6));
    REQUIRE_THAT(sum(v4), Catch::Matchers::WithinAbs(10.0, 1e-6));
}