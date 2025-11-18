#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_util.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Vector Utility Tests (vector_util.hpp)
// ============================================================================

TEST_CASE("Vector - Clamp", "[vector][util]") {
    Vector<3> v = {-2.0f, 0.5f, 5.0f};
    
    Vector<3> result = clamp(v, -1.0f, 2.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Sign", "[vector][util]") {
    Vector<4> v = {-2.0f, 0.0f, 3.0f, -0.5f};
    
    Vector<4> result = sign(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[3], Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Abs", "[vector][util]") {
    Vector<3> v = {-1.0f, 2.0f, -3.0f};
    
    Vector<3> result = abs(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

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

TEST_CASE("Vector - Project Onto Plane", "[vector][util]") {
    Vector<3> v = {1.0f, 1.0f, 1.0f};
    Vector<3> n = {0.0f, 0.0f, 1.0f};  // Z-axis normal
    
    Vector<3> result = projectOntoPlane(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Reject From", "[vector][util]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = rejectFrom(v, u);
    
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

TEST_CASE("Vector - Sum Elements", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = sumElements(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Vector - Product Elements", "[vector][util]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = productElements(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(24.0, 1e-6));
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

TEST_CASE("Vector - Skew Symmetric Matrix", "[vector][util]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Matrix<3, 3> skewMat = skew(v);
    
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