#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector_util.hpp"

#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"


using namespace cobalt::math::linear_algebra;
using index_t = cobalt::math::index_t;

// ============================================================================
// Vector Utility Tests (vector_util.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Clamping Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Clamp Within Range", "[vector][util][clamp]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = clamp(v, 0.0f, 5.0f);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Clamp Above Max", "[vector][util][clamp]") {
    Vector<3> v = {-2.0f, 0.5f, 5.0f};
    
    Vector<3> result = clamp(v, -1.0f, 2.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Clamp Below Min", "[vector][util][clamp]") {
    Vector<3> v = {-5.0f, 2.0f, -1.0f};
    
    Vector<3> result = clamp(v, 0.0f, 5.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Clamp Mixed Values", "[vector][util][clamp]") {
    Vector<4> v = {-2.0f, 3.0f, 8.0f, 1.0f};
    
    Vector<4> result = clamp(v, 0.0f, 5.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[3], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Clamp Symmetric Absolute", "[vector][util][clamp]") {
    Vector<3> v = {-7.0f, 2.0f, 8.0f};
    
    Vector<3> result = clamp(v, 5.0f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-5.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Clamp Zero Vector", "[vector][util][clamp]") {
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = clamp(zero, -1.0f, 1.0f);
    
    REQUIRE(result == zero);
}

// ----------------------------------------------------------------------------
// Element-wise Operation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Min Element", "[vector][util][element]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    float result = min(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Min Element All Positive", "[vector][util][element]") {
    Vector<3> v = {5.0f, 2.0f, 3.0f};
    
    float result = min(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Min Element All Negative", "[vector][util][element]") {
    Vector<3> v = {-5.0f, -2.0f, -3.0f};
    
    float result = min(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(-5.0, 1e-6));
}

TEST_CASE("Vector - Max Element", "[vector][util][element]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    float result = max(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Max Element All Negative", "[vector][util][element]") {
    Vector<3> v = {-5.0f, -2.0f, -3.0f};
    
    float result = max(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Vector - Abs Element-wise", "[vector][util][element]") {
    Vector<3> v = {-1.0f, 2.0f, -3.0f};
    
    Vector<3> result = abs(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Abs All Positive", "[vector][util][element]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = abs(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Abs All Negative", "[vector][util][element]") {
    Vector<3> v = {-1.0f, -2.0f, -3.0f};
    
    Vector<3> result = abs(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Sign Element-wise", "[vector][util][element]") {
    Vector<4> v = {-2.0f, 0.0f, 3.0f, -0.5f};
    
    Vector<4> result = sign(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[3], Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Sign All Positive", "[vector][util][element]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = sign(v);
    
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result[i], Catch::Matchers::WithinAbs(1.0, 1e-6));
    }
}

TEST_CASE("Vector - Floor Element-wise", "[vector][util][element]") {
    Vector<3> v = {1.7f, -2.3f, 0.5f};
    
    Vector<3> result = floor(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Floor All Integers", "[vector][util][element]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = floor(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Ceil Element-wise", "[vector][util][element]") {
    Vector<3> v = {1.2f, -2.7f, 0.5f};
    
    Vector<3> result = ceil(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Ceil All Integers", "[vector][util][element]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = ceil(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Round Element-wise", "[vector][util][element]") {
    Vector<3> v = {1.4f, 2.6f, -3.5f};
    
    Vector<3> result = round(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Vector - Round All Integers", "[vector][util][element]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = round(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Round Half Values", "[vector][util][element]") {
    Vector<4> v = {0.5f, 1.5f, 2.5f, -0.5f};
    
    Vector<4> result = round(v);
    
    // Note: Rounding behavior for 0.5 is implementation-defined
    REQUIRE(std::abs(result[0]) < 1.1f);
    REQUIRE(std::abs(result[3]) < 1.1f);
}

// ----------------------------------------------------------------------------
// Element-wise Min/Max Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Min Elements", "[vector][util][minmax]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result = minElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Min Elements Commutative", "[vector][util][minmax]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result1 = minElements(v1, v2);
    Vector<3> result2 = minElements(v2, v1);
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Min Elements With Negatives", "[vector][util][minmax]") {
    Vector<3> v1 = {-1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, -3.0f, 4.0f};
    
    Vector<3> result = minElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Max Elements", "[vector][util][minmax]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result = maxElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Max Elements Commutative", "[vector][util][minmax]") {
    Vector<3> v1 = {1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, 3.0f, 4.0f};
    
    Vector<3> result1 = maxElements(v1, v2);
    Vector<3> result2 = maxElements(v2, v1);
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Max Elements With Negatives", "[vector][util][minmax]") {
    Vector<3> v1 = {-1.0f, 5.0f, 3.0f};
    Vector<3> v2 = {2.0f, -3.0f, 4.0f};
    
    Vector<3> result = maxElements(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Argmin/Argmax Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Argmin", "[vector][util][argmin]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    index_t result = argmin(v);
    
    REQUIRE(result == 1);
}

TEST_CASE("Vector - Argmin First Element", "[vector][util][argmin]") {
    Vector<3> v = {-5.0f, 2.0f, 3.0f};
    
    index_t result = argmin(v);
    
    REQUIRE(result == 0);
}

TEST_CASE("Vector - Argmin Last Element", "[vector][util][argmin]") {
    Vector<4> v = {3.0f, 2.0f, 5.0f, -10.0f};
    
    index_t result = argmin(v);
    
    REQUIRE(result == 3);
}

TEST_CASE("Vector - Argmax", "[vector][util][argmax]") {
    Vector<4> v = {3.0f, -1.0f, 5.0f, 2.0f};
    
    index_t result = argmax(v);
    
    REQUIRE(result == 2);
}

TEST_CASE("Vector - Argmax First Element", "[vector][util][argmax]") {
    Vector<3> v = {10.0f, 2.0f, 3.0f};
    
    index_t result = argmax(v);
    
    REQUIRE(result == 0);
}

TEST_CASE("Vector - Argmax Last Element", "[vector][util][argmax]") {
    Vector<4> v = {3.0f, 2.0f, 5.0f, 15.0f};
    
    index_t result = argmax(v);
    
    REQUIRE(result == 3);
}

// ----------------------------------------------------------------------------
// Projection Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Project Plane", "[vector][util][project]") {
    Vector<3> v = {1.0f, 1.0f, 1.0f};
    Vector<3> n = {0.0f, 0.0f, 1.0f};
    
    Vector<3> result = projectPlane(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Plane Already In Plane", "[vector][util][project]") {
    Vector<3> v = {1.0f, 2.0f, 0.0f};
    Vector<3> n = {0.0f, 0.0f, 1.0f};
    
    Vector<3> result = projectPlane(v, n);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Project Plane Perpendicular", "[vector][util][project]") {
    Vector<3> v = {0.0f, 0.0f, 5.0f};
    Vector<3> n = {0.0f, 0.0f, 1.0f};
    
    Vector<3> result = projectPlane(v, n);
    
    REQUIRE(result == Vector<3>::zero());
}

TEST_CASE("Vector - Ortho Rejection", "[vector][util][project]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = ortho(v, u);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Ortho Already Orthogonal", "[vector][util][project]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> u = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = ortho(v, u);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Ortho Parallel Vectors", "[vector][util][project]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    Vector<3> u = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = ortho(v, u);
    
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result[i], Catch::Matchers::WithinAbs(0.0, 1e-5));
    }
}

TEST_CASE("Vector - Reflect", "[vector][util][reflect]") {
    Vector<3> v = {1.0f, -1.0f, 0.0f};
    Vector<3> n = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = reflect(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Reflect Perpendicular", "[vector][util][reflect]") {
    Vector<3> v = {0.0f, -1.0f, 0.0f};
    Vector<3> n = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = reflect(v, n);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Reflect Parallel", "[vector][util][reflect]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> n = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = reflect(v, n);
    
    REQUIRE(result == v);
}

// ----------------------------------------------------------------------------
// Statistical Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Sum", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = sum(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Vector - Sum With Negatives", "[vector][util][stats]") {
    Vector<3> v = {1.0f, -2.0f, 3.0f};
    
    float result = sum(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Sum Zero Vector", "[vector][util][stats]") {
    Vector<3> zero = Vector<3>::zero();
    
    float result = sum(zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Product", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = product(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(24.0, 1e-6));
}

TEST_CASE("Vector - Product With Zero", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 0.0f, 3.0f, 4.0f};
    
    float result = product(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Product With Negatives", "[vector][util][stats]") {
    Vector<3> v = {-2.0f, 3.0f, -4.0f};
    
    float result = product(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(24.0, 1e-6));
}

TEST_CASE("Vector - Mean", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = mean(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(2.5, 1e-6));
}

TEST_CASE("Vector - Mean All Same", "[vector][util][stats]") {
    Vector<3> v = {5.0f, 5.0f, 5.0f};
    
    float result = mean(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Mean With Negatives", "[vector][util][stats]") {
    Vector<3> v = {-1.0f, 0.0f, 1.0f};
    
    float result = mean(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Variance", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = variance(v);
    
    // Mean = 2.5, variance = ((1-2.5)^2 + (2-2.5)^2 + (3-2.5)^2 + (4-2.5)^2)/4 = 1.25
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.25, 1e-6));
}

TEST_CASE("Vector - Variance All Same", "[vector][util][stats]") {
    Vector<3> v = {5.0f, 5.0f, 5.0f};
    
    float result = variance(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Standard Deviation", "[vector][util][stats]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float result = stdDev(v);
    
    // Variance = 1.25, stdDev = sqrt(1.25) ≈ 1.118
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.118033988, 1e-6));
}

TEST_CASE("Vector - Standard Deviation All Same", "[vector][util][stats]") {
    Vector<3> v = {5.0f, 5.0f, 5.0f};
    
    float result = stdDev(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Covariance", "[vector][util][stats]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {2.0f, 4.0f, 6.0f};
    
    float result = covariance(v, u);
    
    // Perfect positive correlation
    REQUIRE(result > 0.0f);
}

TEST_CASE("Vector - Covariance Uncorrelated", "[vector][util][stats]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {3.0f, 2.0f, 1.0f};
    
    float result = covariance(v, u);
    
    // Negative correlation
    REQUIRE(result < 0.0f);
}

TEST_CASE("Vector - Clean Zero", "[vector][util][stats]") {
    Vector<3> v = {1e-10f, 2.0f, -1e-10f};
    
    Vector<3> result = cleanZero(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Clean Zero Preserves Significant", "[vector][util][stats]") {
    Vector<3> v = {0.001f, 2.0f, 0.0001f};
    
    Vector<3> result = cleanZero(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.001, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0001, 1e-6));
}

// ----------------------------------------------------------------------------
// Interpolation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Linear Interpolation", "[vector][util][interp]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {10.0f, 10.0f, 10.0f};
    
    Vector<3> result = lerp(v1, v2, 0.5f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Linear Interpolation Boundaries", "[vector][util][interp]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result0 = lerp(v1, v2, 0.0f);
    Vector<3> result1 = lerp(v1, v2, 1.0f);
    
    REQUIRE(result0 == v1);
    REQUIRE(result1 == v2);
}

TEST_CASE("Vector - Linear Interpolation Beyond Range", "[vector][util][interp]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {10.0f, 10.0f, 10.0f};
    
    Vector<3> result = lerp(v1, v2, 1.5f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(15.0, 1e-6));
}

TEST_CASE("Vector - Linear Interpolation Negative t", "[vector][util][interp]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {10.0f, 10.0f, 10.0f};
    
    Vector<3> result = lerp(v1, v2, -0.5f);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-5.0, 1e-6));
}

TEST_CASE("Vector - Spherical Interpolation", "[vector][util][interp]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = slerp(v1, v2, 0.5f);
    
    float magnitude = norm(result);
    REQUIRE_THAT(magnitude, Catch::Matchers::WithinAbs(1.0, 1e-6));
    
    // At t=0.5, should be roughly at 45°
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(result[1], 1e-5));
}

TEST_CASE("Vector - Spherical Interpolation Boundaries", "[vector][util][interp]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result0 = slerp(v1, v2, 0.0f);
    Vector<3> result1 = slerp(v1, v2, 1.0f);
    
    REQUIRE_THAT(result0[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result0[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Spherical Interpolation Maintains Unit Length", "[vector][util][interp]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 0.0f, 1.0f};
    
    for(float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Vector<3> result = slerp(v1, v2, t);
        float magnitude = norm(result);
        REQUIRE_THAT(magnitude, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

// ----------------------------------------------------------------------------
// Conversion Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - To Array Conversion", "[vector][util][conversion]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    std::array<float, 3> arr = toArray(v);
    
    REQUIRE_THAT(arr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(arr[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(arr[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - To Array Various Sizes", "[vector][util][conversion]") {
    Vector<2> v2 = {1.0f, 2.0f};
    Vector<5> v5 = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    
    std::array<float, 2> arr2 = toArray(v2);
    std::array<float, 5> arr5 = toArray(v5);
    
    REQUIRE(arr2.size() == 2);
    REQUIRE(arr5.size() == 5);
    REQUIRE_THAT(arr5[4], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - To Skew Symmetric Matrix", "[vector][util][conversion]") {
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
}

TEST_CASE("Vector - To Skew Antisymmetry", "[vector][util][conversion]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Matrix<3, 3> skewMat = toSkew(v);
    
    // Check antisymmetry
    REQUIRE_THAT(skewMat(1, 0), Catch::Matchers::WithinAbs(-skewMat(0, 1), 1e-6));
    REQUIRE_THAT(skewMat(2, 0), Catch::Matchers::WithinAbs(-skewMat(0, 2), 1e-6));
    REQUIRE_THAT(skewMat(2, 1), Catch::Matchers::WithinAbs(-skewMat(1, 2), 1e-6));
}

TEST_CASE("Vector - To Skew Cross Product Property", "[vector][util][conversion]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {4.0f, 5.0f, 6.0f};
    
    Matrix<3, 3> skewMat = toSkew(v);
    Vector<3> result1 = skewMat * u;
    Vector<3> result2 = cross(v, u);
    
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result1[i], Catch::Matchers::WithinAbs(result2[i], 1e-5));
    }
}

// ----------------------------------------------------------------------------
// Check Function Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Is Normalized True", "[vector][util][check]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    REQUIRE(isNormalized(v));
}

TEST_CASE("Vector - Is Normalized False", "[vector][util][check]") {
    Vector<3> v = {2.0f, 0.0f, 0.0f};
    
    REQUIRE_FALSE(isNormalized(v));
}

TEST_CASE("Vector - Is Normalized After Normalization", "[vector][util][check]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> normalized = normalize(v);
    
    REQUIRE(isNormalized(normalized));
}

TEST_CASE("Vector - Is Normalized Within Epsilon", "[vector][util][check]") {
    Vector<3> v = {1.0f + 1e-7f, 0.0f, 0.0f};
    
    REQUIRE(isNormalized(v));
}

TEST_CASE("Vector - Is Zero True", "[vector][util][check]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    REQUIRE(isZero(v));
}

TEST_CASE("Vector - Is Zero False", "[vector][util][check]") {
    Vector<3> v = {0.01f, 0.0f, 0.0f};
    
    REQUIRE_FALSE(isZero(v));
}

TEST_CASE("Vector - Is Zero Within Epsilon", "[vector][util][check]") {
    Vector<3> v = {1e-10f, 1e-10f, 1e-10f};
    
    REQUIRE(isZero(v));
}

TEST_CASE("Vector - Is Parallel True Same Direction", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {2.0f, 0.0f, 0.0f};
    
    REQUIRE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Parallel True Opposite Direction", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {-2.0f, 0.0f, 0.0f};
    
    REQUIRE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Parallel False", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    REQUIRE_FALSE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Parallel Scaled Vectors", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {2.0f, 4.0f, 6.0f};
    
    REQUIRE(isParallel(v1, v2));
}

TEST_CASE("Vector - Is Orthogonal True", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    REQUIRE(isOrthogonal(v1, v2));
}

TEST_CASE("Vector - Is Orthogonal False", "[vector][util][check]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {1.0f, 1.0f, 0.0f};
    
    REQUIRE_FALSE(isOrthogonal(v1, v2));
}

TEST_CASE("Vector - Is Orthogonal Unit Vectors", "[vector][util][check]") {
    Vector<3> vx = Vector<3>::unitX();
    Vector<3> vy = Vector<3>::unitY();
    Vector<3> vz = Vector<3>::unitZ();
    
    REQUIRE(isOrthogonal(vx, vy));
    REQUIRE(isOrthogonal(vx, vz));
    REQUIRE(isOrthogonal(vy, vz));
}

TEST_CASE("Vector - Is Finite True", "[vector][util][check]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE(isFinite(v));
}

TEST_CASE("Vector - Is Finite False Infinity", "[vector][util][check]") {
    Vector<3> v = {1.0f, std::numeric_limits<float>::infinity(), 3.0f};
    
    REQUIRE_FALSE(isFinite(v));
}

TEST_CASE("Vector - Is Finite False NaN", "[vector][util][check]") {
    Vector<3> v = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f};
    
    REQUIRE_FALSE(isFinite(v));
}

TEST_CASE("Vector - Is Finite Negative Infinity", "[vector][util][check]") {
    Vector<3> v = {-std::numeric_limits<float>::infinity(), 2.0f, 3.0f};
    
    REQUIRE_FALSE(isFinite(v));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Complex Expression", "[vector][integration]") {
    Vector<3> a = {1.0f, 0.0f, 0.0f};
    Vector<3> b = {0.0f, 1.0f, 0.0f};
    Vector<3> c = {0.0f, 0.0f, 1.0f};
    
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
    
    Vector<3> u1 = normalize(v1);
    Vector<3> u2 = normalize(ortho(v2, u1));
    
    REQUIRE(isOrthogonal(u1, u2));
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