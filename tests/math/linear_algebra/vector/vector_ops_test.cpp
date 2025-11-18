#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Vector Operations Tests (vector_ops.hpp)
// ============================================================================

TEST_CASE("Vector - Addition Operator", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result = v1 + v2;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Subtraction Operator", "[vector][ops]") {
    Vector<3> v1 = {5.0f, 7.0f, 9.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v1 - v2;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication", "[vector][ops]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v * 3.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Commutative", "[vector][ops]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = 3.0f * v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division", "[vector][ops]") {
    Vector<3> v = {3.0f, 6.0f, 9.0f};
    
    Vector<3> result = v / 3.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Unary Negation", "[vector][ops]") {
    Vector<3> v = {1.0f, -2.0f, 3.0f};
    
    Vector<3> result = -v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Vector - Equality Operator", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    Vector<3> v3 = {1.0f, 2.0f, 3.1f};
    
    REQUIRE(v1 == v2);
    REQUIRE_FALSE(v1 == v3);
}

TEST_CASE("Vector - Inequality Operator", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    Vector<3> v3 = {1.0f, 2.0f, 3.1f};
    
    REQUIRE_FALSE(v1 != v2);
    REQUIRE(v1 != v3);
}

TEST_CASE("Vector - Dot Product", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    float result = dot(v1, v2);
    
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(32.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Orthogonal", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    float result = dot(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Cross Product", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = cross(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Cross Product Anti-Commutative", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result1 = cross(v1, v2);
    Vector<3> result2 = cross(v2, v1);
    
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(-result2[0], 1e-6));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(-result2[1], 1e-6));
    REQUIRE_THAT(result1[2], Catch::Matchers::WithinAbs(-result2[2], 1e-6));
}

TEST_CASE("Vector - Hadamard Product", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result = hadamard(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(18.0, 1e-6));
}

TEST_CASE("Vector - Norm", "[vector][ops]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    float result = norm(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Norm Unit Vector", "[vector][ops]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    float result = norm(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Normalize", "[vector][ops]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.6, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.8, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Normalize Zero Vector", "[vector][ops]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    // Should return zero vector
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Get Distance", "[vector][ops]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {3.0f, 4.0f, 0.0f};
    
    float result = getDistance(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Get Distance Squared", "[vector][ops]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {3.0f, 4.0f, 0.0f};
    
    float result = getDistanceSqr(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(25.0, 1e-6));
}

TEST_CASE("Vector - Get Angle", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    float result = getAngle(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI/2, 1e-6));
}

TEST_CASE("Vector - Get Angle Parallel", "[vector][ops]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {2.0f, 0.0f, 0.0f};
    
    float result = getAngle(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Onto", "[vector][ops]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = projectOnto(v, u);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Onto Zero Vector", "[vector][ops]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {0.0f, 0.0f, 0.0f};
    
    Vector<3> result = projectOnto(v, u);
    
    // Should return zero vector
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}