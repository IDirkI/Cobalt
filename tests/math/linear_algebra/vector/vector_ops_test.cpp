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

// ----------------------------------------------------------------------------
// Binary Arithmetic Operators
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Addition Operator", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result = v1 + v2;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Addition Preserves Operands", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v1_copy = v1;
    Vector<3> v2_copy = v2;
    
    Vector<3> result = v1 + v2;
    
    REQUIRE(v1 == v1_copy);
    REQUIRE(v2 == v2_copy);
}

TEST_CASE("Vector - Addition With Zero", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = v + zero;
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Addition Commutative", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result1 = v1 + v2;
    Vector<3> result2 = v2 + v1;
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Addition Associative", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v3 = {7.0f, 8.0f, 9.0f};
    
    Vector<3> result1 = (v1 + v2) + v3;
    Vector<3> result2 = v1 + (v2 + v3);
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Subtraction Operator", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {5.0f, 7.0f, 9.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v1 - v2;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Subtraction Self Zeros", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v - v;
    
    REQUIRE(result == Vector<3>::zero());
}

TEST_CASE("Vector - Subtraction Not Commutative", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result1 = v1 - v2;
    Vector<3> result2 = v2 - v1;
    
    REQUIRE(result1 != result2);
    REQUIRE(result1 == -result2);
}

TEST_CASE("Vector - Subtraction With Zero", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = v - zero;
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Scalar Multiplication Right", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v * 2.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Left", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = 3.0f * v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Commutative", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result1 = v * 5.0f;
    Vector<3> result2 = 5.0f * v;
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Scalar Multiplication By Zero", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v * 0.0f;
    
    REQUIRE(result == Vector<3>::zero());
}

TEST_CASE("Vector - Scalar Multiplication By One", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v * 1.0f;
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Scalar Multiplication Negative", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v * -2.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-4.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Fractional", "[vector][ops][arithmetic]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    
    Vector<3> result = v * 0.5f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division", "[vector][ops][arithmetic]") {
    Vector<3> v = {3.0f, 6.0f, 9.0f};
    
    Vector<3> result = v / 3.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division By One", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v / 1.0f;
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Scalar Division Fractional", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = v / 0.5f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Negative", "[vector][ops][arithmetic]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    
    Vector<3> result = v / -2.0f;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Vector - Unary Negation", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, -2.0f, 3.0f};
    
    Vector<3> result = -v;
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Vector - Double Negation", "[vector][ops][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = -(-v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Negation Of Zero", "[vector][ops][arithmetic]") {
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = -zero;
    
    REQUIRE(result == zero);
}

TEST_CASE("Vector - Distributive Property Scalar", "[vector][ops][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float c = 2.0f;
    
    Vector<3> result1 = c * (v1 + v2);
    Vector<3> result2 = c * v1 + c * v2;
    
    REQUIRE(result1 == result2);
}

// ----------------------------------------------------------------------------
// Comparison Operators
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Equality Operator Same Values", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    
    REQUIRE(v1 == v2);
}

TEST_CASE("Vector - Equality Operator Different Values", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.1f};
    
    REQUIRE_FALSE(v1 == v2);
}

TEST_CASE("Vector - Equality Operator Within Epsilon", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f + 1e-7f, 2.0f, 3.0f};
    
    REQUIRE(v1 == v2);
}

TEST_CASE("Vector - Equality Reflexive", "[vector][ops][comparison]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE(v == v);
}

TEST_CASE("Vector - Equality Symmetric", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = v1;
    
    REQUIRE(v1 == v2);
    REQUIRE(v2 == v1);
}

TEST_CASE("Vector - Equality Zero Vectors", "[vector][ops][comparison]") {
    Vector<3> zero1 = Vector<3>::zero();
    Vector<3> zero2 = Vector<3>::zero();
    
    REQUIRE(zero1 == zero2);
}

TEST_CASE("Vector - Inequality Operator", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.1f};
    
    REQUIRE(v1 != v2);
}

TEST_CASE("Vector - Inequality Operator Same Values", "[vector][ops][comparison]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = v1;
    
    REQUIRE_FALSE(v1 != v2);
}

TEST_CASE("Vector - Inequality Different Dimensions Always Different", "[vector][ops][comparison]") {
    Vector<2> v2 = {1.0f, 2.0f};
    Vector<3> v3 = {1.0f, 2.0f, 0.0f};
    
    // These are different types, so can't be compared directly
    // This test just verifies the concept
    REQUIRE(v2.size() != v3.size());
}

// ----------------------------------------------------------------------------
// Dot Product Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Dot Product Basic", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    float result = dot(v1, v2);
    
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(32.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Orthogonal", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    float result = dot(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Commutative", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    float result1 = dot(v1, v2);
    float result2 = dot(v2, v1);
    
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-6));
}

TEST_CASE("Vector - Dot Product With Zero", "[vector][ops][dot]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    
    float result = dot(v, zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Dot Product With Self", "[vector][ops][dot]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    float result = dot(v, v);
    
    // Should equal squared norm
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(25.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Unit Vectors", "[vector][ops][dot]") {
    Vector<3> ux = Vector<3>::unitX();
    Vector<3> uy = Vector<3>::unitY();
    
    REQUIRE_THAT(dot(ux, ux), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(dot(ux, uy), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Parallel Same Direction", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {2.0f, 4.0f, 6.0f};
    
    float result = dot(v1, v2);
    
    // Parallel vectors: dot product = |v1| * |v2|
    float expected = norm(v1) * norm(v2);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(expected, 1e-5));
}

TEST_CASE("Vector - Dot Product Parallel Opposite Direction", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {-2.0f, -4.0f, -6.0f};
    
    float result = dot(v1, v2);
    
    // Opposite parallel vectors: dot product = -|v1| * |v2|
    float expected = -norm(v1) * norm(v2);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(expected, 1e-5));
}

TEST_CASE("Vector - Dot Product Different Dimensions", "[vector][ops][dot]") {
    Vector<2> v2 = {1.0f, 2.0f};
    Vector<4> v4 = {1.0f, 2.0f, 3.0f, 4.0f};
    
    REQUIRE_THAT(dot(v2, v2), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(dot(v4, v4), Catch::Matchers::WithinAbs(30.0, 1e-6));
}

TEST_CASE("Vector - Dot Product Bilinearity", "[vector][ops][dot]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v3 = {7.0f, 8.0f, 9.0f};
    float c = 2.0f;
    
    // dot(c*v1, v2) = c*dot(v1, v2)
    float result1 = dot(c * v1, v2);
    float result2 = c * dot(v1, v2);
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-5));
    
    // dot(v1 + v2, v3) = dot(v1, v3) + dot(v2, v3)
    float result3 = dot(v1 + v2, v3);
    float result4 = dot(v1, v3) + dot(v2, v3);
    REQUIRE_THAT(result3, Catch::Matchers::WithinAbs(result4, 1e-5));
}

// ----------------------------------------------------------------------------
// Cross Product Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Cross Product Basic", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = cross(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Cross Product Anti-Commutative", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result1 = cross(v1, v2);
    Vector<3> result2 = cross(v2, v1);
    
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(-result2[0], 1e-6));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(-result2[1], 1e-6));
    REQUIRE_THAT(result1[2], Catch::Matchers::WithinAbs(-result2[2], 1e-6));
}

TEST_CASE("Vector - Cross Product Parallel Vectors", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {2.0f, 4.0f, 6.0f};
    
    Vector<3> result = cross(v1, v2);
    
    // Parallel vectors have zero cross product
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Vector - Cross Product Self Is Zero", "[vector][ops][cross]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = cross(v, v);
    
    REQUIRE(result == Vector<3>::zero());
}

TEST_CASE("Vector - Cross Product Orthogonal To Inputs", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = cross(v1, v2);
    
    // Result should be orthogonal to both inputs
    REQUIRE_THAT(dot(result, v1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(dot(result, v2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Cross Product Unit Vectors", "[vector][ops][cross]") {
    Vector<3> ux = Vector<3>::unitX();
    Vector<3> uy = Vector<3>::unitY();
    Vector<3> uz = Vector<3>::unitZ();
    
    // Right-hand rule: x cross y = z
    Vector<3> result_xy = cross(ux, uy);
    REQUIRE_THAT(result_xy[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result_xy[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result_xy[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
    
    // y cross z = x
    Vector<3> result_yz = cross(uy, uz);
    REQUIRE_THAT(result_yz[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result_yz[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result_yz[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    
    // z cross x = y
    Vector<3> result_zx = cross(uz, ux);
    REQUIRE_THAT(result_zx[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result_zx[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result_zx[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Cross Product With Zero", "[vector][ops][cross]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = cross(v, zero);
    
    REQUIRE(result == zero);
}

TEST_CASE("Vector - Cross Product Magnitude", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    
    Vector<3> result = cross(v1, v2);
    
    // |v1 × v2| = |v1| * |v2| * sin(θ)
    // For orthogonal unit vectors, this is 1
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Cross Product Jacobi Identity", "[vector][ops][cross]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v3 = {7.0f, 8.0f, 9.0f};
    
    // v1 × (v2 × v3) + v2 × (v3 × v1) + v3 × (v1 × v2) = 0
    Vector<3> term1 = cross(v1, cross(v2, v3));
    Vector<3> term2 = cross(v2, cross(v3, v1));
    Vector<3> term3 = cross(v3, cross(v1, v2));
    
    Vector<3> result = term1 + term2 + term3;
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(0.0, 1e-4));
}

// ----------------------------------------------------------------------------
// Hadamard Product Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Hadamard Product Basic", "[vector][ops][hadamard]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result = hadamard(v1, v2);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(18.0, 1e-6));
}

TEST_CASE("Vector - Hadamard Product Commutative", "[vector][ops][hadamard]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    Vector<3> result1 = hadamard(v1, v2);
    Vector<3> result2 = hadamard(v2, v1);
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Hadamard Product With Ones", "[vector][ops][hadamard]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> ones = {1.0f, 1.0f, 1.0f};
    
    Vector<3> result = hadamard(v, ones);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Hadamard Product With Zero", "[vector][ops][hadamard]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    
    Vector<3> result = hadamard(v, zero);
    
    REQUIRE(result == zero);
}

TEST_CASE("Vector - Hadamard Product Associative", "[vector][ops][hadamard]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v3 = {7.0f, 8.0f, 9.0f};
    
    Vector<3> result1 = hadamard(hadamard(v1, v2), v3);
    Vector<3> result2 = hadamard(v1, hadamard(v2, v3));
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Vector - Hadamard Product Distributive", "[vector][ops][hadamard]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    Vector<3> v3 = {7.0f, 8.0f, 9.0f};
    
    Vector<3> result1 = hadamard(v1, v2 + v3);
    Vector<3> result2 = hadamard(v1, v2) + hadamard(v1, v3);
    
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result1[i], Catch::Matchers::WithinAbs(result2[i], 1e-5));
    }
}

// ----------------------------------------------------------------------------
// Triple Product Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Triple Product Basic", "[vector][ops][triple]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> u = {0.0f, 1.0f, 0.0f};
    Vector<3> w = {0.0f, 0.0f, 1.0f};
    
    float result = tripleProduct(v, u, w);
    
    // Volume of unit cube = 1
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Triple Product Coplanar Vectors", "[vector][ops][triple]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> u = {0.0f, 1.0f, 0.0f};
    Vector<3> w = {1.0f, 1.0f, 0.0f};
    
    float result = tripleProduct(v, u, w);
    
    // Coplanar vectors have zero volume
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Triple Product Cyclic Permutation", "[vector][ops][triple]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {4.0f, 5.0f, 6.0f};
    Vector<3> w = {7.0f, 8.0f, 9.0f};
    
    float result1 = tripleProduct(v, u, w);
    float result2 = tripleProduct(u, w, v);
    float result3 = tripleProduct(w, v, u);
    
    // Cyclic permutations should give same result
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-5));
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result3, 1e-5));
}

TEST_CASE("Vector - Triple Product Sign Change", "[vector][ops][triple]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {4.0f, 5.0f, 6.0f};
    Vector<3> w = {7.0f, 8.0f, 9.0f};
    
    float result1 = tripleProduct(v, u, w);
    float result2 = tripleProduct(u, v, w);
    
    // Swapping two vectors changes sign
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(-result2, 1e-5));
}

TEST_CASE("Vector - Triple Product With Zero", "[vector][ops][triple]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {4.0f, 5.0f, 6.0f};
    Vector<3> zero = Vector<3>::zero();
    
    float result = tripleProduct(v, u, zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Triple Product Parallelepiped Volume", "[vector][ops][triple]") {
    Vector<3> v = {2.0f, 0.0f, 0.0f};
    Vector<3> u = {0.0f, 3.0f, 0.0f};
    Vector<3> w = {0.0f, 0.0f, 4.0f};
    
    float result = std::abs(tripleProduct(v, u, w));
    
    // Volume = 2 * 3 * 4 = 24
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(24.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Norm Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Norm Basic", "[vector][ops][norm]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    float result = norm(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Norm Unit Vector", "[vector][ops][norm]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    float result = norm(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Norm Zero Vector", "[vector][ops][norm]") {
    Vector<3> zero = Vector<3>::zero();
    
    float result = norm(zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Norm Positive Definite", "[vector][ops][norm]") {
    Vector<3> v = {-1.0f, -2.0f, -3.0f};
    
    float result = norm(v);
    
    REQUIRE(result > 0.0f);
}

TEST_CASE("Vector - Norm 3D Pythagorean", "[vector][ops][norm]") {
    Vector<3> v = {1.0f, 2.0f, 2.0f};
    
    float result = norm(v);
    
    // sqrt(1 + 4 + 4) = sqrt(9) = 3
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Norm Different Dimensions", "[vector][ops][norm]") {
    Vector<2> v2 = {3.0f, 4.0f};
    Vector<4> v4 = {1.0f, 2.0f, 2.0f, 4.0f};
    
    REQUIRE_THAT(norm(v2), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(norm(v4), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Norm Triangle Inequality", "[vector][ops][norm]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    float norm_sum = norm(v1 + v2);
    float sum_norms = norm(v1) + norm(v2);
    
    // ||v1 + v2|| <= ||v1|| + ||v2||
    REQUIRE(norm_sum <= sum_norms + 1e-5);
}

TEST_CASE("Vector - Norm Squared", "[vector][ops][norm]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    float result = normSqr(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(25.0, 1e-6));
}

TEST_CASE("Vector - Norm Squared More Efficient", "[vector][ops][norm]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    float normSquared = normSqr(v);
    float normThenSquared = norm(v) * norm(v);
    
    REQUIRE_THAT(normSquared, Catch::Matchers::WithinAbs(normThenSquared, 1e-5));
}

TEST_CASE("Vector - Norm Squared Zero Vector", "[vector][ops][norm]") {
    Vector<3> zero = Vector<3>::zero();
    
    float result = normSqr(zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Norm Homogeneity", "[vector][ops][norm]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    float c = 2.5f;
    
    float result = norm(c * v);
    float expected = std::abs(c) * norm(v);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(expected, 1e-5));
}

// ----------------------------------------------------------------------------
// Normalize Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Normalize Basic", "[vector][ops][normalize]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.6, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.8, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Normalize Zero Vector", "[vector][ops][normalize]") {
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    // Should return zero vector
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Normalize Already Normalized", "[vector][ops][normalize]") {
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    REQUIRE(result == v);
}

TEST_CASE("Vector - Normalize Result Unit Length", "[vector][ops][normalize]") {
    Vector<3> v = {5.0f, 12.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Normalize Direction Preserved", "[vector][ops][normalize]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    // Result should be parallel to original
    float scale = norm(v);
    REQUIRE_THAT(result[0] * scale, Catch::Matchers::WithinAbs(v[0], 1e-5));
    REQUIRE_THAT(result[1] * scale, Catch::Matchers::WithinAbs(v[1], 1e-5));
    REQUIRE_THAT(result[2] * scale, Catch::Matchers::WithinAbs(v[2], 1e-5));
}

TEST_CASE("Vector - Normalize Negative Vector", "[vector][ops][normalize]") {
    Vector<3> v = {-3.0f, -4.0f, 0.0f};
    
    Vector<3> result = normalize(v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-0.6, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(-0.8, 1e-6));
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Normalize Very Small Vector", "[vector][ops][normalize]") {
    Vector<3> v1 = {1e-8f, 1e-8f, 1e-8f};
    Vector<3> v2 = {1e-4f, 1e-4f, 1e-4f};
    
    Vector<3> result1 = normalize(v1);
    Vector<3> result2 = normalize(v2);
    
    // Should still produce valid unit vector
    REQUIRE_THAT(norm(result1), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(norm(result2), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

// ----------------------------------------------------------------------------
// Distance Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Distance Basic", "[vector][ops][distance]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {3.0f, 4.0f, 0.0f};
    
    float result = distance(v1, v2);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Distance Same Point", "[vector][ops][distance]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
float result = distance(v, v);

REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Distance Symmetric", "[vector][ops][distance]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float result1 = distance(v1, v2);
    float result2 = distance(v2, v1);

    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-6));
}

TEST_CASE("Vector - Distance Positive", "[vector][ops][distance]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float result = distance(v1, v2);

    REQUIRE(result >= 0.0f);
}

TEST_CASE("Vector - Distance Triangle Inequality", "[vector][ops][distance]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {1.0f, 1.0f, 0.0f};
    Vector<3> v3 = {2.0f, 0.0f, 0.0f};
    float d12 = distance(v1, v2);
    float d23 = distance(v2, v3);
    float d13 = distance(v1, v3);

    // d(v1, v3) <= d(v1, v2) + d(v2, v3)
    REQUIRE(d13 <= d12 + d23 + 1e-5);
}

TEST_CASE("Vector - Distance From Origin", "[vector][ops][distance]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> origin = Vector<3>::zero();
    float result = distance(origin, v);

    // Distance from origin equals norm
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(norm(v), 1e-6));
}

TEST_CASE("Vector - Distance Squared Basic", "[vector][ops][distance]") {
    Vector<3> v1 = {0.0f, 0.0f, 0.0f};
    Vector<3> v2 = {3.0f, 4.0f, 0.0f};
    float result = distanceSqr(v1, v2);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(25.0, 1e-6));
}

TEST_CASE("Vector - Distance Squared Same Point", "[vector][ops][distance]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    float result = distanceSqr(v, v);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}
TEST_CASE("Vector - Distance Squared More Efficient", "[vector][ops][distance]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float distSquared = distanceSqr(v1, v2);
    float distThenSquared = distance(v1, v2) * distance(v1, v2);

    REQUIRE_THAT(distSquared, Catch::Matchers::WithinAbs(distThenSquared, 1e-5));
}

TEST_CASE("Vector - Distance Squared Symmetric", "[vector][ops][distance]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float result1 = distanceSqr(v1, v2);
    float result2 = distanceSqr(v2, v1);

    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-6));
}

// ----------------------------------------------------------------------------
// Angle Tests
// ----------------------------------------------------------------------------
TEST_CASE("Vector - Angle Between Orthogonal Vectors", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    float result = angle(v1, v2);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI/2, 1e-6));
}

TEST_CASE("Vector - Angle Between Parallel Same Direction", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {2.0f, 0.0f, 0.0f};
    float result = angle(v1, v2);

REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Angle Between Parallel Opposite Direction", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {-1.0f, 0.0f, 0.0f};
    float result = angle(v1, v2);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI, 1e-6));
}

TEST_CASE("Vector - Angle Symmetric", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float result1 = angle(v1, v2);
    float result2 = angle(v2, v1);

    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-6));
}

TEST_CASE("Vector - Angle Between Same Vector", "[vector][ops][angle]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    float result = angle(v, v);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Angle With Zero Vector", "[vector][ops][angle]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> zero = Vector<3>::zero();
    float result = angle(v, zero);

    // Angle with zero vector is undefined, returns 0
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Angle 45 Degrees", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {1.0f, 1.0f, 0.0f};
    float result = angle(v1, v2);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI/4, 1e-6));
}

TEST_CASE("Vector - Angle Between Unit Vectors", "[vector][ops][angle]") {
    Vector<3> ux = Vector<3>::unitX();
    Vector<3> uy = Vector<3>::unitY();
    Vector<3> uz = Vector<3>::unitZ();
    REQUIRE_THAT(angle(ux, uy), Catch::Matchers::WithinAbs(M_PI/2, 1e-6));
    REQUIRE_THAT(angle(uy, uz), Catch::Matchers::WithinAbs(M_PI/2, 1e-6));
    REQUIRE_THAT(angle(uz, ux), Catch::Matchers::WithinAbs(M_PI/2, 1e-6));
}

TEST_CASE("Vector - Angle Range", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {-4.0f, -5.0f, -6.0f};
    float result = angle(v1, v2);

    // Angle should be in [0, π]
    REQUIRE(result >= 0.0f);
    REQUIRE(result <= M_PI + 1e-5);
    }
    TEST_CASE("Vector - Angle Cosine Relation", "[vector][ops][angle]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float ang = angle(v1, v2);
    float cosAngle = std::cos(ang);
    float expected = dot(v1, v2) / (norm(v1) * norm(v2));

    REQUIRE_THAT(cosAngle, Catch::Matchers::WithinAbs(expected, 1e-5));
}

// ----------------------------------------------------------------------------
// Project Tests
// ----------------------------------------------------------------------------
TEST_CASE("Vector - Project Basic", "[vector][ops][project]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    Vector<3> result = project(v, u);

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Onto Self", "[vector][ops][project]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> result = project(v, v);

    REQUIRE(result == v);
}

TEST_CASE("Vector - Project Onto Zero Vector", "[vector][ops][project]") {
    Vector<3> v = {1.0f, 1.0f, 0.0f};
    Vector<3> u = {0.0f, 0.0f, 0.0f};
    Vector<3> result = project(v, u);

    // Should return zero vector
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Orthogonal Vectors", "[vector][ops][project]") {
    Vector<3> v = {0.0f, 1.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    Vector<3> result = project(v, u);

    // Projection of orthogonal vectors is zero
    REQUIRE(result == Vector<3>::zero());
}

TEST_CASE("Vector - Project Parallel Vectors", "[vector][ops][project]") {
    Vector<3> v = {2.0f, 0.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    Vector<3> result = project(v, u);

    // Projection of parallel vectors equals v
    REQUIRE(result == v);
}

TEST_CASE("Vector - Project Result Parallel To Target", "[vector][ops][project]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {1.0f, 1.0f, 1.0f};
    Vector<3> result = project(v, u);

    // Result should be parallel to u
    Vector<3> normalized_u = normalize(u);
    Vector<3> normalized_result = normalize(result);

    REQUIRE_THAT(std::abs(dot(normalized_u, normalized_result)), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Vector - Project Magnitude", "[vector][ops][project]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    Vector<3> result = project(v, u);

    // |proj_u(v)| = |v| * |cos(angle)|
    float expected = std::abs(dot(v, u)) / norm(u);
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(expected, 1e-5));
}

TEST_CASE("Vector - Project Scale Invariance", "[vector][ops][project]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> u = {1.0f, 1.0f, 0.0f};
    float scale = 3.0f;
    Vector<3> result1 = project(v, u);
    Vector<3> result2 = project(v, scale * u);

    // Projection is invariant to scaling of u
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result1[i], Catch::Matchers::WithinAbs(result2[i], 1e-5));
    }
}

TEST_CASE("Vector - Project Zero Vector", "[vector][ops][project]") {
    Vector<3> zero = Vector<3>::zero();
    Vector<3> u = {1.0f, 2.0f, 3.0f};
    Vector<3> result = project(zero, u);

    REQUIRE(result == zero);
}

TEST_CASE("Vector - Project Onto Unit Vector", "[vector][ops][project]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> ux = Vector<3>::unitX();
    Vector<3> result = project(v, ux);

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Project Decomposition", "[vector][ops][project]") {
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> u = {1.0f, 0.0f, 0.0f};
    Vector<3> proj = project(v, u);
    Vector<3> perp = v - proj;

    // proj and perp should be orthogonal
    REQUIRE_THAT(dot(proj, perp), Catch::Matchers::WithinAbs(0.0, 1e-5));

    // proj + perp should equal v
    Vector<3> reconstructed = proj + perp;
    REQUIRE(reconstructed == v);
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------
TEST_CASE("Vector - Combined Operations Complex Expression", "[vector][integration]") {
    Vector<3> a = {1.0f, 0.0f, 0.0f};
    Vector<3> b = {0.0f, 1.0f, 0.0f};
    Vector<3> c = {0.0f, 0.0f, 1.0f};
    // Test complex expression: 2a + 3b - c
    Vector<3> result = 2.0f * a + 3.0f * b - c;

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Vector - Gram-Schmidt Step", "[vector][integration]") {
    Vector<3> v1 = {1.0f, 1.0f, 0.0f};
    Vector<3> v2 = {1.0f, 0.0f, 1.0f};
    // Orthogonalize v2 with respect to v1
    Vector<3> u1 = normalize(v1);
    Vector<3> projection = project(v2, u1);
    Vector<3> u2_unnormalized = v2 - projection;
    Vector<3> u2 = normalize(u2_unnormalized);

    // Check orthogonality
    REQUIRE_THAT(dot(u1, u2), Catch::Matchers::WithinAbs(0.0, 1e-5));

    // Check normalization
    REQUIRE_THAT(norm(u1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(norm(u2), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Centroid Calculation", "[vector][integration]") {
    std::array<Vector<3>, 4> points = {
        Vector<3>{0.0f, 0.0f, 0.0f},
        Vector<3>{2.0f, 0.0f, 0.0f},
        Vector<3>{2.0f, 2.0f, 0.0f},
        Vector<3>{0.0f, 2.0f, 0.0f}
    };

    Vector<3> centroid = Vector<3>::zero();

    for(const auto& p : points) { centroid += p; }
    centroid /= 4.0f;

    REQUIRE_THAT(centroid[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(centroid[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(centroid[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
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

TEST_CASE("Vector - Reflection Physics", "[vector][integration]") {
    // Incident vector
    Vector<3> incident = {1.0f, -1.0f, 0.0f};
    // Surface normal (pointing up)
    Vector<3> normal = {0.0f, 1.0f, 0.0f};
    // Calculate reflection: v - 2(v·n)n
    Vector<3> reflected = incident - 2.0f * dot(incident, normal) * normal;

    REQUIRE_THAT(reflected[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(reflected[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(reflected[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
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

TEST_CASE("Vector - Linear Combination", "[vector][integration]") {
    Vector<3> v1 = {1.0f, 0.0f, 0.0f};
    Vector<3> v2 = {0.0f, 1.0f, 0.0f};
    Vector<3> v3 = {0.0f, 0.0f, 1.0f};
    float a = 2.0f, b = 3.0f, c = 4.0f;

    Vector<3> result = a * v1 + b * v2 + c * v3;

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Cauchy-Schwarz Inequality", "[vector][integration]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    float dotProduct = std::abs(dot(v1, v2));
    float productOfNorms = norm(v1) * norm(v2);

    // |v1·v2| <= ||v1|| * ||v2||
    REQUIRE(dotProduct <= productOfNorms + 1e-5);
}

TEST_CASE("Vector - Pythagorean Theorem in 3D", "[vector][integration]") {
    Vector<3> v = {3.0f, 4.0f, 12.0f};
    float normSquared = normSqr(v);
    float sumOfSquares = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];

    REQUIRE_THAT(normSquared, Catch::Matchers::WithinAbs(sumOfSquares, 1e-6));
    REQUIRE_THAT(norm(v), Catch::Matchers::WithinAbs(13.0, 1e-6));
    }
    TEST_CASE("Vector - Different Dimensions", "[vector][integration]") {
    Vector<2> v2 = {1.0f, 2.0f};
    Vector<4> v4 = {1.0f, 2.0f, 3.0f, 4.0f};
    Vector<6> v6 = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    REQUIRE(v2.size() == 2);
    REQUIRE(v4.size() == 4);
    REQUIRE(v6.size() == 6);

    REQUIRE_THAT(norm(v2), Catch::Matchers::WithinAbs(std::sqrt(5.0f), 1e-6));
    REQUIRE_THAT(dot(v4, v4), Catch::Matchers::WithinAbs(30.0, 1e-6));
}