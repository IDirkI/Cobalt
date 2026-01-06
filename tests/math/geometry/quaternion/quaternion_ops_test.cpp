#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Quaternion Operations Tests (quaternion_ops.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Binary Arithmetic Operators
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Addition Operator", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion<> result = q1 + q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Quaternion - Addition Preserves Operands", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    Quaternion<> q1_copy(q1);
    Quaternion<> q2_copy(q2);
    
    Quaternion<> result = q1 + q2;
    
    REQUIRE(q1 == q1_copy);
    REQUIRE(q2 == q2_copy);
}

TEST_CASE("Quaternion - Addition Commutative", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion<> result1 = q1 + q2;
    Quaternion<> result2 = q2 + q1;
    
    REQUIRE(result1 == result2);
}

TEST_CASE("Quaternion - Addition With Zero", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> zero = Quaternion<>::zero();
    
    Quaternion<> result = q + zero;
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Addition Scalar Right", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = q + 5.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Addition Scalar Left", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = 5.0f + q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Subtraction Operator", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(5.0f, 6.0f, 7.0f, 8.0f);
    Quaternion<> q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = q1 - q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Subtraction Self Zeros", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = q - q;
    
    REQUIRE(result == Quaternion<>::zero());
}

TEST_CASE("Quaternion - Subtraction Not Commutative", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion<> result1 = q1 - q2;
    Quaternion<> result2 = q2 - q1;
    
    REQUIRE(result1 != result2);
    REQUIRE(result1 == -result2);
}

TEST_CASE("Quaternion - Subtraction Scalar Right", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(5.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = q - 2.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Subtraction Scalar Left", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = 5.0f - q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Operator", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion<> q2(0.0f, 1.0f, 0.0f, 0.0f);
    
    Quaternion<> result = q1 * q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Not Commutative", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(0.0f, 1.0f, 0.0f, 0.0f);  // i
    Quaternion<> q2(0.0f, 0.0f, 1.0f, 0.0f);  // j
    
    Quaternion<> result1 = q1 * q2;  // i*j = k
    Quaternion<> result2 = q2 * q1;  // j*i = -k
    
    REQUIRE(result1 != result2);
    REQUIRE_THAT(result1.z(), Catch::Matchers::WithinAbs(-result2.z(), 1e-6));
}

TEST_CASE("Quaternion - Multiplication Identity Left", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> identity = Quaternion<>::eye();
    
    Quaternion<> result = identity * q;
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Multiplication Identity Right", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> identity = Quaternion<>::eye();
    
    Quaternion<> result = q * identity;
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Multiplication Scalar Right", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = q * 2.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Quaternion - Multiplication Scalar Left", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = 3.0f * q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Quaternion - Division Operator", "[quaternion][ops][arithmetic]") {
    Quaternion<> q1(2.0f, 0.0f, 0.0f, 0.0f);
    Quaternion<> q2(2.0f, 0.0f, 0.0f, 0.0f);
    
    Quaternion<> result = q1 / q2;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Division Scalar Right", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(2.0f, 4.0f, 6.0f, 8.0f);
    
    Quaternion<> result = q / 2.0f;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Division Scalar Left", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(2.0f, 0.0f, 0.0f, 0.0f);
    
    Quaternion<> result = 4.0f / q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Unary Negation", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, -2.0f, 3.0f, -4.0f);
    
    Quaternion<> result = -q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Quaternion - Double Negation", "[quaternion][ops][arithmetic]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = -(-q);
    
    REQUIRE(result == q);
}

// ----------------------------------------------------------------------------
// Comparison Operators
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Equality Operator Same Values", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE(q1 == q2);
}

TEST_CASE("Quaternion - Equality Operator Different Values", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(1.0f, 2.0f, 3.0f, 5.0f);
    
    REQUIRE_FALSE(q1 == q2);
}

TEST_CASE("Quaternion - Equality Operator Within Epsilon", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(1.0f + 1e-7f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE(q1 == q2);
}

TEST_CASE("Quaternion - Equality Reflexive", "[quaternion][ops][comparison]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE(q == q);
}

TEST_CASE("Quaternion - Equality Symmetric", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(q1);
    
    REQUIRE(q1 == q2);
    REQUIRE(q2 == q1);
}

TEST_CASE("Quaternion - Inequality Operator", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(1.0f, 2.0f, 3.0f, 5.0f);
    
    REQUIRE(q1 != q2);
}

TEST_CASE("Quaternion - Inequality Operator Same Values", "[quaternion][ops][comparison]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(q1);
    
    REQUIRE_FALSE(q1 != q2);
}

// ----------------------------------------------------------------------------
// Norm Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Norm Unit Quaternion", "[quaternion][ops][norm]") {
    Quaternion<> q = Quaternion<>::eye();
    
    float result = norm(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - Norm Basic", "[quaternion][ops][norm]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    float result = norm(q);
    
    // sqrt(1 + 4 + 9 + 16) = sqrt(30)
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(std::sqrt(30.0f), 1e-6));
}

TEST_CASE("Quaternion - Norm Zero Quaternion", "[quaternion][ops][norm]") {
    Quaternion<> q = Quaternion<>::zero();
    
    float result = norm(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Norm Positive Definite", "[quaternion][ops][norm]") {
    Quaternion<> q(-1.0f, -2.0f, -3.0f, -4.0f);
    
    float result = norm(q);
    
    REQUIRE(result > 0.0f);
}

TEST_CASE("Quaternion - Norm Squared", "[quaternion][ops][norm]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    float result = normSqr(q);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(30.0, 1e-6));
}

TEST_CASE("Quaternion - Norm Squared More Efficient", "[quaternion][ops][norm]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    float normSquared = normSqr(q);
    float normThenSquared = norm(q) * norm(q);
    
    REQUIRE_THAT(normSquared, Catch::Matchers::WithinAbs(normThenSquared, 1e-5));
}

// ----------------------------------------------------------------------------
// Conjugate Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Conjugate Basic", "[quaternion][ops][conjugate]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = conj(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Quaternion - Conjugate Identity", "[quaternion][ops][conjugate]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Quaternion<> result = conj(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Conjugate Double Application", "[quaternion][ops][conjugate]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = conj(conj(q));
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Conjugate Pure Imaginary", "[quaternion][ops][conjugate]") {
    Quaternion<> q(0.0f, 1.0f, 2.0f, 3.0f);
    
    Quaternion<> result = conj(q);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Inverse Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Inverse Identity", "[quaternion][ops][inverse]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Quaternion<> result = inv(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Inverse Unit Quaternion", "[quaternion][ops][inverse]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI/2);
    q = normalize(q);
    
    Quaternion<> result = inv(q);
    
    // For unit quaternion, inverse equals conjugate
    Quaternion<> conjugate = conj(q);
    REQUIRE(result == conjugate);
}

TEST_CASE("Quaternion - Inverse Multiply Gives Identity", "[quaternion][ops][inverse]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> qInv = inv(q);
    Quaternion<> result = q * qInv;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Inverse Double Application", "[quaternion][ops][inverse]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = inv(inv(q));
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-5));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-5));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-5));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-5));
}

// ----------------------------------------------------------------------------
// Dot Product Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Dot Product Basic", "[quaternion][ops][dot]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    float result = dot(q1, q2);
    
    // 1*5 + 2*6 + 3*7 + 4*8 = 5 + 12 + 21 + 32 = 70
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(70.0, 1e-6));
}

TEST_CASE("Quaternion - Dot Product Commutative", "[quaternion][ops][dot]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    float result1 = dot(q1, q2);
    float result2 = dot(q2, q1);
    
    REQUIRE_THAT(result1, Catch::Matchers::WithinAbs(result2, 1e-6));
}

TEST_CASE("Quaternion - Dot Product With Self", "[quaternion][ops][dot]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    float result = dot(q, q);
    
    // Should equal squared norm
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(normSqr(q), 1e-6));
}

TEST_CASE("Quaternion - Dot Product With Zero", "[quaternion][ops][dot]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> zero = Quaternion<>::zero();
    
    float result = dot(q, zero);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Normalize Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Normalize Basic", "[quaternion][ops][normalize]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = normalize(q);
    
    float n = norm(result);
    REQUIRE_THAT(n, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Quaternion - Normalize Already Normalized", "[quaternion][ops][normalize]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Quaternion<> result = normalize(q);
    
    REQUIRE(result == q);
}

TEST_CASE("Quaternion - Normalize Zero Returns Zero", "[quaternion][ops][normalize]") {
    Quaternion<> q = Quaternion<>::zero();
    
    Quaternion<> result = normalize(q);
    
    REQUIRE(result == Quaternion<>::zero());
}

TEST_CASE("Quaternion - Normalize Direction Preserved", "[quaternion][ops][normalize]") {
    Quaternion<> q(2.0f, 4.0f, 6.0f, 8.0f);
    
    Quaternion<> result = normalize(q);
    
    float scale = norm(q);
    REQUIRE_THAT(result.w() * scale, Catch::Matchers::WithinAbs(q.w(), 1e-5));
    REQUIRE_THAT(result.x() * scale, Catch::Matchers::WithinAbs(q.x(), 1e-5));
    REQUIRE_THAT(result.y() * scale, Catch::Matchers::WithinAbs(q.y(), 1e-5));
    REQUIRE_THAT(result.z() * scale, Catch::Matchers::WithinAbs(q.z(), 1e-5));
}

// ----------------------------------------------------------------------------
// Logarithm Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Log Identity", "[quaternion][ops][log]") {
    Quaternion<> q = Quaternion<>::eye();
    
    Quaternion<> result = log(q);
    
    // log(1) ≈ 0
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(norm(result.vector()), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Log Pure Real", "[quaternion][ops][log]") {
    Quaternion<> q(2.0f, 0.0f, 0.0f, 0.0f);
    
    Quaternion<> result = log(q);
    
    // Should have real part log(2) and imaginary parts ~0
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(std::log(2.0f), 1e-5));
    REQUIRE_THAT(norm(result.vector()), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Log Unit Quaternion", "[quaternion][ops][log]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 4.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Quaternion<> result = log(q);
    
    // For unit quaternion, w should be ~0
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

// ----------------------------------------------------------------------------
// Exponential Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Exp Zero", "[quaternion][ops][exp]") {
    Quaternion<> q = Quaternion<>::zero();
    
    Quaternion<> result = exp(q);
    
    // exp(0) = 1
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Exp Pure Real", "[quaternion][ops][exp]") {
    Quaternion<> q(1.0f, 0.0f, 0.0f, 0.0f);
    
    Quaternion<> result = exp(q);
    
    // exp(1) = e
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(std::exp(1.0f), 1e-5));
    REQUIRE_THAT(norm(result.vector()), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Exp Log Inverse", "[quaternion][ops][exp]") {
    Quaternion<> q(1.1f, 0.1f, 0.1f, 0.1f);
    
    Quaternion<> logQ = log(q);
    Quaternion<> result = exp(logQ);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-3));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-3));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-3));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-3));
}

// ----------------------------------------------------------------------------
// Power Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Power Zero Exponent", "[quaternion][ops][pow]") {
    Quaternion<> q(1.0f, 2.0f, 3.0f, 4.0f);
    
    Quaternion<> result = pow(q, 0.0f);
    
    // q^0 = 1
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(1.0, 1e-3));
    REQUIRE_THAT(norm(result.vector()), Catch::Matchers::WithinAbs(0.0, 1e-3));
}

TEST_CASE("Quaternion - Power One Exponent", "[quaternion][ops][pow]") {
    Quaternion<> q(1.1f, 0.1f, 0.1f, 0.1f);
    
    Quaternion<> result = pow(q, 1.0f);
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(q.w(), 1e-3));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(q.x(), 1e-3));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(q.y(), 1e-3));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(q.z(), 1e-3));
}

TEST_CASE("Quaternion - Power Square", "[quaternion][ops][pow]") {
    Quaternion<> q(1.1f, 0.1f, 0.1f, 0.1f);
    
    Quaternion<> result = pow(q, 2.0f);
    Quaternion<> expected = q * q;
    
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(expected.w(), 1e-2));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(expected.x(), 1e-2));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(expected.y(), 1e-2));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(expected.z(), 1e-2));
}

TEST_CASE("Quaternion - Power Fractional", "[quaternion][ops][pow]") {
    Quaternion<> q(1.1f, 0.1f, 0.1f, 0.1f);
    
    Quaternion<> result = pow(q, 0.5f);
    
    // q^0.5 * q^0.5 should equal q
    Quaternion<> squared = result * result;
    REQUIRE_THAT(squared.w(), Catch::Matchers::WithinAbs(q.w(), 1e-2));
}

// ----------------------------------------------------------------------------
// Rotate Vector Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Rotate Vector X-Axis 90", "[quaternion][ops][rotate]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 2.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {0.0f, 1.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector Y-Axis 90", "[quaternion][ops][rotate]") {
    Vector<3> axis = {0.0f, 1.0f, 0.0f};
    float angle = M_PI / 2.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(-1.0, 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector Z-Axis 90", "[quaternion][ops][rotate]") {
    Vector<3> axis = {0.0f, 0.0f, 1.0f};
    float angle = M_PI / 2.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector Identity", "[quaternion][ops][rotate]") {
    Quaternion<> q = Quaternion<>::eye();
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(v[0], 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(v[1], 1e-6));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(v[2], 1e-6));
}

TEST_CASE("Quaternion - Rotate Vector Preserves Length", "[quaternion][ops][rotate]") {
    Vector<3> axis = {1.0f, 1.0f, 1.0f};
    float angle = M_PI / 3.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {3.0f, 4.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(norm(v), 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector 180 Degrees", "[quaternion][ops][rotate]") {
    Vector<3> axis = {0.0f, 0.0f, 1.0f};
    float angle = M_PI;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {1.0f, 0.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector 360 Degrees", "[quaternion][ops][rotate]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = 2.0f * M_PI;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {0.0f, 1.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    // 360 degree rotation should return to original
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(v[0], 1e-4));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(v[1], 1e-4));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(v[2], 1e-4));
}

TEST_CASE("Quaternion - Rotate Vector Parallel To Axis", "[quaternion][ops][rotate]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 2.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {5.0f, 0.0f, 0.0f};  // Parallel to axis
    Vector<3> result = rotate(q, v);
    
    // Vector parallel to axis should be unchanged
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(v[0], 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(v[1], 1e-5));
    REQUIRE_THAT(result[2], Catch::Matchers::WithinAbs(v[2], 1e-5));
}

TEST_CASE("Quaternion - Rotate Vector Zero Vector", "[quaternion][ops][rotate]") {
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    float angle = M_PI / 2.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    
    Vector<3> v = {0.0f, 0.0f, 0.0f};
    Vector<3> result = rotate(q, v);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Quaternion - Multiplication Associative", "[quaternion][integration]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    Quaternion<> q3(9.0f, 10.0f, 11.0f, 12.0f);
    
    Quaternion<> result1 = (q1 * q2) * q3;
    Quaternion<> result2 = q1 * (q2 * q3);
    
    REQUIRE_THAT(result1.w(), Catch::Matchers::WithinAbs(result2.w(), 1e-5));
    REQUIRE_THAT(result1.x(), Catch::Matchers::WithinAbs(result2.x(), 1e-5));
    REQUIRE_THAT(result1.y(), Catch::Matchers::WithinAbs(result2.y(), 1e-5));
    REQUIRE_THAT(result1.z(), Catch::Matchers::WithinAbs(result2.z(), 1e-5));
}

TEST_CASE("Quaternion - Rotation Composition", "[quaternion][integration]") {
    // Two 90 degree rotations about X should equal 180 degree rotation
    Vector<3> axis = {1.0f, 0.0f, 0.0f};
    Quaternion<> q90 = Quaternion<>::fromAxisAngle(axis, M_PI / 2.0f);
    Quaternion<> q180 = Quaternion<>::fromAxisAngle(axis, M_PI);
    
    Quaternion<> composed = q90 * q90;
    
    // Apply to test vector
    Vector<3> v = {0.0f, 1.0f, 0.0f};
    Vector<3> result1 = rotate(composed, v);
    Vector<3> result2 = rotate(q180, v);
    
    REQUIRE_THAT(result1[0], Catch::Matchers::WithinAbs(result2[0], 1e-5));
    REQUIRE_THAT(result1[1], Catch::Matchers::WithinAbs(result2[1], 1e-5));
    REQUIRE_THAT(result1[2], Catch::Matchers::WithinAbs(result2[2], 1e-5));
}

TEST_CASE("Quaternion - Inverse Rotation", "[quaternion][integration]") {
    Vector<3> axis = {1.0f, 1.0f, 1.0f};
    float angle = M_PI / 3.0f;
    Quaternion<> q = Quaternion<>::fromAxisAngle(axis, angle);
    Quaternion<> qInv = inv(q);
    
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> rotated = rotate(q, v);
    Vector<3> restored = rotate(qInv, rotated);
    
    REQUIRE_THAT(restored[0], Catch::Matchers::WithinAbs(v[0], 1e-5));
    REQUIRE_THAT(restored[1], Catch::Matchers::WithinAbs(v[1], 1e-5));
    REQUIRE_THAT(restored[2], Catch::Matchers::WithinAbs(v[2], 1e-5));
}

TEST_CASE("Quaternion - Quaternion Identities i*j*k", "[quaternion][integration]") {
    Quaternion<> i(0.0f, 1.0f, 0.0f, 0.0f);
    Quaternion<> j(0.0f, 0.0f, 1.0f, 0.0f);
    Quaternion<> k(0.0f, 0.0f, 0.0f, 1.0f);
    
    Quaternion<> result = i * j * k;
    
    // i*j*k = -1
    REQUIRE_THAT(result.w(), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Quaternion - Norm Multiplicative Property", "[quaternion][integration]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    float norm1 = norm(q1);
    float norm2 = norm(q2);
    float normProduct = norm(q1 * q2);
    
    // ||q1 * q2|| = ||q1|| * ||q2||
    REQUIRE_THAT(normProduct, Catch::Matchers::WithinAbs(norm1 * norm2, 1e-4));
}

TEST_CASE("Quaternion - Conjugate Distributive", "[quaternion][integration]") {
    Quaternion<> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<> q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    Quaternion<> result1 = conj(q1 * q2);
    Quaternion<> result2 = conj(q2) * conj(q1);
    
    REQUIRE_THAT(result1.w(), Catch::Matchers::WithinAbs(result2.w(), 1e-5));
    REQUIRE_THAT(result1.x(), Catch::Matchers::WithinAbs(result2.x(), 1e-5));
    REQUIRE_THAT(result1.y(), Catch::Matchers::WithinAbs(result2.y(), 1e-5));
    REQUIRE_THAT(result1.z(), Catch::Matchers::WithinAbs(result2.z(), 1e-5));
}

TEST_CASE("Quaternion - Unit Quaternion Conjugate Equals Inverse", "[quaternion][integration]") {
    Vector<3> axis = {1.0f, 2.0f, 3.0f};
    float angle = M_PI / 4.0f;
    Quaternion<> q = normalize(Quaternion<>::fromAxisAngle(axis, angle));
    
    Quaternion<> conjugate = conj(q);
    Quaternion<> inverse = inv(q);
    
    REQUIRE_THAT(conjugate.w(), Catch::Matchers::WithinAbs(inverse.w(), 1e-5));
    REQUIRE_THAT(conjugate.x(), Catch::Matchers::WithinAbs(inverse.x(), 1e-5));
    REQUIRE_THAT(conjugate.y(), Catch::Matchers::WithinAbs(inverse.y(), 1e-5));
    REQUIRE_THAT(conjugate.z(), Catch::Matchers::WithinAbs(inverse.z(), 1e-5));
}