#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/algebra/complex/complex.hpp"
#include "cobalt/math/algebra/complex/complex_ops.hpp"

using namespace cobalt::math::algebra;

// ================================================================================
// Complex Ops Tests (complex_ops.hpp)
// ================================================================================

TEST_CASE("Complex - Binary Addition Complex + Complex", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    Complex b(4.0f, 5.0f);
    
    Complex result = a + b;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(6.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(8.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Addition Complex + Float", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = a + 5.0f;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(7.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Addition Float + Complex", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = 5.0f + a;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(7.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Subtraction Complex - Complex", "[complex][ops][binary]") {
    Complex a(5.0f, 7.0f);
    Complex b(2.0f, 3.0f);
    
    Complex result = a - b;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Subtraction Complex - Float", "[complex][ops][binary]") {
    Complex a(5.0f, 3.0f);
    
    Complex result = a - 2.0f;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Subtraction Float - Complex", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = 5.0f - a;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Multiplication Complex * Complex", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    Complex b(4.0f, 5.0f);
    
    Complex result = a * b;
    
    // (2+3i)(4+5i) = 8+10i+12i+15i² = -7+22i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-7.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(22.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Multiplication Complex * Float", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = a * 2.0f;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(6.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Multiplication Float * Complex", "[complex][ops][binary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = 2.0f * a;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(6.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Division Complex / Complex", "[complex][ops][binary]") {
    Complex a(10.0f, 5.0f);
    Complex b(2.0f, 1.0f);
    
    Complex result = a / b;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Division Complex / Float", "[complex][ops][binary]") {
    Complex a(4.0f, 6.0f);
    
    Complex result = a / 2.0f;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Binary Division Float / Complex", "[complex][ops][binary]") {
    Complex a(3.0f, 4.0f);
    
    Complex result = 5.0f / a;
    
    // 5/(3+4i) = 5(3-4i)/(9+16) = (15-20i)/25 = 0.6-0.8i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.6f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-0.8f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Unary Negation", "[complex][ops][unary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = -a;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Double Negation", "[complex][ops][unary]") {
    Complex a(2.0f, 3.0f);
    
    Complex result = -(-a);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Equality Complex == Complex True", "[complex][ops][equality]") {
    Complex a(2.0f, 3.0f);
    Complex b(2.0f, 3.0f);
    
    REQUIRE(a == b);
}

TEST_CASE("Complex - Equality Complex == Complex False", "[complex][ops][equality]") {
    Complex a(2.0f, 3.0f);
    Complex b(2.0f, 4.0f);
    
    REQUIRE_FALSE(a == b);
}

TEST_CASE("Complex - Equality Complex == Float", "[complex][ops][equality]") {
    Complex a(2.0f, 0.0f);
    
    REQUIRE(a == 2.0f);
}

TEST_CASE("Complex - Equality Float == Complex", "[complex][ops][equality]") {
    Complex a(2.0f, 0.0f);
    
    REQUIRE(2.0f == a);
}

TEST_CASE("Complex - Inequality Complex != Complex", "[complex][ops][equality]") {
    Complex a(2.0f, 3.0f);
    Complex b(2.0f, 4.0f);
    
    REQUIRE(a != b);
}

TEST_CASE("Complex - Inequality Complex != Float", "[complex][ops][equality]") {
    Complex a(2.0f, 3.0f);
    
    REQUIRE(a != 2.0f);
}

TEST_CASE("Complex - Inequality Float != Complex", "[complex][ops][equality]") {
    Complex a(2.0f, 3.0f);
    
    REQUIRE(2.0f != a);
}

TEST_CASE("Complex - Norm of 3-4-5 Triangle", "[complex][ops][norm]") {
    Complex z(3.0f, 4.0f);
    
    float result = norm(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Norm of Unit Circle Point", "[complex][ops][norm]") {
    Complex z = Complex::polar(1.0f, M_PI / 4.0f);
    
    float result = norm(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Norm of Zero", "[complex][ops][norm]") {
    Complex z = Complex::zero();
    
    float result = norm(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Squared Norm", "[complex][ops][norm]") {
    Complex z(3.0f, 4.0f);
    
    float result = normSqr(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(25.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Norm Squared Equals Norm^2 ", "[complex][ops][norm]") {
    Complex z(5.0f, 12.0f);
    
    float normVal = norm(z);
    float normSqrVal = normSqr(z);
    
    REQUIRE_THAT(normVal * normVal, Catch::Matchers::WithinAbs(normSqrVal, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Argument of Positive Real", "[complex][ops][arg]") {
    Complex z(1.0f, 0.0f);
    
    float result = arg(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Argument of Positive Imaginary", "[complex][ops][arg]") {
    Complex z(0.0f, 1.0f);
    
    float result = arg(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI / 2.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Argument of Negative Real", "[complex][ops][arg]") {
    Complex z(-1.0f, 0.0f);
    
    float result = arg(z);
    
    REQUIRE_THAT(std::abs(result), Catch::Matchers::WithinAbs(M_PI, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Argument of First Quadrant", "[complex][ops][arg]") {
    Complex z(1.0f, 1.0f);
    
    float result = arg(z);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(M_PI / 4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Conjugate Basic", "[complex][ops][conj]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = conj(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Conjugate of Real Number", "[complex][ops][conj]") {
    Complex z(5.0f, 0.0f);
    
    Complex result = conj(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Double Conjugate", "[complex][ops][conj]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = conj(conj(z));
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Conjugate Multiplication Property", "[complex][ops][conj]") {
    Complex z(3.0f, 4.0f);
    Complex w = conj(z);
    
    Complex product = z * w;
    
    // z * conj(z) = |z|²
    REQUIRE_THAT(product.real(), Catch::Matchers::WithinAbs(25.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(product.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Inverse Basic", "[complex][ops][inv]") {
    Complex z(2.0f, 0.0f);
    
    Complex result = inv(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.5f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Inverse of i", "[complex][ops][inv]") {
    Complex z = Complex::oneIm();
    
    Complex result = inv(z);
    
    // 1/i = -i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Inverse Multiplication Property", "[complex][ops][inv]") {
    Complex z(3.0f, 4.0f);
    Complex w = inv(z);
    
    Complex product = z * w;
    
    // z * z⁻¹ = 1
    REQUIRE_THAT(product.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(product.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Double Inverse", "[complex][ops][inv]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = inv(inv(z));
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Exponential of Zero", "[complex][ops][exp]") {
    Complex z = Complex::zero();
    
    Complex result = exp(z);
    
    // e⁰ = 1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Exponential of Pure Imaginary", "[complex][ops][exp]") {
    Complex z(0.0f, M_PI);
    
    Complex result = exp(z);
    
    // e^(iπ) = -1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Exponential of Real Number", "[complex][ops][exp]") {
    Complex z(1.0f, 0.0f);
    
    Complex result = exp(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(M_E, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Euler's Formula", "[complex][ops][exp]") {
    float theta = M_PI / 4.0f;
    Complex z(0.0f, theta);
    
    Complex result = exp(z);
    
    // e^(iθ) = cos(θ) + i*sin(θ)
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(std::cos(theta), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(std::sin(theta), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Logarithm of One", "[complex][ops][log]") {
    Complex z = Complex::one();
    
    Complex result = log(z);
    
    // ln(1) = 0
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Logarithm of e", "[complex][ops][log]") {
    Complex z(M_E, 0.0f);
    
    Complex result = log(z);
    
    // ln(e) = 1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Logarithm of Negative Real", "[complex][ops][log]") {
    Complex z(-1.0f, 0.0f);
    
    Complex result = log(z);
    
    // ln(-1) = iπ
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(std::abs(result.imag()), Catch::Matchers::WithinAbs(M_PI, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Logarithm Inverse of Exponential", "[complex][ops][log]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = log(exp(z));
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), 1e-4));
}

TEST_CASE("Complex - Power to Zero", "[complex][ops][pow]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = pow(z, 0.0f);
    
    // z⁰ = 1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Power to One", "[complex][ops][pow]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = pow(z, 1.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Power to Two (Squared)", "[complex][ops][pow]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = pow(z, 2.0f);
    Complex squared = z * z;
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(squared.real(), 1e-4));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(squared.imag(), 1e-4));
}

TEST_CASE("Complex - Power i Squared", "[complex][ops][pow]") {
    Complex z = Complex::oneIm();
    
    Complex result = pow(z, 2.0f);
    
    // i² = -1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Square Root of One", "[complex][ops][sqrt]") {
    Complex z = Complex::one();
    
    Complex result = sqrt(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Square Root of Four", "[complex][ops][sqrt]") {
    Complex z(4.0f, 0.0f);
    
    Complex result = sqrt(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Square Root of Negative One", "[complex][ops][sqrt]") {
    Complex z(-1.0f, 0.0f);
    
    Complex result = sqrt(z);
    
    // √(-1) = i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(std::abs(result.imag()), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Square Root Inverse of Square", "[complex][ops][sqrt]") {
    Complex z(3.0f, 4.0f);
    
    Complex squared = z * z;
    Complex result = sqrt(squared);
    
    // Due to branch cuts, we check |z| = |√(z²)|
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(norm(z), 1e-4));
}

TEST_CASE("Complex - Complex Arithmetic Expression", "[complex][integration]") {
    Complex a(1.0f, 2.0f);
    Complex b(3.0f, 4.0f);
    Complex c(2.0f, 1.0f);
    
    // (a + b) * c - a
    Complex result = (a + b) * c - a;
    
    Complex expected = Complex(1.0f, 14.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(expected.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(expected.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Polar Form Consistency", "[complex][integration]") {
    Complex z(3.0f, 4.0f);
    
    float r = norm(z);
    float theta = arg(z);
    Complex polar_form = Complex::polar(r, theta);
    
    REQUIRE_THAT(polar_form.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(polar_form.imag(), Catch::Matchers::WithinAbs(z.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - exp(log(z)) = z", "[complex][integration]") {
    Complex z(2.0f, 3.0f);
    
    Complex result = exp(log(z));
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), 1e-4));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), 1e-4));
}

TEST_CASE("Complex - De Moivre's Theorem", "[complex][integration]") {
    // (cos(θ) + i*sin(θ))ⁿ = cos(nθ) + i*sin(nθ)
    float theta = M_PI / 6.0f;
    float n = 3.0f;
    
    Complex z = Complex::polar(1.0f, theta);
    Complex result = pow(z, n);
    Complex expected = Complex::polar(1.0f, n * theta);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(expected.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(expected.imag(), COMPLEX_EQUAL_THRESHOLD));
}