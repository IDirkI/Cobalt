#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/algebra/complex/complex.hpp"

using namespace cobalt::math::algebra;

// ================================================================================
// Complex Core Tests (complex.hpp)
// ================================================================================

TEST_CASE("Complex - Default Constructor", "[complex][core]") {
    Complex c;
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Real Only Constructor", "[complex][core]") {
    Complex c(3.5f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(3.5f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Real and Imaginary Constructor", "[complex][core]") {
    Complex c(2.0f, 3.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Negative Values Constructor", "[complex][core]") {
    Complex c(-4.5f, -2.5f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(-4.5f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(-2.5f, 1e-6));
}

TEST_CASE("Complex - Zero Factory", "[complex][core][factory]") {
    Complex c = Complex::zero();
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - One Factory", "[complex][core][factory]") {
    Complex c = Complex::one();
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Imaginary Unit Factory", "[complex][core][factory]") {
    Complex c = Complex::oneIm();
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Complex - Polar Form r=1, theta=0", "[complex][core][factory]") {
    Complex c = Complex::polar(1.0f, 0.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Polar Form r=1, theta=pi/2", "[complex][core][factory]") {
    Complex c = Complex::polar(1.0f, M_PI / 2.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Complex - Polar Form r=2, theta=pi/4", "[complex][core][factory]") {
    Complex c = Complex::polar(2.0f, M_PI / 4.0f);
    
    float expected_real = 2.0f * std::cos(M_PI / 4.0f);
    float expected_imag = 2.0f * std::sin(M_PI / 4.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(expected_real, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(expected_imag, 1e-6));
}

TEST_CASE("Complex - Polar Form Euler's Identity", "[complex][core][factory]") {
    // e^(iπ) = -1, so r=1, θ=π should give (-1, 0)
    Complex c = Complex::polar(1.0f, M_PI);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Set Real Part", "[complex][core][accessor]") {
    Complex c(1.0f, 2.0f);
    c.real(5.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
}

TEST_CASE("Complex - Set Imaginary Part", "[complex][core][accessor]") {
    Complex c(1.0f, 2.0f);
    c.imag(7.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(7.0f, 1e-6));
}

TEST_CASE("Complex - Set Both Parts", "[complex][core][accessor]") {
    Complex c;
    c.real(3.0f);
    c.imag(4.0f);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Complex - Addition with Complex", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex b(4.0f, 5.0f);
    
    a += b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(8.0f, 1e-6));
}

TEST_CASE("Complex - Addition with Real", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    
    a += 5.0f;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(7.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Addition with Zero", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex zero = Complex::zero();
    
    a += zero;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Addition Commutativity", "[complex][core][ops]") {
    Complex a(1.0f, 2.0f);
    Complex b(3.0f, 4.0f);
    Complex c(1.0f, 2.0f);
    Complex d(3.0f, 4.0f);
    
    a += b;
    d += c;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(d.real(), 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(d.imag(), 1e-6));
}

TEST_CASE("Complex - Subtraction with Complex", "[complex][core][ops]") {
    Complex a(5.0f, 7.0f);
    Complex b(2.0f, 3.0f);
    
    a -= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
}

TEST_CASE("Complex - Subtraction with Real", "[complex][core][ops]") {
    Complex a(5.0f, 3.0f);
    
    a -= 2.0f;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Subtraction with Zero", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex zero = Complex::zero();
    
    a -= zero;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Subtraction Self", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex b(2.0f, 3.0f);
    
    a -= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Subtraction Negative Result", "[complex][core][ops]") {
    Complex a(1.0f, 2.0f);
    Complex b(3.0f, 5.0f);
    
    a -= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(-2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(-3.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication with Complex", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex b(4.0f, 5.0f);
    
    a *= b;
    
    // (2+3i)(4+5i) = 8+10i+12i+15i² = 8+22i-15 = -7+22i
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(-7.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(22.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication with Real", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    
    a *= 2.0f;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(6.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication with Zero", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex zero = Complex::zero();
    
    a *= zero;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication with One", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex one = Complex::one();
    
    a *= one;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication with i", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex i = Complex::oneIm();
    
    a *= i;
    
    // (2+3i)(i) = 2i+3i² = 2i-3 = -3+2i
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(-3.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
}

TEST_CASE("Complex - Multiplication Commutativity", "[complex][core][ops]") {
    Complex a(1.0f, 2.0f);
    Complex b(3.0f, 4.0f);
    Complex c(1.0f, 2.0f);
    Complex d(3.0f, 4.0f);
    
    a *= b;
    d *= c;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(d.real(), 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(d.imag(), 1e-6));
}

TEST_CASE("Complex - Multiplication i^2 = -1", "[complex][core][ops]") {
    Complex i = Complex::oneIm();
    
    i *= Complex::oneIm();
    
    REQUIRE_THAT(i.real(), Catch::Matchers::WithinAbs(-1.0f, 1e-6));
    REQUIRE_THAT(i.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Division with Complex", "[complex][core][ops]") {
    Complex a(10.0f, 5.0f);
    Complex b(2.0f, 1.0f);
    
    a /= b;
    
    // (10+5i)/(2+i) = (10+5i)(2-i)/(4+1) = (20-10i+10i-5i²)/5 = (20+5)/5 = 25/5 = 5
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Division with Real", "[complex][core][ops]") {
    Complex a(4.0f, 6.0f);
    
    a /= 2.0f;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Division with One", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex one = Complex::one();
    
    a /= one;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
}

TEST_CASE("Complex - Division by i", "[complex][core][ops]") {
    Complex a(2.0f, 3.0f);
    Complex i = Complex::oneIm();
    
    a /= i;
    
    // (2+3i)/i = (2+3i)(-i)/(i*-i) = (-2i-3i²)/1 = (-2i+3)/1 = 3-2i
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(-2.0f, 1e-6));
}

TEST_CASE("Complex - Division Self", "[complex][core][ops]") {
    Complex a(3.0f, 4.0f);
    Complex b(3.0f, 4.0f);
    
    a /= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Division Pure Imaginary", "[complex][core][ops]") {
    Complex a(0.0f, 8.0f);
    Complex b(0.0f, 2.0f);
    
    a /= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(4.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Complex - Chained Operations", "[complex][integration]") {
    Complex a(1.0f, 2.0f);
    Complex b(2.0f, 1.0f);
    
    a += b;
    a *= 2.0f;
    a -= Complex(1.0f, 1.0f);
    
    // (1+2i) + (2+1i) = (3+3i)
    // (3+3i) * 2 = (6+6i)
    // (6+6i) - (1+1i) = (5+5i)
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(5.0f, 1e-6));
}

TEST_CASE("Complex - Multiplicative Inverse Property", "[complex][integration]") {
    Complex a(3.0f, 4.0f);
    Complex b(3.0f, 4.0f);
    Complex one = Complex::one();
    
    a *= b;
    a /= b;
    
    REQUIRE_THAT(a.real(), Catch::Matchers::WithinAbs(3.0f, 1e-4));
    REQUIRE_THAT(a.imag(), Catch::Matchers::WithinAbs(4.0f, 1e-4));
}

TEST_CASE("Complex - Distributive Property", "[complex][integration]") {
    Complex a(2.0f, 1.0f);
    Complex b(3.0f, 2.0f);
    Complex c(1.0f, 1.0f);
    
    // a * (b + c)
    Complex sum = b;
    sum += c;
    Complex left = a;
    left *= sum;
    
    // a * b + a * c
    Complex term1 = a;
    term1 *= b;
    Complex term2 = a;
    term2 *= c;
    Complex right = term1;
    right += term2;
    
    REQUIRE_THAT(left.real(), Catch::Matchers::WithinAbs(right.real(), 1e-6));
    REQUIRE_THAT(left.imag(), Catch::Matchers::WithinAbs(right.imag(), 1e-6));
}

TEST_CASE("Complex - Polar to Cartesian Roundtrip", "[complex][integration]") {
    float r = 5.0f;
    float theta = M_PI / 3.0f;
    
    Complex c = Complex::polar(r, theta);
    
    // Verify magnitude: |c| = sqrt(re² + im²) = r
    float mag_squared = c.real() * c.real() + c.imag() * c.imag();
    REQUIRE_THAT(std::sqrt(mag_squared), Catch::Matchers::WithinAbs(r, 1e-6));
    
    // Verify angle: arg(c) = atan2(im, re) = theta
    float angle = std::atan2(c.imag(), c.real());
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(theta, 1e-6));
}

TEST_CASE("Complex - De Moivre's Formula", "[complex][integration]") {
    // (cos(θ) + i*sin(θ))² = cos(2θ) + i*sin(2θ)
    float theta = M_PI / 6.0f;
    Complex c = Complex::polar(1.0f, theta);

    CAPTURE(c.real());
    CAPTURE(c.imag());
    
    c *= c;
    
    Complex expected = Complex::polar(1.0f, 2.0f * theta);
    
    REQUIRE_THAT(c.real(), Catch::Matchers::WithinAbs(expected.real(), 1e-6));
    REQUIRE_THAT(c.imag(), Catch::Matchers::WithinAbs(expected.imag(), 1e-6));
}