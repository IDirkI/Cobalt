#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/math/linear_algebra/vector/vector.hpp>
#include <cobalt/math/linear_algebra/vector/vector_ops.hpp>
#include <cobalt/math/linear_algebra/vector/vector_util.hpp>

#include <cobalt/math/algebra/complex/complex.hpp>
#include <cobalt/math/algebra/complex/complex_ops.hpp>

using cobalt::math::linear_algebra::Vector;
using cobalt::math::algebra::Complex;

TEST_CASE("Compelx-Vector, default construction", "[complex-vector]") {
    Vector<3, Complex> v;

    REQUIRE(v[0].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v[0].imag() == Catch::Approx(0.0f).margin(1e-6));

    REQUIRE(v[1].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v[1].imag() == Catch::Approx(0.0f).margin(1e-6));

    REQUIRE(v[2].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v[2].imag() == Catch::Approx(0.0f).margin(1e-6));
}   

TEST_CASE("Compelx-Vector, constructor", "[complex-vector]") {
    Vector<3, Complex> v(Complex(1.0f, 1.0f), Complex(4.0f, -3.0f), Complex(0.0f, -2.3f));

    REQUIRE(v[0].real() == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(v[0].imag() == Catch::Approx(1.0f).margin(1e-6));

    REQUIRE(v[1].real() == Catch::Approx(4.0f).margin(1e-6));
    REQUIRE(v[1].imag() == Catch::Approx(-3.0f).margin(1e-6));

    REQUIRE(v[2].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v[2].imag() == Catch::Approx(-2.3f).margin(1e-6));
}

TEST_CASE("Compelx-Vector, basic operations", "[complex-vector]") {
    Vector<3, Complex> v(Complex(1.0f, 1.0f), Complex(4.0f, -3.0f), Complex(0.0f, -2.3f));
    Vector<3, Complex> u(Complex(1.5f, -4.0f), Complex(2.0f, -9.5f), Complex(3.0f, 0.0f));
    Vector<3, Complex> w = v + u;

    REQUIRE(w[0].real() == Catch::Approx(2.5f).margin(1e-6));
    REQUIRE(w[0].imag() == Catch::Approx(-3.0f).margin(1e-6));
    REQUIRE(w[1].real() == Catch::Approx(6.0f).margin(1e-6));
    REQUIRE(w[1].imag() == Catch::Approx(-12.5f).margin(1e-6));
    REQUIRE(w[2].real() == Catch::Approx(3.0f).margin(1e-6));
    REQUIRE(w[2].imag() == Catch::Approx(-2.3f).margin(1e-6));

    w = v - u;

    REQUIRE(w[0].real() == Catch::Approx(-0.5f).margin(1e-6));
    REQUIRE(w[0].imag() == Catch::Approx(5.0f).margin(1e-6));
    REQUIRE(w[1].real() == Catch::Approx(2.0f).margin(1e-6));
    REQUIRE(w[1].imag() == Catch::Approx(6.5f).margin(1e-6));
    REQUIRE(w[2].real() == Catch::Approx(-3.0f).margin(1e-6));
    REQUIRE(w[2].imag() == Catch::Approx(-2.3f).margin(1e-6));

    CAPTURE(v[0].real());
    CAPTURE(v[0].imag());

    CAPTURE((v*3)[0].real());
    CAPTURE((v*3)[0].imag());

    w = v*3;

    REQUIRE(w[0].real() == Catch::Approx(3.0f).margin(1e-6));
    REQUIRE(w[0].imag() == Catch::Approx(3.0f).margin(1e-6));
    REQUIRE(w[1].real() == Catch::Approx(12.0f).margin(1e-6));
    REQUIRE(w[1].imag() == Catch::Approx(-9.0f).margin(1e-6));
    REQUIRE(w[2].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(w[2].imag() == Catch::Approx(-6.9f).margin(1e-6));
}

TEST_CASE("Compelx-Vector, misc. ops", "[complex-vector]") {
    Vector<3, Complex> v(Complex(1.0f, 1.0f), Complex(4.0f, -3.0f), Complex(0.0f, -2.3f));

    REQUIRE(v[0].real() == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(v[0].imag() == Catch::Approx(1.0f).margin(1e-6));

    REQUIRE(v[1].real() == Catch::Approx(4.0f).margin(1e-6));
    REQUIRE(v[1].imag() == Catch::Approx(-3.0f).margin(1e-6));

    REQUIRE(v[2].real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v[2].imag() == Catch::Approx(-2.3f).margin(1e-6));
}