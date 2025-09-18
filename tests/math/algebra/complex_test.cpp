#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/math/algebra/complex/complex.hpp>
#include <cobalt/math/algebra/complex/complex_ops.hpp>
#include <cobalt/math/algebra/complex/complex_util.hpp>

using cobalt::math::algebra::Complex;

TEST_CASE("Complex, construction & operations", "[complex]") {
    Complex z(1, 1);
    Complex x(3, -2);
    Complex w;

    REQUIRE(z.real() == Catch::Approx(1.0f));
    REQUIRE(z.imag() == Catch::Approx(1.0f));

    REQUIRE(x.real() == Catch::Approx(3.0f));
    REQUIRE(x.imag() == Catch::Approx(-2.0f));

    REQUIRE(w.real() == Catch::Approx(0.0f));
    REQUIRE(w.imag() == Catch::Approx(0.0f));

    z += x;

    REQUIRE(z.real() == Catch::Approx(4.0f));
    REQUIRE(z.imag() == Catch::Approx(-1.0f));

    z *= 2;

    REQUIRE(z.real() == Catch::Approx(8.0f));
    REQUIRE(z.imag() == Catch::Approx(-2.0f));

    z -= x;

    REQUIRE(z.real() == Catch::Approx(5.0f));
    REQUIRE(z.imag() == Catch::Approx(0.0f));

    z += Complex::oneIm();
    z = -z;

    REQUIRE(z.real() == Catch::Approx(-5.0f));
    REQUIRE(z.imag() == Catch::Approx(-1.0f));

}   

TEST_CASE("Complex, arg/abs/conj", "[complex]") {
    Complex z(1, 1);
    Complex x(0, 4);

    REQUIRE(abs(z) == Catch::Approx(std::sqrt(2)));
    REQUIRE(arg(z) == Catch::Approx(M_PI_4));

    REQUIRE(abs(x) == Catch::Approx(4));
    REQUIRE(arg(x) == Catch::Approx(M_PI_2));

    z = conj(z);

    REQUIRE(abs(z) == Catch::Approx(std::sqrt(2)));
    REQUIRE(arg(z) == Catch::Approx(-M_PI_4));
}   

TEST_CASE("Complex, exp, log, pow", "[complex]") {
    Complex z(1, 1);
    Complex w(0, 1);

    REQUIRE(exp(z).real() == Catch::Approx(1.4686939f).margin(1e-6));
    REQUIRE(exp(z).imag() == Catch::Approx(2.28735528f).margin(1e-6));

    REQUIRE(log(z).real() == Catch::Approx(0.346573f).margin(1e-6));
    REQUIRE(log(z).imag() == Catch::Approx(0.7853981f).margin(1e-6));

    REQUIRE(pow(w, 2).real() == Catch::Approx(-1.0f).margin(1e-6));
    REQUIRE(pow(w, 2).imag() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(pow(w, 3).real() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(pow(w, 3).imag() == Catch::Approx(-1.0f).margin(1e-6));
    REQUIRE(pow(w, 4).real() == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(pow(w, 4).imag() == Catch::Approx(0.0f).margin(1e-6));
}   
