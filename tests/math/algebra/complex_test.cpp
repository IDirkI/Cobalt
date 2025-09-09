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

TEST_CASE("Complex, toString", "[complex]") {
    Complex z(12, -3);
    Complex x(0, -1.5);
    Complex w(3, 0);
    Complex y(0, 0);

    REQUIRE(z.toString() == "12.000 - 3.000j");
    REQUIRE(x.toString() == "-1.500j");
    REQUIRE(w.toString() == "3.000");
    REQUIRE(y.toString() == "0");

}   
