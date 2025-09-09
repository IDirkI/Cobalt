#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/math/linear_algebra/vector/vector.hpp>

#include <cobalt/math/geometry/quaternion/quaternion.hpp>
#include <cobalt/math/geometry/quaternion/quaternion_ops.hpp>
#include <cobalt/math/geometry/quaternion/quaternion_util.hpp>

using cobalt::math::linear_algebra::Vector;
using cobalt::math::geometry::Quaternion;

TEST_CASE("Quaternion, default construction", "[quaternion]") {
    Quaternion q;

    REQUIRE(q.w() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(q.x() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(q.y() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(q.z() == Catch::Approx(0.0f).margin(1e-6));
}   

TEST_CASE("Quaternion, construction", "[quaternion]") {
    Quaternion q(1.0f, 5.0f, -0.2f, -0.34f);

    REQUIRE(q.w() == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(q.x() == Catch::Approx(5.0f).margin(1e-6));
    REQUIRE(q.y() == Catch::Approx(-0.2f).margin(1e-6));
    REQUIRE(q.z() == Catch::Approx(-0.34f).margin(1e-6));
}   

TEST_CASE("Quaternion, vector construction", "[quaternion]") {
    Vector<3> v{3.0f, -4.0f, 0.0f};
    Quaternion q = Quaternion::fromVector(v);

    REQUIRE(q.w() == Catch::Approx(-0.8011436f).margin(1e-6));
    REQUIRE(q.x() == Catch::Approx(0.3590833f).margin(1e-6));
    REQUIRE(q.y() == Catch::Approx(-0.4787777f).margin(1e-6));
    REQUIRE(q.z() == Catch::Approx(0.0f).margin(1e-6));

    Vector<3> u = toVector(q);

    REQUIRE(u.x() == Catch::Approx(3.0f).margin(1e-6));
    REQUIRE(u.y() == Catch::Approx(-4.0f).margin(1e-6));
    REQUIRE(u.z() == Catch::Approx(0.0f).margin(1e-6));
}   