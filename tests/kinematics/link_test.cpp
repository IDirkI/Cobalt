#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/link.hpp"

using cobalt::kinematics::Link;

TEST_CASE("Link, default construction", "[kinematics]") {
    Link link = Link();
    REQUIRE(true);
} 
