#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/core/joint.hpp"

using cobalt::kinematics::Joint;

TEST_CASE("Joint, default construction", "[kinematics]") {
    Joint joint = Joint();
    
    REQUIRE(true);
} 
