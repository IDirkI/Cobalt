#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/joint.hpp"
#include "cobalt/kinematics/link.hpp"

using cobalt::kinematics::Joint;
using cobalt::kinematics::Link;

TEST_CASE("Kinematics, default construction", "[kinematics]") {
    Joint j(cobalt::kinematics::JointType::Revolute);
    Link leg;

    REQUIRE(true);
} 