#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/joint.hpp"
#include "cobalt/kinematics/link.hpp"
#include "cobalt/kinematics/robot_chain.hpp"

using cobalt::math::geometry::Transform;

using cobalt::math::linear_algebra::Vector;

using cobalt::kinematics::Joint;
using cobalt::kinematics::Link;
using cobalt::kinematics::RobotChain;

TEST_CASE("Kinematics, default construction", "[kinematics]") {
    Joint j(cobalt::kinematics::JointType::Revolute);
    Link leg("leg");

    REQUIRE(true);
} 

TEST_CASE("Kinematics, forward-kinematics 1R Arm", "[kinematics]") {
    RobotChain<3, 2> chain;

    chain.link(0) = Link("base", -1, 0, 0);
    chain.link(1) = Link("arm_high", 1, 1, 1);
    chain.link(2) = Link("arm_low", 1, 2, -1);

    chain.joint(0) = Joint(cobalt::kinematics::JointType::Revolute, 0, 0, 1);
    chain.joint(0).setValue(0);
    chain.joint(1) = Joint(cobalt::kinematics::JointType::Revolute, 1, 1, 2);
    chain.joint(1).setValue(M_PI_2);

    REQUIRE(chain.updateLinks());
    
    Transform<> endFrame = chain.forwardKinematics();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    chain.joint(1).setValue(-M_PI_2);

    endFrame = chain.forwardKinematics();
    
    REQUIRE(endFrame.translation()[0] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(-1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 