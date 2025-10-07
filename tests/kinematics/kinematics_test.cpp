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
    RobotChain<2, 1> chain;

    chain.link(0).setName("base");
    chain.link(0).frame() = Transform<>::eye(); // Base

    chain.link(1).setName("arm");
    chain.link(1).frame() = Transform<>::fromTranslation(Vector<3>(1.0f, 0.0f, 0.0f)); // End effector

    chain.joint(0).parentIndex() = 0;
    chain.joint(0).childIndex() = 1;
    chain.joint(0).setValue(M_PI_2);

    chain.forwardKinematics();

    Transform<> endFrame = chain.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 