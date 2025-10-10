#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/joint.hpp"
#include "cobalt/kinematics/link.hpp"
#include "cobalt/kinematics/robot_chain.hpp"

#include "cobalt/kinematics/robots/robot_dog.hpp"
#include "cobalt/kinematics/robots/arm_2r.hpp"

using cobalt::math::geometry::Transform;

using cobalt::math::linear_algebra::Vector;

using cobalt::kinematics::Joint;
using cobalt::kinematics::Link;
using cobalt::kinematics::RobotChain;
using cobalt::kinematics::JointType;

using cobalt::kinematics::robot::robot_dog;
using cobalt::kinematics::robot::arm_2r;

TEST_CASE("Kinematics, default construction", "[kinematics]") {
    Joint j(cobalt::kinematics::JointType::Revolute);
    Link leg("leg");

    REQUIRE(true);
} 

TEST_CASE("Kinematics, Link getters-setters", "[kinematics]") {
    Link link("leg", 0.3, 0, 2, 0.5);
    REQUIRE(link.getName() == "leg");
    REQUIRE(link.getLength() == Catch::Approx(0.3f).margin(1e-6));
    REQUIRE(link.getId() == 0);
    REQUIRE(link.getChild() == 2);
    REQUIRE(link.getMass() == Catch::Approx(0.5f).margin(1e-6));

    link.setName("tail");
    REQUIRE(link.getName() == "tail");
    link.setLength(0.4);
    REQUIRE(link.getLength() == Catch::Approx(0.4f).margin(1e-6));
    link.setId(2);
    REQUIRE(link.getId() == 2);
    link.setChild(1);
    REQUIRE(link.getChild() == 1);
    link.setMass(1.2f);
    REQUIRE(link.getMass() == Catch::Approx(1.2f).margin(1e-6));
} 

TEST_CASE("Kinematics, Joint getters-setters", "[kinematics]") {
    Joint joint(JointType::Revolute, 0, 2, 3, cobalt::math::linear_algebra::Vector<3>{0.6, 0.8, 0}, -M_PI_2, M_PI_2, 0, 0);
    REQUIRE(joint.getType() == JointType::Revolute);
    REQUIRE(joint.getId() == 0);
    REQUIRE(joint.getParent() == 2);
    REQUIRE(joint.getChild() == 3);
    REQUIRE(joint.getAxis() == cobalt::math::linear_algebra::Vector<3>{0.6, 0.8, 0});
    REQUIRE(joint.getMinLimit() == Catch::Approx(-M_PI_2).margin(1e-6));
    REQUIRE(joint.getMaxLimit() == Catch::Approx(M_PI_2).margin(1e-6));
    REQUIRE(joint.getValue() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(joint.getHome() == Catch::Approx(0.0f).margin(1e-6));

    joint.setType(JointType::Prismatic);
    REQUIRE(joint.getType() == JointType::Prismatic);
    joint.setId(1);
    REQUIRE(joint.getId() == 1);
    joint.setParent(1);
    REQUIRE(joint.getParent() == 1);
    joint.setChild(2);
    REQUIRE(joint.getChild() == 2);
    joint.setAxis(cobalt::math::linear_algebra::Vector<3>{0.0f, 0.0f, -1.0f});
    REQUIRE(joint.getAxis() == cobalt::math::linear_algebra::Vector<3>{0.0f, 0.0f, -1.0f});
    joint.setLimits(-M_PI_4, M_PI_4);
    REQUIRE(joint.getMinLimit() == Catch::Approx(-M_PI_4).margin(1e-6));
    REQUIRE(joint.getMaxLimit() == Catch::Approx(M_PI_4).margin(1e-6));
    joint.setHome(M_PI_2);
    REQUIRE(joint.getHome() == Catch::Approx(M_PI_2).margin(1e-6));
    REQUIRE(joint.getValue() == Catch::Approx(M_PI_2).margin(1e-6));
    joint.setValue(M_PI/6);
    REQUIRE(joint.getValue() == Catch::Approx(M_PI_2 + M_PI/6).margin(1e-6));
} 

TEST_CASE("Kinematics, forward-kinematics 1R Arm", "[kinematics]") {
    RobotChain<2, 1> chain;

    chain.link(0) = Link("base", -1, 0, 0);
    chain.link(1) = Link("arm_high", 1, 1, -1);

    chain.joint(0) = Joint(cobalt::kinematics::JointType::Revolute, 0, 0, 1);

    REQUIRE(chain.findLinks());

    chain.setJoint(0, M_PI_2);
    
    Transform<> endFrame = chain.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 

TEST_CASE("Kinematics, forward-kinematics 2R Arm", "[kinematics]") {
    RobotChain<3, 2> chain;

    chain.link(0) = Link("base", -1, 0, 0);
    chain.link(1) = Link("arm_high", 1, 1, 1);
    chain.link(2) = Link("arm_low", 1, 2, -1);

    chain.joint(0) = Joint(cobalt::kinematics::JointType::Revolute, 0, 0, 1);
    
    chain.joint(1) = Joint(cobalt::kinematics::JointType::Revolute, 1, 1, 2);
    REQUIRE(chain.findLinks());

    chain.setJoints({0.0f, M_PI_2});

    Transform<> endFrame = chain.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    chain.setJoint(1, -M_PI_2);

    endFrame = chain.endEffector();
    
    REQUIRE(endFrame.translation()[0] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(-1.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 

TEST_CASE("Kinematics, ROB parsers & forward-kinematics 2R Leg", "[kinematics]") {
    std::array<Link, 3> robot_dog_links = {
      Link("base", -1.0f, 0, 0, 0.0f),
      Link("dog/leg1_high", 0.3f, 1, 1, 0.4f),
      Link("dog/leg1_low", 0.2f, 2, -1, 0.3f),
    };

    std::array<Joint, 2> robot_dog_joints = {
      Joint(cobalt::kinematics::JointType::Revolute, 0, 0, 1,
            cobalt::math::linear_algebra::Vector<3>(0.0f, 0.0f, -1.0f),
            -1.5707963267948966f, 1.5707963267948966f, 0.0f, 0.0f),
      Joint(cobalt::kinematics::JointType::Revolute, 1, 1, 2,
            cobalt::math::linear_algebra::Vector<3>(0.0f, 0.0f, -1.0f),
            -1.5707963267948966f, 1.5707963267948966f, 1.5707963267948966f, 0.0f),
    };

    RobotChain<3, 2> robot_dog(robot_dog_links, robot_dog_joints);

    Transform<> endFrame = robot_dog.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.3f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(-0.2f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    REQUIRE(robot_dog.setJoint(0, M_PI_4));
    endFrame = robot_dog.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.0707106781f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(-0.3535533906f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 

TEST_CASE("Kinematics, ROB parsers & forward-kinematics 2R Leg with home-offset", "[kinematics]") {
    Transform<> endFrame = robot_dog.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.3f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(-0.2f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    robot_dog.setJoint(1, -M_PI_2);
    endFrame = robot_dog.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.5f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    robot_dog.setJoints({-M_PI_2, 0});
    endFrame = robot_dog.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(0.2f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(0.3f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));
} 

TEST_CASE("Kinematics, IK with 2R Arm", "[kinematics]") {
    Transform<> endFrame = arm_2r.endEffector();

    REQUIRE(endFrame.translation()[0] == Catch::Approx(2.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(endFrame.translation()[2] == Catch::Approx(0.0f).margin(1e-6));

    arm_2r.setJoints({0.0f, 0.0f});

    std::array<float, 2> q{};
    cobalt::math::linear_algebra::Vector<3> goal{0.2f, 0.0f, 0.0f};

    size_t iter = arm_2r.inverseKinematics(q, goal, 60);
    arm_2r.setJoints(q);
    endFrame = arm_2r.endEffector();

    CAPTURE(iter);
    REQUIRE(endFrame.translation()[0] == Catch::Approx(goal[0]).margin(1e-2));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(goal[1]).margin(1e-2));
} 