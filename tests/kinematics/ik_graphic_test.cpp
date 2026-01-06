#define _USE_MATH_DEFINES

#include <fstream>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/joint.hpp"
#include "cobalt/kinematics/link.hpp"
#include "cobalt/kinematics/robot_chain.hpp"

#include "cobalt/kinematics/robots/robot_arm.hpp"

using cobalt::math::geometry::Transform;

using cobalt::math::linear_algebra::Vector;

using cobalt::kinematics::Joint;
using cobalt::kinematics::Link;
using cobalt::kinematics::RobotChain;
using cobalt::kinematics::JointType;

using cobalt::kinematics::robot::robot_arm;

TEST_CASE("IK-graphic, default construction", "[kinematics]") {
    Joint j(cobalt::kinematics::JointType::Revolute);
    Link leg("leg");

    REQUIRE(true);
} 

TEST_CASE("IK-graphic, CSV Plotter", "[kinematics]") {
    std::ofstream log;
    log.open("../../results/kinematics/ik_arm.csv");

    Transform<> endFrame = robot_arm.endEffector();

    robot_arm.setJoints({0.0f, 0.0f});

    std::array<float, 3> q{};
    cobalt::math::linear_algebra::Vector<3> goal{-0.6f, 0.1f, 0.0f};

    size_t iter = robot_arm.inverseKinematics(q, goal, 60);
    robot_arm.setJoints(q);
    endFrame = robot_arm.endEffector();

    CAPTURE(iter);
    REQUIRE(endFrame.translation()[0] == Catch::Approx(goal[0]).margin(1e-3));
    REQUIRE(endFrame.translation()[1] == Catch::Approx(goal[1]).margin(1e-3));

    log << "x_goal, y_goal"; 
    for(int i = 0; i < robot_arm.getLinkNum(); i++) {
        log << ", x_" << i << ", y_" << i;
    }
    log << "\n";
    
    log << goal.x() << ", " << goal.y();
    for(int i = 0; i < robot_arm.getLinkNum(); i++) {
        Link l = robot_arm.link(i);
        log << ", " << l.worldFrame().translation().x() << ", " << l.worldFrame().translation().y();
    }

    log.close();
} 
