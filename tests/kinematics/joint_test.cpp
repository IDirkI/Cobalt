#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "test_robot.hpp"
#include "cobalt/kinematics/joint.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"


using cobalt::kinematics::Joint;
using cobalt::kinematics::robot::test_robot;
using cobalt::math::linear_algebra::Matrix;

TEST_CASE("Joint, default construction", "[kinematics]") {
    Joint joint = Joint();
    CAPTURE(test_robot.model().getLinks()[1].getOrigin().translation()[0]);
    CAPTURE(test_robot.model().getLinks()[1].getOrigin().translation()[1]);
    CAPTURE(test_robot.model().getLinks()[1].getOrigin().translation()[2]);

    Matrix<3,3> R = toMatrix(test_robot.model().getLinks()[1].getOrigin().rotation());

    CAPTURE(R(0,0));
    CAPTURE(R(1,0));
    CAPTURE(R(2,0));
    CAPTURE(R(0,1));
    CAPTURE(R(1,1));
    CAPTURE(R(2,1));
    CAPTURE(R(0,2));
    CAPTURE(R(1,2));
    CAPTURE(R(2,2));
    
    REQUIRE(true);
} 
