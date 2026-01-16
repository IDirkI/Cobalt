#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "test_robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/util/robot_logger.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

using cobalt::kinematics::robot::test_robot;
using cobalt::kinematics::solvers::ForwardKinematics;
using cobalt::kinematics::util::logRobotState;

using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Matrix;
using cobalt::math::geometry::Transform;

TEST_CASE("ForwardKinematics, default construction", "[kinematics]") {
    ForwardKinematics fk = ForwardKinematics(test_robot);

    fk.solve(test_robot.state());

    logRobotState(test_robot, "test_state");

    printf(">>> Valid Link: %d\n", test_robot.state().validLinks);
    printf(">>> Valid Frame: %d\n\n", test_robot.state().validFrames);

    for (std::size_t i = 0; i < test_robot.state().linkTransforms.size(); i++) {
        const Transform<> &T = test_robot.state().linkTransforms[i];

        Matrix<3,3> R = toMatrix(T.rotation());   // Matrix<3,3>
        Vector<3> t = T.translation();            // Vector<3>

        printf("- Link %zu [%s] transform:\n", i, test_robot.model().getLinks()[i].getName().c_str());
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(0,0), R(0,1), R(0,2), t[0]);
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(1,0), R(1,1), R(1,2), t[1]);
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(2,0), R(2,1), R(2,2), t[2]);
        printf("    [  0.000000  0.000000  0.000000 |  1.000000 ]\n\n");
    }

    printf("---------------------------------------------------\n\n");

    for (std::size_t i = 0; i < test_robot.state().frameTransforms.size(); i++) {
        const Transform<> &T = test_robot.state().frameTransforms[i];

        Matrix<3,3> R = toMatrix(T.rotation());   // Matrix<3,3>
        Vector<3> t = T.translation();            // Vector<3>

        printf("- Frame %zu [%s] transform:\n", i, test_robot.model().getFrames()[i].getName().c_str());
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(0,0), R(0,1), R(0,2), t[0]);
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(1,0), R(1,1), R(1,2), t[1]);
        printf("    [ % .6f % .6f % .6f | % .6f ]\n", R(2,0), R(2,1), R(2,2), t[2]);
        printf("    [  0.000000  0.000000  0.000000 |  1.000000 ]\n\n");
    }

    REQUIRE(true);
} 
