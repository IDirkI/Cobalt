#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "kuka.hpp"
#include "arm_robot.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/util/robot_logger.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/solvers/inverse_kinematics/inverse_kinematics.hpp"
#include "cobalt/kinematics/util/robot_logger.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

using cobalt::kinematics::robot::makeKuka;
using cobalt::kinematics::robot::makeArmRobot;
using cobalt::kinematics::Robot;
using cobalt::kinematics::solvers::ForwardKinematics;
using cobalt::kinematics::solvers::InverseKinematics;
using cobalt::kinematics::solvers::IKTarget;
using cobalt::kinematics::solvers::IKMode;
using cobalt::kinematics::solvers::IKSolution;
using cobalt::kinematics::util::logRobotState;

using cobalt::math::index_t;
using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Matrix;
using cobalt::math::geometry::Transform;
using cobalt::math::geometry::Quaternion;

TEST_CASE("InverseKinematics, default construction", "[kinematics]") {
    Robot test_robot = makeKuka();
    
    InverseKinematics ik(test_robot);
    
    IKTarget target {
        0,
        IKMode::Pose,
        Transform<>(Quaternion<>::fromEuler(M_PI_4, M_PI, 0), Vector<3>(0.9,1.1,0.3)),
        {1, 1, 1, 0.01, 0.01, 0.01}
    };

    IKSolution sol = ik.solve(target);

    printf(">> w = [ ");
    for(int i = 0; i < 6; i++) {
        if(i != 5) { printf("%3.4f, ", target.weight[i]); }
        else { printf("%3.4f ]\n", target.weight[i]); }
    }
    printf(">> q = [ ");
    for(int i = 0; i < test_robot.model().getJointNum(); i++) {
        if(i != test_robot.model().getJointNum()-1) { printf("%3.4f, ", sol.q[i]); }
        else { printf("%3.4f ]\n", sol.q[i]); }
    }
    printf(">> err = [ %3.4f, %3.4f, %3.4f, %3.4f, %3.4f, %3.4f ]\n", sol.error[0], sol.error[1], sol.error[2], sol.error[3], sol.error[4], sol.error[5]);
    printf(">> iterations = %d\n", sol.iterations);
    printf(">> status = %d\n", static_cast<int>(sol.status));

    test_robot.setJoints(sol.q);
    

    ForwardKinematics fk = ForwardKinematics(test_robot);
    fk.solve(test_robot.state());

    logRobotState(test_robot, "test_state");

    REQUIRE(true);
} 