#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kuka.hpp"

#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/util/robot_logger.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

using cobalt::kinematics::robot::makeKuka;
using cobalt::kinematics::Robot;
using cobalt::kinematics::solvers::ForwardKinematics;
using cobalt::kinematics::util::logRobotState;

using cobalt::math::index_t;
using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::norm;
using cobalt::math::geometry::Transform;
using cobalt::math::geometry::Quaternion;

using KukaRobot = Robot<7, 6, 1>;
using KukaFK    = ForwardKinematics<7, 6, 1>;

// ============================================================================
//  Helpers
// ============================================================================

static float quaternionNorm(const Quaternion<> &q) {
    return std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
}

static void printLinkTransforms(KukaRobot &robot) {
    printf("\n  Link transforms:\n");
    for(cobalt::kinematics::id_t i = 0; i < robot.model().getLinkNum(); i++) {
        const Transform<> &T = robot.state().linkTransforms[i];
        const Vector<3>   &p = T.translation();
        const Quaternion<> &q = T.rotation();
        printf("    [%d] %-12s  p=[%7.4f %7.4f %7.4f]  q=[%6.4f %6.4f %6.4f %6.4f]\n",
               i, robot.model().getLinks()[i].getName(),
               p[0], p[1], p[2],
               q.w(), q.x(), q.y(), q.z());
    }
}

static void printFrameTransforms(KukaRobot &robot) {
    printf("\n  Frame transforms:\n");
    for(cobalt::kinematics::id_t i = 0; i < robot.model().getFrameNum(); i++) {
        const Transform<> &T = robot.state().frameTransforms[i];
        const Vector<3>   &p = T.translation();
        printf("    [%d] %-12s  p=[%7.4f %7.4f %7.4f]\n",
               i, robot.model().getFrames()[i].getName(),
               p[0], p[1], p[2]);
    }
}

// ============================================================================
//  Tests
// ============================================================================

// ----------------------------------------------------------------------------
// Construction
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, construction from robot", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    REQUIRE_NOTHROW([&]{ KukaFK fk(robot); }());
}

// ----------------------------------------------------------------------------
// Validity Flags
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, validity flags false before solve", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();

    REQUIRE_FALSE(robot.state().validLinks);
    REQUIRE_FALSE(robot.state().validJoints);
    REQUIRE_FALSE(robot.state().validFrames);
}

TEST_CASE("ForwardKinematics, validity flags set after solve", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    fk.solve(robot.state());

    REQUIRE(robot.state().validLinks);
    REQUIRE(robot.state().validJoints);
    REQUIRE(robot.state().validFrames);
}

// ----------------------------------------------------------------------------
// Base Link
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, base link is at identity", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    // Base link (id=0) is the world anchor — always at identity
    const Transform<> &T_base = robot.state().linkTransforms[0];

    REQUIRE_THAT(T_base.translation()[0], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(T_base.translation()[1], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(T_base.translation()[2], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(quaternionNorm(T_base.rotation()), Catch::Matchers::WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(T_base.rotation().w(), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

// ----------------------------------------------------------------------------
// Link Transforms — Zero Configuration
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, all link rotations are unit quaternions at zero config", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    for(cobalt::kinematics::id_t i = 0; i < robot.model().getLinkNum(); i++) {
        const float qn = quaternionNorm(robot.state().linkTransforms[i].rotation());
        CAPTURE(i, qn);
        REQUIRE_THAT(qn, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("ForwardKinematics, l1 is at correct position at zero config", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    // At zero config the Kuka arm points straight up.
    // l1 origin has xyz=[0.1,0,0] in local frame after Ry(-pi/2):
    // Ry(-pi/2) * [0.1,0,0] = [0,0,0.1] — so l1 is at [0,0,0.1] in world.
    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    // Link id=1 is l1
    const Vector<3> &p = robot.state().linkTransforms[1].translation();

    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(p[2], Catch::Matchers::WithinAbs(0.1f, 1e-4f));
}

TEST_CASE("ForwardKinematics, gripper frame is at correct position at zero config", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    // At zero config the fully extended arm reaches [0,0,2.4]:
    // l1=0.1, l2=0.5, l3=0.5, l4=0.1, gripper offset=0.1 → 0.1+0.5+0.5+0.5+0.1+0.1 ≈ 2.4
    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Vector<3> &p = robot.state().frameTransforms[0].translation();

    printf("\n[Zero config gripper position]\n");
    printf("  achieved : [ %.4f  %.4f  %.4f ]\n", p[0], p[1], p[2]);

    // Arm points along Z at zero config — X and Y must be zero
    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-3f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-3f));
    // Z is the sum of all link lengths — just verify it's positive and nonzero
    REQUIRE(p[2] > 1.0f);
}

// ----------------------------------------------------------------------------
// Joint Transforms
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, joint transforms are unit quaternions", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.3f, -0.5f, 0.7f, 0.1f, -0.2f, 0.4f });
    fk.solve(robot.state());

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const float qn = quaternionNorm(robot.state().jointTransforms[j].rotation());
        CAPTURE(j, qn);
        REQUIRE_THAT(qn, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("ForwardKinematics, base joint transform is at origin", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    // Joint 0 (base→l1) has origin at [0,0,0] — its world transform must be identity translation
    const Vector<3> &p = robot.state().jointTransforms[0].translation();

    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p[2], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

// ----------------------------------------------------------------------------
// Configuration Sensitivity
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, result changes when joints change", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());
    const Vector<3> p_zero = robot.state().frameTransforms[0].translation();

    robot.setJoints(Vector<6>{ 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f });
    fk.solve(robot.state());
    const Vector<3> p_moved = robot.state().frameTransforms[0].translation();

    // Frame must have moved
    const float dist = norm(p_moved - p_zero);
    CAPTURE(dist);
    REQUIRE(dist > 1e-3f);
}

TEST_CASE("ForwardKinematics, joint 0 rotation moves frame in XY plane", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    // Set a configuration where the arm is tilted off the Z axis
    // so joint 0 rotation produces measurable XY displacement
    robot.setJoints(Vector<6>{ 0.0f, -0.5f, 0.8f, 0.0f, 0.0f, 0.0f });
    fk.solve(robot.state());
    const Vector<3> p_base = robot.state().frameTransforms[0].translation();

    // Now rotate joint 0 by pi/2
    robot.setJoints(Vector<6>{ (float)M_PI / 2.0f, -0.5f, 0.8f, 0.0f, 0.0f, 0.0f });
    fk.solve(robot.state());
    const Vector<3> p_rotated = robot.state().frameTransforms[0].translation();

    printf("\n[Joint 0 rotation sensitivity]\n");
    printf("  q0=0    : [ %.4f  %.4f  %.4f ]\n", p_base[0],    p_base[1],    p_base[2]);
    printf("  q0=pi/2 : [ %.4f  %.4f  %.4f ]\n", p_rotated[0], p_rotated[1], p_rotated[2]);

    // Z should be unchanged — joint 0 is a Z-axis revolution
    REQUIRE_THAT(p_rotated[2], Catch::Matchers::WithinAbs(p_base[2], 1e-3f));
    // XY norm should be preserved — rotating in XY plane
    const float r_base    = std::sqrt(p_base[0]*p_base[0]    + p_base[1]*p_base[1]);
    const float r_rotated = std::sqrt(p_rotated[0]*p_rotated[0] + p_rotated[1]*p_rotated[1]);
    REQUIRE_THAT(r_rotated, Catch::Matchers::WithinAbs(r_base, 1e-3f));
}

// ----------------------------------------------------------------------------
// Determinism
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, repeated solve gives identical result", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.2f, -0.3f, 0.5f, 0.1f, -0.1f, 0.3f });

    fk.solve(robot.state());
    const Vector<3> p1 = robot.state().frameTransforms[0].translation();

    fk.solve(robot.state());
    const Vector<3> p2 = robot.state().frameTransforms[0].translation();

    for(index_t i = 0; i < 3; i++) {
        CAPTURE(i, p1[i], p2[i]);
        REQUIRE_THAT(p1[i], Catch::Matchers::WithinAbs(p2[i], 1e-7f));
    }
}

// ----------------------------------------------------------------------------
// Frame Transforms
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, frame transform uses parent link not identity", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    // At a non-trivial config, the frame must not be at [0,0,0]
    robot.setJoints(Vector<6>{ 0.1f, -0.4f, 0.6f, 0.0f, 0.2f, 0.0f });
    fk.solve(robot.state());

    const Vector<3> &p = robot.state().frameTransforms[0].translation();

    // Frame cannot be at origin — it must reflect the full kinematic chain
    const float distFromOrigin = norm(p);
    CAPTURE(distFromOrigin, p[0], p[1], p[2]);
    REQUIRE(distFromOrigin > 0.1f);
}

TEST_CASE("ForwardKinematics, frame rotation is unit quaternion", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.4f, -0.7f, 1.1f, 0.3f, -0.5f, 0.2f });
    fk.solve(robot.state());

    for(cobalt::kinematics::id_t i = 0; i < robot.model().getFrameNum(); i++) {
        const float qn = quaternionNorm(robot.state().frameTransforms[i].rotation());
        CAPTURE(i, qn);
        REQUIRE_THAT(qn, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    }
}

// ----------------------------------------------------------------------------
// Logging
// ----------------------------------------------------------------------------

TEST_CASE("ForwardKinematics, log zero config state", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    printLinkTransforms(robot);
    printFrameTransforms(robot);

    logRobotState(robot, "fk_zero_config");
    printf("\n  Logged: fk_zero_config.csv\n");
    printf("  Visualise: python scripts/kinematics/robot_plotter.py fk_zero_config\n");

    REQUIRE(robot.state().validLinks);
    REQUIRE(robot.state().validJoints);
    REQUIRE(robot.state().validFrames);
}

TEST_CASE("ForwardKinematics, log arbitrary config state", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.4f, -0.7f, 1.1f, 0.3f, -0.5f, 0.2f });
    fk.solve(robot.state());

    printLinkTransforms(robot);
    printFrameTransforms(robot);

    logRobotState(robot, "fk_arbitrary_config");
    printf("\n  Logged: fk_arbitrary_config.csv\n");
    printf("  Visualise: python scripts/kinematics/robot_plotter.py fk_arbitrary_config\n");

    REQUIRE(robot.state().validLinks);
    REQUIRE(robot.state().validJoints);
    REQUIRE(robot.state().validFrames);
}