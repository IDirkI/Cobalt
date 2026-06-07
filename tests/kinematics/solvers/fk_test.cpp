#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kuka.hpp"

#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/logging/logger_desktop.hpp"

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
using cobalt::kinematics::logging::DesktopLogger;

using cobalt::math::index_t;
using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::norm;
using cobalt::math::geometry::Transform;
using cobalt::math::geometry::Quaternion;

using KukaRobot = Robot<7, 6, 1>;
using KukaFK    = ForwardKinematics<7, 6, 1>;
using KukaLog   = DesktopLogger<7, 6, 1>;

// ============================================================================
//  Helpers
// ============================================================================

static float quaternionNorm(const Quaternion<> &q) {
    return std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
}

static void printLinkTransforms(KukaRobot &robot) {
    printf("\n  Link transforms:\n");
    for(cobalt::kinematics::id_t i = 0; i < robot.model().getLinkNum(); i++) {
        const Transform<>  &T = robot.state().linkTransforms[i];
        const Vector<3>    &p = T.translation();
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

TEST_CASE("ForwardKinematics, construction from robot", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    REQUIRE_NOTHROW([&]{ KukaFK fk(robot); }());
}

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

TEST_CASE("ForwardKinematics, base link is at identity", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Transform<> &T_base = robot.state().linkTransforms[0];

    REQUIRE_THAT(T_base.translation()[0], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(T_base.translation()[1], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(T_base.translation()[2], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(quaternionNorm(T_base.rotation()), Catch::Matchers::WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(T_base.rotation().w(), Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

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

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Vector<3> &p = robot.state().linkTransforms[1].translation();

    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(p[2], Catch::Matchers::WithinAbs(0.1f, 1e-4f));
}

TEST_CASE("ForwardKinematics, gripper frame is at correct position at zero config", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Vector<3> &p = robot.state().frameTransforms[0].translation();

    printf("\n[Zero config gripper position]\n");
    printf("  achieved : [ %.4f  %.4f  %.4f ]\n", p[0], p[1], p[2]);

    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-3f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-3f));
    REQUIRE(p[2] > 1.0f);
}

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

    const Vector<3> &p = robot.state().jointTransforms[0].translation();

    REQUIRE_THAT(p[0], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p[1], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p[2], Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ForwardKinematics, result changes when joints change", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());
    const Vector<3> p_zero = robot.state().frameTransforms[0].translation();

    robot.setJoints(Vector<6>{ 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f });
    fk.solve(robot.state());
    const Vector<3> p_moved = robot.state().frameTransforms[0].translation();

    const float dist = norm(p_moved - p_zero);
    CAPTURE(dist);
    REQUIRE(dist > 1e-3f);
}

TEST_CASE("ForwardKinematics, joint 0 rotation moves frame in XY plane", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.0f, -0.5f, 0.8f, 0.0f, 0.0f, 0.0f });
    fk.solve(robot.state());
    const Vector<3> p_base = robot.state().frameTransforms[0].translation();

    robot.setJoints(Vector<6>{ (float)M_PI / 2.0f, -0.5f, 0.8f, 0.0f, 0.0f, 0.0f });
    fk.solve(robot.state());
    const Vector<3> p_rotated = robot.state().frameTransforms[0].translation();

    printf("\n[Joint 0 rotation sensitivity]\n");
    printf("  q0=0    : [ %.4f  %.4f  %.4f ]\n", p_base[0],    p_base[1],    p_base[2]);
    printf("  q0=pi/2 : [ %.4f  %.4f  %.4f ]\n", p_rotated[0], p_rotated[1], p_rotated[2]);

    REQUIRE_THAT(p_rotated[2], Catch::Matchers::WithinAbs(p_base[2], 1e-3f));

    const float r_base    = std::sqrt(p_base[0]*p_base[0]       + p_base[1]*p_base[1]);
    const float r_rotated = std::sqrt(p_rotated[0]*p_rotated[0] + p_rotated[1]*p_rotated[1]);
    REQUIRE_THAT(r_rotated, Catch::Matchers::WithinAbs(r_base, 1e-3f));
}

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

TEST_CASE("ForwardKinematics, frame transform uses parent link not identity", "[fk][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    robot.setJoints(Vector<6>{ 0.1f, -0.4f, 0.6f, 0.0f, 0.2f, 0.0f });
    fk.solve(robot.state());

    const Vector<3> &p        = robot.state().frameTransforms[0].translation();
    const float      distFromOrigin = norm(p);

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

    KukaLog::snapshot(robot, "fk_zero_config");
    printf("\n  Logged: fk_zero_config.clog\n");

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

    KukaLog::snapshot(robot, "fk_arbitrary_config");
    printf("\n  Logged: fk_arbitrary_config.clog\n");

    REQUIRE(robot.state().validLinks);
    REQUIRE(robot.state().validJoints);
    REQUIRE(robot.state().validFrames);
}