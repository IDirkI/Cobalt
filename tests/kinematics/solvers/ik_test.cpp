#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kuka.hpp"

#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/solvers/inverse_kinematics/inverse_kinematics.hpp"
#include "cobalt/kinematics/util/robot_logger.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

using cobalt::kinematics::robot::makeKuka;
using cobalt::kinematics::Robot;
using cobalt::kinematics::solvers::ForwardKinematics;
using cobalt::kinematics::solvers::InverseKinematics;
using cobalt::kinematics::solvers::IKTarget;
using cobalt::kinematics::solvers::IKMode;
using cobalt::kinematics::solvers::IKConfig;
using cobalt::kinematics::solvers::IKSolver;
using cobalt::kinematics::solvers::IKSolution;
using cobalt::kinematics::solvers::IKStatus;
using cobalt::kinematics::util::logRobotState;

using cobalt::math::linear_algebra::Vector;
using cobalt::math::geometry::Transform;
using cobalt::math::geometry::Quaternion;

using KukaRobot = Robot<7, 6, 1>;
using KukaFK    = ForwardKinematics<7, 6, 1>;
using KukaIK    = InverseKinematics<7, 6, 1>;

// ============================================================================
//  Helpers
// ============================================================================

static float positionError(const IKSolution<6> &sol) {
    return std::sqrt(sol.err[0]*sol.err[0]
                   + sol.err[1]*sol.err[1]
                   + sol.err[2]*sol.err[2]);
}

static float orientationError(const IKSolution<6> &sol) {
    return std::sqrt(sol.err[3]*sol.err[3]
                   + sol.err[4]*sol.err[4]
                   + sol.err[5]*sol.err[5]);
}

static void printSolution(const IKSolution<6> &sol, const char *label) {
    const char *statusStr = "???";
    switch(sol.status) {
        case IKStatus::Success:       statusStr = "Success";       break;
        case IKStatus::MaxIterations: statusStr = "MaxIterations"; break;
        case IKStatus::Unreachable:   statusStr = "Unreachable";   break;
        case IKStatus::Singular:      statusStr = "Singular";      break;
        case IKStatus::InvalidInput:  statusStr = "InvalidInput";  break;
    }

    printf("\n[%s]\n", label);
    printf("  status     : %s\n", statusStr);
    printf("  iterations : %d\n", sol.iterations);
    printf("  q          : [ ");
    for(int i = 0; i < 6; i++) { printf("%.4f ", sol.q[i]); }
    printf("]\n");
    printf("  pos_err    : %.6f m\n",   positionError(sol));
    printf("  rot_err    : %.6f rad\n", orientationError(sol));
}

static void applyAndLog(KukaRobot &robot, KukaFK &fk,
                        const IKSolution<6> &sol, const char *filename) {
    robot.setJoints(sol.q);
    fk.solve(robot.state());

    const Vector<3> &p = robot.state().frameTransforms[0].translation();
    printf("  achieved   : [ %.4f  %.4f  %.4f ]\n", p[0], p[1], p[2]);

    logRobotState(robot, filename);
    printf("  logged     : %s.csv\n", filename);
}

static IKConfig makeDLSConfig() {
    return IKConfig{
        .solver        = IKSolver::DLS,
        .maxIterations = 100,
        .threshold     = 1e-3f,
        .step          = 1.0f,
        .dampingDLS    = 1e-2f,
        .dampingMin    = 1e-5f,
        .dampingMax    = 1e-1f,
        .projectMargin = 0.05f,
    };
}

static IKConfig makeSVDConfig() {
    IKConfig c = makeDLSConfig();
    c.solver = IKSolver::SVD;
    return c;
}

// ============================================================================
//  Tests
// ============================================================================

// ----------------------------------------------------------------------------
// Position IK — reachable targets
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, position IK converges on reachable target", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);
    KukaIK ik(robot, makeDLSConfig());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.0f, 0.0f, 1.5f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Position IK — straight up");

    REQUIRE(sol.status == IKStatus::Success);
    REQUIRE_THAT(positionError(sol), Catch::Matchers::WithinAbs(0.0f, 1e-3f));

    applyAndLog(robot, fk, sol, "ik_position_straight");
}

TEST_CASE("InverseKinematics, position IK converges at lateral target", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);
    KukaIK ik(robot, makeDLSConfig());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.3f, 0.2f, 0.8f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Position IK — lateral offset");

    REQUIRE((sol.status == IKStatus::Success || sol.status == IKStatus::MaxIterations));
    REQUIRE_THAT(positionError(sol), Catch::Matchers::WithinAbs(0.0f, 5e-3f));

    applyAndLog(robot, fk, sol, "ik_position_lateral");
}

// ----------------------------------------------------------------------------
// Pose IK — full position + orientation
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, pose IK converges with DLS", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);
    KukaIK ik(robot, makeDLSConfig());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Pose,
        .pose    = Transform<>(
            Quaternion<>::fromAxisAngle(Vector<3>{ 1.0f, 0.0f, 0.0f }, (float)M_PI / 4.0f),
            Vector<3>{ 0.0f, 0.2f, 0.9f }
        ),
        .weight = { 1, 1, 1, 0.3f, 0.3f, 0.3f },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Pose IK — DLS");

    REQUIRE((sol.status == IKStatus::Success || sol.status == IKStatus::MaxIterations));
    REQUIRE_THAT(positionError(sol), Catch::Matchers::WithinAbs(0.0f, 5e-3f));

    applyAndLog(robot, fk, sol, "ik_pose_dls");
}

TEST_CASE("InverseKinematics, pose IK converges with SVD", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);
    KukaIK ik(robot, makeSVDConfig());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Pose,
        .pose    = Transform<>(
            Quaternion<>::fromAxisAngle(Vector<3>{ 0.0f, 1.0f, 0.0f }, (float)M_PI / 6.0f),
            Vector<3>{ 0.1f, 0.0f, 1.0f }
        ),
        .weight = { 1, 1, 1, 0.3f, 0.3f, 0.3f },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Pose IK — SVD");

    REQUIRE((sol.status == IKStatus::Success || sol.status == IKStatus::MaxIterations));
    REQUIRE_THAT(positionError(sol), Catch::Matchers::WithinAbs(0.0f, 5e-3f));

    applyAndLog(robot, fk, sol, "ik_pose_svd");
}

// ----------------------------------------------------------------------------
// Unreachable target
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, reports unreachable for out-of-workspace target", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaIK ik(robot, makeDLSConfig());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.0f, 0.0f, 99.0f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Unreachable target");

    REQUIRE(sol.status == IKStatus::Unreachable);
}

// ----------------------------------------------------------------------------
// Invalid input
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, reports invalid input for bad frame ID", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaIK ik(robot, makeDLSConfig());

    IKTarget target{
        .frameId = 99,
        .mode    = IKMode::Pose,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.0f, 0.0f, 1.0f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol = ik.solve(target);
    printSolution(sol, "Invalid frame ID");

    REQUIRE(sol.status == IKStatus::InvalidInput);
}

// ----------------------------------------------------------------------------
// State is non-destructive
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, robot state restored after solve", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);
    KukaIK ik(robot, makeDLSConfig());

    const Vector<6> qBefore{ 0.2f, -0.4f, 0.6f, 0.1f, -0.2f, 0.3f };
    robot.setJoints(qBefore);
    fk.solve(robot.state());

    IKTarget target{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.1f, 0.1f, 1.2f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    ik.solve(target);

    const Vector<6> qAfter = robot.state().q;

    printf("\n[State restoration]\n");
    printf("  qBefore : [ ");
    for(int i = 0; i < 6; i++) { printf("%.4f ", qBefore[i]); }
    printf("]\n  qAfter  : [ ");
    for(int i = 0; i < 6; i++) { printf("%.4f ", qAfter[i]); }
    printf("]\n");

    for(int i = 0; i < 6; i++) {
        CAPTURE(i, qBefore[i], qAfter[i]);
        REQUIRE_THAT(qAfter[i], Catch::Matchers::WithinAbs(qBefore[i], 1e-6f));
    }
}

// ----------------------------------------------------------------------------
// Warm-starting
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, warm starting reduces iterations on nearby target", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    IKConfig config = makeDLSConfig();
    config.maxIterations = 50;
    KukaIK ik(robot, config);

    IKTarget t1{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.05f, 0.0f, 1.2f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol1 = ik.solve(t1);

    printf("\n[Warm-starting]\n");
    printf("  cold solve : %d iterations  pos_err=%.6f\n",
           sol1.iterations, positionError(sol1));

    robot.setJoints(sol1.q);
    fk.solve(robot.state());
    applyAndLog(robot, fk, sol1, "ik_warmstart_t1");

    IKTarget t2{
        .frameId = 0,
        .mode    = IKMode::Position,
        .pose    = Transform<>(Quaternion<>::eye(), Vector<3>{ 0.06f, 0.0f, 1.2f }),
        .weight  = { 1, 1, 1, 1, 1, 1 },
    };

    IKSolution<6> sol2 = ik.solve(t2);
    printf("  warm solve : %d iterations  pos_err=%.6f\n",
           sol2.iterations, positionError(sol2));

    applyAndLog(robot, fk, sol2, "ik_warmstart_t2");

    REQUIRE((sol1.status == IKStatus::Success || sol1.status == IKStatus::MaxIterations));
    REQUIRE((sol2.status == IKStatus::Success || sol2.status == IKStatus::MaxIterations));
    REQUIRE(sol2.iterations <= sol1.iterations);
}

// ----------------------------------------------------------------------------
// Arc trajectory — logging each waypoint for visualisation
// ----------------------------------------------------------------------------

TEST_CASE("InverseKinematics, arc trajectory with per-waypoint logging", "[ik][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK fk(robot);

    IKConfig config = makeDLSConfig();
    config.maxIterations = 50;
    KukaIK ik(robot, config);

    // Arc centered at [0, 0, 1.0] with radius 0.3 — stays well inside working zone.
    // Sweeps from [0, 0, 1.3] (straight up) to [0.3, 0, 1.0] (forward reach).
    constexpr float cx         = 0.0f;
    constexpr float cz         = 1.0f;
    constexpr float r          = 0.3f;
    constexpr int   nWaypoints = 8;

    printf("\n[Arc trajectory — %d waypoints, center=[%.1f,0,%.1f] r=%.2f]\n",
           nWaypoints, cx, cz, r);
    printf("  wp   status        iters   pos_err(m)   x        y        z\n");
    printf("  --   ------        -----   ----------   ------   ------   ------\n");

    for(int wp = 0; wp < nWaypoints; wp++) {
        const float angle = (float)wp / (float)(nWaypoints - 1) * (float)M_PI / 2.0f;

        IKTarget target{
            .frameId = 0,
            .mode    = IKMode::Position,
            .pose    = Transform<>(
                Quaternion<>::eye(),
                Vector<3>{ cx + r * std::sin(angle), 0.0f, cz + r * std::cos(angle) }
            ),
            .weight = { 1, 1, 1, 1, 1, 1 },
        };

        IKSolution<6> sol = ik.solve(target);

        const char *statusStr = "???          ";
        switch(sol.status) {
            case IKStatus::Success:       statusStr = "Success      "; break;
            case IKStatus::MaxIterations: statusStr = "MaxIterations"; break;
            case IKStatus::Unreachable:   statusStr = "Unreachable  "; break;
            case IKStatus::Singular:      statusStr = "Singular     "; break;
            case IKStatus::InvalidInput:  statusStr = "InvalidInput "; break;
        }

        robot.setJoints(sol.q);
        fk.solve(robot.state());
        const Vector<3> &p = robot.state().frameTransforms[0].translation();
        const float posErr = positionError(sol);

        printf("  %2d   %s  %3d     %.6f   %.4f   %.4f   %.4f\n",
               wp, statusStr, sol.iterations, posErr, p[0], p[1], p[2]);

        char filename[64];
        std::snprintf(filename, sizeof(filename), "ik_trajectory_wp%d", wp);
        logRobotState(robot, filename);

        CAPTURE(wp, p[0], p[1], p[2]);
        REQUIRE((sol.status == IKStatus::Success || sol.status == IKStatus::MaxIterations));
        REQUIRE_THAT(posErr, Catch::Matchers::WithinAbs(0.0f, 5e-3f));
    }

    printf("\n  Visualise any waypoint:\n");
    printf("    python scripts/kinematics/robot_plotter.py ik_trajectory_wp0\n");
}