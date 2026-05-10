#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kuka.hpp"

#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/solvers/inverse_kinematics/jacobian_builder.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"

using cobalt::kinematics::robot::makeKuka;
using cobalt::kinematics::Robot;
using cobalt::kinematics::solvers::ForwardKinematics;
using cobalt::kinematics::solvers::JacobianBuilder;

using cobalt::math::index_t;
using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Matrix;
using cobalt::math::geometry::Transform;
using cobalt::math::geometry::Quaternion;

// Kuka: 7 links, 6 joints, 1 frame
using KukaRobot = Robot<7, 6, 1>;
using KukaFK    = ForwardKinematics<7, 6, 1>;
using KukaJB    = JacobianBuilder<7, 6, 1>;

// ----------------------------------------------------------------------------
// Central-Difference Finite Difference Helper
// ----------------------------------------------------------------------------

/**
 *  @brief Numerically approximate one column of the geometric Jacobian
 *         via central differences.
 *
 *  Position column (world frame):
 *    dp/dq_j  ≈  ( p(q + eps*ej) − p(q − eps*ej) ) / (2*eps)
 *
 *  Orientation column (world frame):
 *    dω/dq_j  ≈  toRotationVector( R_plus * R_minus^-1 ) / (2*eps)
 *
 *  Note: R_plus * conj(R_minus) gives the world-frame delta rotation,
 *  matching the geometric Jacobian's angular convention. Using the reverse
 *  order gives body-frame delta, which does NOT match.
 *
 *  eps = 1e-3 balances float32 cancellation against truncation error.
 *  Central differences give O(eps^2) truncation, so 1e-3 gives ~1e-6
 *  truncation error while keeping cancellation well below 1e-3.
 */
Vector<6> numericalJacobianColumn(
    KukaRobot &robot,
    KukaFK    &fk,
    cobalt::kinematics::id_t frameId,
    cobalt::kinematics::id_t jointId,
    float eps = 1e-3f) {

    const Vector<6> q0 = robot.state().q;

    // --- Forward: q + eps ---
    Vector<6> q_plus = q0;
    q_plus[jointId] += eps;
    robot.setJoints(q_plus);
    fk.solve(robot.state());
    const Vector<3>    p_plus = robot.state().frameTransforms[frameId].translation();
    const Quaternion<> r_plus = cobalt::math::geometry::normalize(
                                    robot.state().frameTransforms[frameId].rotation());

    // --- Backward: q - eps ---
    Vector<6> q_minus = q0;
    q_minus[jointId] -= eps;
    robot.setJoints(q_minus);
    fk.solve(robot.state());
    const Vector<3>    p_minus = robot.state().frameTransforms[frameId].translation();
    const Quaternion<> r_minus = cobalt::math::geometry::normalize(
                                     robot.state().frameTransforms[frameId].rotation());

    // --- Restore ---
    robot.setJoints(q0);
    fk.solve(robot.state());

    // Position: central difference (no frame ambiguity)
    const Vector<3> dp = (p_plus - p_minus) * (1.0f / (2.0f * eps));

    // Orientation: world-frame angular velocity
    //   delta_R = R_plus * R_minus^-1  (world frame — matches geometric Jacobian)
    const Quaternion<> r_minus_conj = cobalt::math::geometry::conj(r_minus);
    const Quaternion<> delta_q      = cobalt::math::geometry::normalize(r_plus * r_minus_conj);
    const Vector<3>    dw           = cobalt::math::geometry::toRotationVector(delta_q)
                                      * (1.0f / (2.0f * eps));

    return Vector<6>{ dp[0], dp[1], dp[2], dw[0], dw[1], dw[2] };
}

// ============================================================================
//  Tests
// ============================================================================

// ----------------------------------------------------------------------------
// Construction
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, construction from robot", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaJB    jb(robot);

    REQUIRE(true);
}

// ----------------------------------------------------------------------------
// FK Validity Flags
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, FK populates joint transforms", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);

    REQUIRE_FALSE(robot.state().validJoints);

    fk.solve(robot.state());

    REQUIRE(robot.state().validLinks);
    REQUIRE(robot.state().validJoints);
    REQUIRE(robot.state().validFrames);
}

TEST_CASE("JacobianBuilder, joint transforms are unit quaternions after FK", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    fk.solve(robot.state());

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const Transform<>  &T = robot.state().jointTransforms[j];
        const Quaternion<> &q = T.rotation();

        float qNorm = std::sqrt(
            q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z()
        );

        CAPTURE(j, qNorm);
        REQUIRE_THAT(qNorm, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    }
}

// ----------------------------------------------------------------------------
// Finite Difference — Zero Configuration
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, position rows correct at zero config", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Matrix<6, 6> J = jb.compute(robot.state(), 0);

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const Vector<6> col = numericalJacobianColumn(robot, fk, 0, j);
        CAPTURE(j);
        REQUIRE_THAT(J(0, j), Catch::Matchers::WithinAbs(col[0], 2e-3f));
        REQUIRE_THAT(J(1, j), Catch::Matchers::WithinAbs(col[1], 2e-3f));
        REQUIRE_THAT(J(2, j), Catch::Matchers::WithinAbs(col[2], 2e-3f));
    }
}

TEST_CASE("JacobianBuilder, orientation rows correct at zero config", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());

    const Matrix<6, 6> J = jb.compute(robot.state(), 0);

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const Vector<6> col = numericalJacobianColumn(robot, fk, 0, j);
        CAPTURE(j);
        REQUIRE_THAT(J(3, j), Catch::Matchers::WithinAbs(col[3], 1e-3f));
        REQUIRE_THAT(J(4, j), Catch::Matchers::WithinAbs(col[4], 1e-3f));
        REQUIRE_THAT(J(5, j), Catch::Matchers::WithinAbs(col[5], 1e-3f));
    }
}

// ----------------------------------------------------------------------------
// Finite Difference — Arbitrary Configuration
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, position rows correct at arbitrary config", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>{ 0.4f, -0.7f, 1.1f, 0.3f, -0.5f, 0.2f });
    fk.solve(robot.state());

    const Matrix<6, 6> J = jb.compute(robot.state(), 0);

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const Vector<6> col = numericalJacobianColumn(robot, fk, 0, j);
        CAPTURE(j);
        REQUIRE_THAT(J(0, j), Catch::Matchers::WithinAbs(col[0], 2e-3f));
        REQUIRE_THAT(J(1, j), Catch::Matchers::WithinAbs(col[1], 2e-3f));
        REQUIRE_THAT(J(2, j), Catch::Matchers::WithinAbs(col[2], 2e-3f));
    }
}

TEST_CASE("JacobianBuilder, orientation rows correct at arbitrary config", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>{ 0.4f, -0.7f, 1.1f, 0.3f, -0.5f, 0.2f });
    fk.solve(robot.state());

    const Matrix<6, 6> J = jb.compute(robot.state(), 0);

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        const Vector<6> col = numericalJacobianColumn(robot, fk, 0, j);
        CAPTURE(j);
        REQUIRE_THAT(J(3, j), Catch::Matchers::WithinAbs(col[3], 1e-3f));
        REQUIRE_THAT(J(4, j), Catch::Matchers::WithinAbs(col[4], 1e-3f));
        REQUIRE_THAT(J(5, j), Catch::Matchers::WithinAbs(col[5], 1e-3f));
    }
}

// ----------------------------------------------------------------------------
// Consistency
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, repeated compute gives identical result", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>{ 0.1f, -0.3f, 0.6f, 0.0f, 0.2f, -0.1f });
    fk.solve(robot.state());

    const Matrix<6, 6> J1 = jb.compute(robot.state(), 0);
    const Matrix<6, 6> J2 = jb.compute(robot.state(), 0);

    for(cobalt::kinematics::id_t j = 0; j < robot.model().getJointNum(); j++) {
        for(index_t i = 0; i < 6; i++) {
            CAPTURE(i, j);
            REQUIRE_THAT(J1(i, j), Catch::Matchers::WithinAbs(J2(i, j), 1e-7f));
        }
    }
}

TEST_CASE("JacobianBuilder, result changes when configuration changes", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>::zero());
    fk.solve(robot.state());
    const Matrix<6, 6> J_zero = jb.compute(robot.state(), 0);

    robot.setJoints(Vector<6>{ 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f });
    fk.solve(robot.state());
    const Matrix<6, 6> J_moved = jb.compute(robot.state(), 0);

    bool anyDiffers = false;
    for(cobalt::kinematics::id_t j = 0; j < 6 && !anyDiffers; j++) {
        for(index_t i = 0; i < 6 && !anyDiffers; i++) {
            if(std::abs(J_zero(i, j) - J_moved(i, j)) > 1e-4f) {
                anyDiffers = true;
            }
        }
    }

    REQUIRE(anyDiffers);
}

// ----------------------------------------------------------------------------
// Singularity Inspection
// ----------------------------------------------------------------------------

TEST_CASE("JacobianBuilder, JJt is positive semi-definite", "[jacobian][kinematics]") {
    KukaRobot robot = makeKuka();
    KukaFK    fk(robot);
    KukaJB    jb(robot);

    robot.setJoints(Vector<6>{ 0.2f, -0.4f, 0.8f, 0.1f, -0.3f, 0.5f });
    fk.solve(robot.state());

    const Matrix<6, 6> J   = jb.compute(robot.state(), 0);
    const Matrix<6, 6> JJt = J * cobalt::math::linear_algebra::transpose(J);

    for(index_t i = 0; i < 6; i++) {
        CAPTURE(i, JJt(i, i));
        REQUIRE(JJt(i, i) >= -1e-6f);
    }
}