#pragma once

#include <stdio.h>

#include "jacobian_builder.hpp"

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"
#include "cobalt/kinematics/state/robot_state.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics::solvers {

constexpr iter_t IK_SINGULAR_THRESHOLD = 100;

constexpr iter_t IK_DEFAULT_MAX_ITERATIONS = 200;
constexpr float IK_DEFAULT_THRESHOLD = 1e-3;
constexpr float IK_DEFAULT_DAMPING = 1e-2;
constexpr float IK_DEFAULT_STEP = 0.2f;
constexpr float IK_DEFAULT_MARGIN = 0.05f;

enum class IKStatus : uint8_t {
    Success,
    Unreachable,
    MaxIterations,
    Singular,
};

enum class IKMode : uint8_t {
    Position,
    Orientation,
    Pose,
};

/**
 *  @brief Frames goal position/orientation/pose after the IK process
 */
struct IKTarget {
    id_t frameId;
    cobalt::math::geometry::Transform<> pose;
    IKMode mode;
};

/**
 *  @brief Best joint configuration for desierd IKTargets
 */
template<id_t nJ>
struct IKSolution {
    cobalt::math::linear_algebra::Vector<nJ> q;
    cobalt::math::linear_algebra::Vector<6> error;
    iter_t iterations;
    IKStatus status;
};

// --------------------------------------
//           Inverse Kinematics
// --------------------------------------
/**
 *  @brief IK solver for a robot made up of RobotModel and RobotState
 */
template<id_t nL, id_t nJ, id_t nE>
class InverseKinematics {   
    private:
        Robot<nL, nJ, nE> &robot_;
        JacobianBuilder<nL, nJ, nE> jacobian_;
        ForwardKinematics<nL, nJ, nE> fk_;

        iter_t maxIterations_{IK_DEFAULT_MAX_ITERATIONS};
        float threshold_{IK_DEFAULT_THRESHOLD};
        float damping_{IK_DEFAULT_DAMPING};
        float step_{IK_DEFAULT_STEP};
        float projectMargin_{IK_DEFAULT_STEP};

        // ---------------- Helper Functions ----------------
        /**
         *  @brief Compute the current error of the robot state vs the target destination
         */
        inline cobalt::math::linear_algebra::Vector<3> computePosError(const IKTarget &target, const cobalt::math::geometry::Transform<> &T_frame) {
            return target.pose.translation() - T_frame.translation();
        }

        /**
         *  @brief Compute the damped psuedo-inverse of the position jacobian
         */
        inline cobalt::math::linear_algebra::Matrix<nJ, 3> computePosPseudoInv(const cobalt::math::linear_algebra::Matrix<3, nJ> &J, float errNorm) {
            float lambda = damping_*(1.0f + errNorm);

            cobalt::math::linear_algebra::Matrix<3, 3> JJt= J * transpose(J);

            cobalt::math::linear_algebra::Matrix<3, 3> A = JJt + cobalt::math::linear_algebra::Matrix<3, 3>::eye()*(lambda*lambda);
            cobalt::math::linear_algebra::Matrix<3, 3> Ainv;
            bool success = inv(A, Ainv);

            cobalt::math::linear_algebra::Matrix<nJ, 3> Jpinv = transpose(J) * Ainv;
            return Jpinv;
        }

        /**
         *  @brief Precompute joint values and move away from them instead of towards them
         */
        inline void projectAwayFromLimits(const cobalt::math::linear_algebra::Vector<nJ> &q, cobalt::math::linear_algebra::Vector<nJ> &dq) {
            for(id_t j = 0; j < nJ; j++) {
                float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                float upperMargin = robot_.model().getJoints()[j].getMaxLimit() - projectMargin_ * range;
                float lowerMargin = robot_.model().getJoints()[j].getMinLimit() + projectMargin_ * range;

                if((q[j] + dq[j] * step_ > upperMargin) && (dq[j] > 0)) { dq[j] = 0.0f; }
                if((q[j] + dq[j] * step_ < lowerMargin) && (dq[j] < 0)) { dq[j] = 0.0f; }
            }
            
        }

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a inverse kinematics solver for an arbitrary robot
         *  @param robot Robot to solve IK for
         */
        explicit InverseKinematics(Robot<nL, nJ, nE> &robot, 
                                   iter_t maxIterations = IK_DEFAULT_MAX_ITERATIONS,
                                   float threshold = IK_DEFAULT_THRESHOLD,
                                   float step = IK_DEFAULT_STEP,
                                   float damping = IK_DEFAULT_DAMPING,
                                   float projectionMargin = IK_DEFAULT_MARGIN
                                ) : robot_(robot), jacobian_(robot), fk_(robot), maxIterations_(maxIterations), step_(step), damping_(damping), projectMargin_(projectionMargin)  {}

        // ---------------- Getters ----------------
        inline constexpr iter_t getMaxIterations() const { return maxIterations_; }
        inline constexpr float getThreshold() const { return threshold_; }
        inline constexpr float getDampingCoeff() const { return damping_; }
        inline constexpr float getStepSize() const { return step_; }
        inline constexpr float getProjMargin() const { return projectMargin_; }

        // ---------------- Setters ----------------
        void setMaxIterations(iter_t iterations) { maxIterations_ = iterations; }
        void setThreshold(float threshold) { 
            assert(threshold >= 0.0f && "Threshold must be non-negative");
            threshold_ = threshold; 
        }
        void setDampingCoeff(float dampingCoeff) { 
            assert(dampingCoeff >= 0.0f && "Damping coefficient must be non-negative");
            damping_ = dampingCoeff; 
        }
        void setStepSize(float stepSize) { 
            assert(stepSize >= 0.0f && "Step size must be positive");
            step_ = stepSize; 
        }
        void setProjMargin(float projectionMargin) { 
            assert(projectMargin_ >= 0.0f && "Margion of projection must be positive");
            projectMargin_ = projectionMargin; 
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Compute the joint angles needed to achieve a single target state of the robot
         */
        IKSolution<nJ> solve(const IKTarget &target) {
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;

            IKSolution<nJ> output;
            output.status = IKStatus::MaxIterations;
            output.error = cobalt::math::linear_algebra::Vector<6>::zero();
            output.q = robot_.state().q;

            iter_t iter;
            for(iter = 0; iter < maxIterations_; iter++) {
                fk_.solve(robot_.state());

                cobalt::math::linear_algebra::Vector<3> posErr = computePosError(target, robot_.state().frameTransforms[target.frameId]);

                output.error[0] = posErr.x();
                output.error[1] = posErr.y();
                output.error[2] = posErr.z();
                output.error[3] = 0.0f;
                output.error[4] = 0.0f;
                output.error[5] = 0.0f;

                if(norm(posErr) < threshold_) {
                    output.status = IKStatus::Success;
                    output.iterations = iter;
                    output.q = robot_.state().q;

                    break;
                }

                cobalt::math::linear_algebra::Matrix<6, nJ> J = jacobian_.compute(robot_.state(), target.frameId);
                cobalt::math::linear_algebra::Matrix<3, nJ> posJ = cobalt::math::linear_algebra::Matrix<3, nJ>::zero();
                for(cobalt::math::index_t i = 0; i < 3; i++) {
                    for(cobalt::math::index_t j = 0; j < nJ; j++) {
                        posJ(i, j) = J(i, j);
                    }
                }

                cobalt::math::linear_algebra::Matrix<nJ, 3> Jpinv = computePosPseudoInv(posJ, norm(posErr));
                cobalt::math::linear_algebra::Vector<nJ> dq = Jpinv * posErr;

                projectAwayFromLimits(robot_.state().q, dq);

                cobalt::math::linear_algebra::Vector<nJ> q = robot_.state().q + dq * step_;
                robot_.setJoints(q);
            }

            if(iter >= maxIterations_) { 
                output.status = IKStatus::MaxIterations; 
                output.iterations = maxIterations_;
                output.q = robot_.state().q;

                fk_.solve(robot_.state());
                cobalt::math::linear_algebra::Vector<3> finalPos = robot_.state().frameTransforms[target.frameId].translation();
                cobalt::math::linear_algebra::Vector<3> finalErr = target.pose.translation() - finalPos;

                output.error[0] = finalErr.x();
                output.error[1] = finalErr.y();
                output.error[2] = finalErr.z();
            }
            
            robot_.setJoints(q_init);

            return output;
        }
};

} // cobalt::kinematics::solver