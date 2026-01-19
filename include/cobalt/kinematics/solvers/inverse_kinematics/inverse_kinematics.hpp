#pragma once

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

constexpr iter_t IK_SINGULAR_THRESHOLD = 10;

constexpr iter_t IK_DEFAULT_MAX_ITERATIONS = 100;
constexpr float IK_DEFAULT_THRESHOLD = 1e-4;
constexpr float IK_DEFAULT_DAMPING = 1e-3;
constexpr float IK_DEFAULT_STEP = 0.1f;

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

         // ---------------- Helper Functions ----------------
        /**
         *  @brief Compute the current error of the robot state vs the target destination
         */
        inline cobalt::math::linear_algebra::Vector<6> computeError(const IKTarget &target, const cobalt::math::geometry::Transform<> &T_frame) {
            cobalt::math::linear_algebra::Vector<6> err;

            err[0] = target.pose.translation().x() - T_frame.translation().x();
            err[1] = target.pose.translation().y() - T_frame.translation().y();
            err[2] = target.pose.translation().z() - T_frame.translation().z();

            cobalt::math::geometry::Quaternion<> errQ = target.pose.rotation() * inv(T_frame.rotation());
            cobalt::math::linear_algebra::Vector<3> errRot = toRotationVector(errQ);
            err[3] = errRot.x();
            err[4] = errRot.y();
            err[5] = errRot.z();

            return err;
        }

        /**
         *  @brief Compute the damped 
         */
        inline cobalt::math::linear_algebra::Matrix<nJ, 6> computeDampedPseudoInv(const cobalt::math::linear_algebra::Matrix<6, nJ> &J, IKSolution<nJ> &solution) {
            cobalt::math::linear_algebra::Matrix<nJ, nJ> JtJ= transpose(J) * J;

            if(conditionNum(JtJ) > IK_SINGULAR_THRESHOLD) { solution.status = IKStatus::Singular; }

            cobalt::math::linear_algebra::Matrix<nJ, nJ> A = JtJ + cobalt::math::linear_algebra::Matrix<nJ, nJ>::eye()*(damping_*damping_);
            cobalt::math::linear_algebra::Matrix<nJ, nJ> Ainv;
            bool result = inv(A, Ainv);
            cobalt::math::linear_algebra::Matrix<nJ, 6> Jpsi = Ainv * transpose(J);
            return Jpsi;
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
                                   float damping = IK_DEFAULT_DAMPING
                                ) : robot_(robot), jacobian_(robot), fk_(robot), maxIterations_(maxIterations), step_(step), damping_(damping) {}

        // ---------------- Getters ----------------
        inline constexpr iter_t getMaxIterations() const { return maxIterations_; }
        inline constexpr float getThreshold() const { return threshold_; }
        inline constexpr float getDampingCoeff() const { return damping_; }
        inline constexpr float getStepSize() const { return step_; }

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

        // ---------------- Member Functions ----------------
        /**
         *  @brief Compute the joint angles needed to achieve a single target state of the robot
         */
        IKSolution<nJ> solve(const IKTarget &target) {
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;

            iter_t iter;
            cobalt::math::linear_algebra::Vector<nJ> q = robot_.state().q;
            cobalt::math::linear_algebra::Vector<6> err;
            IKSolution<nJ> output;

            cobalt::math::linear_algebra::Matrix<6, nJ> J = cobalt::math::linear_algebra::Matrix<6, nJ>::eye();
            for(iter = 0; iter < maxIterations_; iter++) {
                fk_.solve(robot_.state());

                J = jacobian_.compute(robot_.state(), target.frameId);
                err = computeError(target, robot_.state().frameTransforms[target.frameId]);

                switch (target.mode) {
                    case (IKMode::Position): {
                        err[3] = 0.0f;
                        err[4] = 0.0f;
                        err[5] = 0.0f;
                        break;
                    }
                    case (IKMode::Orientation): {
                        err[0] = 0.0f;
                        err[1] = 0.0f;
                        err[2] = 0.0f;
                        break;
                    }
                    case (IKMode::Pose): { break; }
                    default: { break; }
                }

                if(norm(err) < threshold_) {
                    output.status = IKStatus::Success;
                    break;
                }
                
                cobalt::math::linear_algebra::Matrix<nJ, 6> Jpsi = computeDampedPseudoInv(J, output);
                cobalt::math::linear_algebra::Vector<nJ> dq = Jpsi * err;

                q += dq * step_;
                robot_.setJoints(q);
                q = robot_.state().q;
            }

            if(iter == maxIterations_) { output.status = IKStatus::MaxIterations; }
            
            robot_.setJoints(q_init);

            output.q = q;
            output.error = err;
            output.iterations = iter;

            return output;
        }
};

} // cobalt::kinematics::solver