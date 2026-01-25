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
constexpr float IK_MANIPULABILITY_THRESHOLD = 1e-4f;

constexpr float IK_MAXREACH_MARGIN = 1.05;
constexpr float IK_MIN_ERROR_CHANGE = 1e-5;
constexpr float IK_MIN_STEP_SIZE = 1e-7;
constexpr float IK_MAX_SINGULAR_COUNT = 5;
constexpr float IK_UNREACHABLE_MULT = 5;
constexpr float IK_SINGULAR_ESCAPE_FACTOR = 0.1f;
constexpr iter_t IK_NOPROG_THRESHOLD = 100;
constexpr iter_t IK_SMALLPROG_THRESHOLD = 10;

constexpr iter_t IK_DEFAULT_MAX_ITERATIONS = 200;
constexpr float IK_DEFAULT_THRESHOLD = 1e-3;
constexpr float IK_DEFAULT_DAMPING = 1e-2;
constexpr float IK_DEFAULT_STEP = 0.3f;
constexpr float IK_DEFAULT_MARGIN = 0.05f;

enum class IKStatus : uint8_t {
    Success,
    Unreachable,
    MaxIterations,
    Singular,
    InvalidMode,
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
         *  @param dst Destination frame
         *  @param src Source frame
         *  @return 6-Vector total error from T_src to T_dst
         */
        inline cobalt::math::linear_algebra::Vector<6> computeError(const cobalt::math::geometry::Transform<> &dst, const cobalt::math::geometry::Transform<> &src) {
            cobalt::math::linear_algebra::Vector<6> err = cobalt::math::linear_algebra::Vector<6>::zero();

            // Position error
            cobalt::math::linear_algebra::Vector<3> posErr = dst.translation() - src.translation();

            // Orientation error
            cobalt::math::geometry::Quaternion<> dstQ = normalize(dst.rotation());
            cobalt::math::geometry::Quaternion<> srcQ = normalize(src.rotation());

            cobalt::math::geometry::Quaternion<> qErr = cobalt::math::geometry::shortestPath(srcQ, dstQ);

            cobalt::math::linear_algebra::Vector<3> rotErr = cobalt::math::geometry::toRotationVector(qErr);

            // Full error
            err[0] = posErr.x();
            err[1] = posErr.y();
            err[2] = posErr.z();
            err[3] = rotErr.x();
            err[4] = rotErr.y();
            err[5] = rotErr.z();

            return err;
        }

        /**
         *  @brief Extract task-space error associated with the given IKMode from the total error
         *  @param err Total 6-Vector error
         *  @param mode IKMode the target frame wants to achieve
         *  @return 3/6-Vector task error extracted from the full error
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Vector<M> extractTaskError(const cobalt::math::linear_algebra::Vector<6> &err, IKMode mode) {
            cobalt::math::linear_algebra::Vector<M> taskErr = cobalt::math::linear_algebra::Vector<M>::zero();

            switch(mode) {
                case(IKMode::Position): {
                    assert(M == 3 &&"Position IK uses a 3-Vector error");
                    taskErr[0] = err[0];
                    taskErr[1] = err[1];
                    taskErr[2] = err[2];
                    break;
                }
                case(IKMode::Orientation): {
                    assert(M == 3 && "Orientation IK uses a 3-Vector error");
                    taskErr[0] = err[3];
                    taskErr[1] = err[4];
                    taskErr[2] = err[5];
                    break;
                }
                case(IKMode::Pose): {
                    assert(M == 6 && "Pose IK uses a 6-Vector error");
                    for(cobalt::math::index_t i = 0; i < M; i++) {
                        taskErr[i] = err[i];
                    }
                    break;
                }
                default: { break; }
            }

            return taskErr;
        }

        /**
         *  @brief Extracts the task-space jacobian associated with the given IKMode from the total jacobian
         *  @param J Full robot jacobian
         *  @param mode IKMode the target frame wants to achieve
         *  @return 3/6xnJ task-space jacobian extracted from the full jacobian
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<M, nJ> extractTaskJacobian(const cobalt::math::linear_algebra::Matrix<6, nJ> &J, IKMode mode) {
            cobalt::math::linear_algebra::Matrix<M, nJ> J_task = cobalt::math::linear_algebra::Matrix<M, nJ>::zero();

            switch(mode) {
                case(IKMode::Position): {
                    assert(M == 3 && "Position IK uses a 3xnJ-Jacobian");
                    for(cobalt::math::index_t i = 0; i < 3; i++) {
                        for(cobalt::math::index_t j = 0; j < nJ; j++) {
                            J_task(i, j) = J(i, j);
                        }
                    }
                    break;
                }
                case(IKMode::Orientation): {
                    assert(M == 3 && "Orientation IK uses a 3xnJ-Jacobian");
                    for(cobalt::math::index_t i = 0; i < 3; i++) {
                        for(cobalt::math::index_t j = 0; j < nJ; j++) {
                            J_task(i, j) = J(i+3, j);
                        }
                    }
                    break;
                }
                case(IKMode::Pose): {
                    assert(M == 6 && "Pose IK uses a 6xnJ-Jacobian");
                    for(cobalt::math::index_t i = 0; i < M; i++) {
                        for(cobalt::math::index_t j = 0; j < nJ; j++) {
                            J_task(i, j) = J(i, j);
                        }
                    }
                    break;
                }
                default: { break; }
            }

            return J_task;
        }

        /**
         *  @brief Compute manipulability measure for singularity detection
         *  @param J Task jacobian
         *  @return Manipulability measure
         */
        template<cobalt::math::index_t M>
        inline float computeManipulability(const cobalt::math::linear_algebra::Matrix<M, nJ> &J) {
            if constexpr (nJ >= M) {
                cobalt::math::linear_algebra::Matrix<M, M> JJt = J * transpose(J);
                return std::abs(det(JJt));
            } else {
                cobalt::math::linear_algebra::Matrix<nJ, nJ> JtJ = transpose(J) * J;
                return std::abs(det(JtJ));
            }
        }

        /**
         *  @brief Check if configuration is near a singularity
         *  @param J Task jacobian
         *  @return `true` if near singularity, `false` otherwise
         */
        template<cobalt::math::index_t M>
        inline bool isNearSingularity(const cobalt::math::linear_algebra::Matrix<M, nJ> &J) {
            float manipulability = computeManipulability<M>(J);
            return (manipulability < IK_MANIPULABILITY_THRESHOLD);
        }


        /**
         *  @brief Compute the maximum max reachable distance of the robot
         *  @param frameId ID of the frame to check max reach of
         *  @return Maxium distance the robot can reach 
         */
        inline float computeMaxReach(id_t frameId) const {
            float maxReach = 0.0f;

            for(const Joint &j : robot_.model().getJoints()) {
                if(j.getType() == JointType::Prismatic) { 
                    maxReach += std::abs(j.getMaxLimit());
                }
                else if(j.getType() == JointType::Revolute) {
                    maxReach += norm(j.getOrigin().translation());
                }
            }

            maxReach += norm(robot_.model().getFrames()[frameId].getOrigin().translation());

            return maxReach;
        } 

        /**
         *  @brief Compute the damped psuedo-inverse of the position jacobian
         *  @param J Task-jacobian of the current robot state
         *  @param errNorm Last norm of the error to be used in adaptive damping
         *  @return nJx3/6 damped pseudo-inverse of the task jacobian
         *  @note Uses right-inverse for [nJ >= M] and left-inverse for [nJ < M]
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInv(const cobalt::math::linear_algebra::Matrix<M, nJ> &J, float errNorm, bool &isSingular) {
            float lambda = damping_*(1.0f + errNorm);

            if constexpr (nJ >= M) {    // Wide J, J† = Jᵀ(JJᵀ + λ²I)⁻¹
                cobalt::math::linear_algebra::Matrix<M, M> JJt = J * transpose(J);
                cobalt::math::linear_algebra::Matrix<M, M> A = JJt + cobalt::math::linear_algebra::Matrix<M, M>::eye()*(lambda * lambda);

                float cond = cobalt::math::linear_algebra::conditionNum(JJt);
                isSingular = (cond > IK_SINGULAR_THRESHOLD);

                cobalt::math::linear_algebra::Matrix<M, M> Ainv;
                bool success = inv(A, Ainv);
                if(!success) { 
                    isSingular = true;
                    return cobalt::math::linear_algebra::Matrix<nJ, M>::zero(); 
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> Jpinv = transpose(J) * Ainv;
                return Jpinv;
            }
            else {                      // Tall J, J† = (JᵀJ + λ²I)⁻¹Jᵀ
                cobalt::math::linear_algebra::Matrix<nJ, nJ> JtJ = transpose(J) * J;
                cobalt::math::linear_algebra::Matrix<nJ, nJ> A = JtJ + cobalt::math::linear_algebra::Matrix<nJ, nJ>::eye()*(lambda * lambda);

                float cond = cobalt::math::linear_algebra::conditionNum(JtJ);
                isSingular = (cond > IK_SINGULAR_THRESHOLD);

                cobalt::math::linear_algebra::Matrix<nJ, nJ> Ainv;
                bool success = inv(A, Ainv);
                if(!success) { 
                    isSingular = true;
                    return cobalt::math::linear_algebra::Matrix<nJ, M>::zero(); 
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> Jpinv = Ainv * transpose(J);
                return Jpinv;
            }
        }

        /**
         *  @brief Attempt to escape singularity using deterministic perturbation
         *  @param q Current joint configuration
         *  @param escapeAttempt Current escape attempt
         *  @return Perturbed joint configuration
         */
        inline cobalt::math::linear_algebra::Vector<nJ> escapeSingularity(  // TODO: Replace with HAL random()
            const cobalt::math::linear_algebra::Vector<nJ> &q, iter_t escapeAttempt) {
            
            cobalt::math::linear_algebra::Vector<nJ> q_escaped = q;
            
            for(id_t j = 0; j < nJ; j++) {
                float range = robot_.model().getJoints()[j].getMaxLimit() - 
                            robot_.model().getJoints()[j].getMinLimit();
                
                float phase = static_cast<float>(j) * 2.0f + static_cast<float>(escapeAttempt);
                float pb = std::sin(phase * 1.618033988f) * std::cos(phase * 2.718281828f);
                
                float perturbation = pb * IK_SINGULAR_ESCAPE_FACTOR * range;
                
                q_escaped[j] += perturbation;
            }
            
            // Ensure limits
            robot_.setJoints(q_escaped);
            q_escaped = robot_.state().q;
            
            return q_escaped;
        }

        /**
         *  @brief Precompute joint values and move away from them instead of towards them
         *  @param q Current joint values
         *  @param dq proposed jonit value steps 
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

        /**
         *  @brief Solve the IK problem for the given task-mode
         */
        template<cobalt::math::index_t M>
        inline IKSolution<nJ> solveTask(const IKTarget &target) {
            // Save initial values to restore later
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;

            // Generate output
            IKSolution<nJ> output;
            output.status = IKStatus::MaxIterations;
            output.q = robot_.state().q;
            output.error = cobalt::math::linear_algebra::Vector<6>::zero();
            output.iterations = 0;

            // Track unreachability
            float prevErr = std::numeric_limits<float>::max();
            float minErr = std::numeric_limits<float>::max();
            iter_t noProgCount = 0;
            iter_t smallProgCount = 0;
            iter_t singularCount = 0;

            // Check max reachability length
            if(target.mode == IKMode::Position || target.mode == IKMode::Pose) {
                float targetDist = norm(target.pose.translation());
                float maxReach = computeMaxReach(target.frameId);

                if((targetDist > maxReach*IK_MAXREACH_MARGIN) && false) { // TODO: Refactor computeMaxReach
                    output.status = IKStatus::Unreachable;
                    fk_.solve(robot_.state());
                    output.error = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
                    robot_.setJoints(q_init);

                    return output;
                }
            }

            // Main IK loop
            iter_t iter;
            for(iter = 0; iter < maxIterations_; iter++) {
                fk_.solve(robot_.state());

                const cobalt::math::geometry::Transform<> &T_curr = robot_.state().frameTransforms[target.frameId];

                cobalt::math::linear_algebra::Vector<6> error = computeError(target.pose, T_curr);
                cobalt::math::linear_algebra::Vector<M> taskErr = extractTaskError<M>(error, target.mode);
                output.error = error;
                float errNorm = norm(taskErr);

                if(errNorm < threshold_) {    // IK terminate check
                    output.status = IKStatus::Success;
                    output.iterations = iter;
                    output.q = robot_.state().q;
                    break;
                }
                
                // Unreachablility checks
                if(errNorm < minErr) { minErr = errNorm; }
                if(prevErr - errNorm < IK_MIN_ERROR_CHANGE) {
                    noProgCount++;
                    if(noProgCount >= IK_NOPROG_THRESHOLD) {
                        if(errNorm > threshold_ * IK_UNREACHABLE_MULT) {
                            printf(">>>>NO-PROG\n");
                            output.status = IKStatus::Unreachable;
                            output.iterations = iter;
                            output.q = robot_.state().q;
                            break;
                        }
                    }
                }
                else {
                    noProgCount = 0;
                }
                prevErr = errNorm;

                // Jacobian computation
                cobalt::math::linear_algebra::Matrix<6, nJ> J = jacobian_.compute(robot_.state(), target.frameId);
                cobalt::math::linear_algebra::Matrix<M, nJ> J_task = extractTaskJacobian<M>(J, target.mode);

                bool isSingular = false;
                bool nearSignularity = isNearSingularity<M>(J_task);

                cobalt::math::linear_algebra::Matrix<nJ,M> J_pinv = computePseudoInv(J_task, errNorm, isSingular);

                // Handle singulartiy
                if(isSingular || nearSignularity) { 
                    if(singularCount >= IK_MAX_SINGULAR_COUNT) {
                        output.status = IKStatus::Singular;
                        output.iterations = iter;
                        output.q = robot_.state().q;
                        break;
                    }

                    // Escape and re-try
                    escapeSingularity(robot_.state().q, singularCount);
                    singularCount++;
                    continue;
                }

                // Get next iteration
                cobalt::math::linear_algebra::Vector<nJ> delta_q = J_pinv*taskErr;
                projectAwayFromLimits(robot_.state().q, delta_q);

                // Handle step-size
                if((norm(delta_q)*step_ < IK_MIN_STEP_SIZE) && (errNorm > threshold_ * IK_UNREACHABLE_MULT)) {
                    smallProgCount++;
                    if(smallProgCount >= IK_SMALLPROG_THRESHOLD) {
                            printf(">>>>SMALL-PROG\n");
                            output.status = IKStatus::Unreachable;
                            output.iterations = iter;
                            output.q = robot_.state().q;
                            break;
                    }
                }
                else {
                    smallProgCount = 0;
                }

                // Actuate
                cobalt::math::linear_algebra::Vector<nJ> q_new = robot_.state().q + delta_q * step_;

                printf("q = [ ");
                for(cobalt::math::index_t i = 0; i < nJ; i++) {
                    printf("%4.3f ", delta_q[i]);
                }
                printf("]\n\n");
                
                robot_.setJoints(q_new);
            }

            if(iter >= maxIterations_) {
                if(minErr > threshold_ * IK_UNREACHABLE_MULT) {
                    printf(">>>>MAX-ITER\n");
                    output.status = IKStatus::Unreachable;
                }
                else {
                    output.status = IKStatus::MaxIterations;
                }

                output.iterations = iter;
                output.q = robot_.state().q;

                fk_.solve(robot_.state());
 
                const cobalt::math::geometry::Transform<> T_final = robot_.state().frameTransforms[target.frameId];
                output.error = computeError(target.pose, T_final);
            }

            robot_.setJoints(q_init);
            fk_.solve(robot_.state());

            return output;
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
         *  @param target The IK solvers target containing frame info, final pose and IK-mode to solve for
         *  @return Struct of IKSolution that contains the optimal joint values, final errors, amount of iterations IK took and status of the solution
         */
        IKSolution<nJ> solve(const IKTarget &target) {
            switch (target.mode) {
                case (IKMode::Position): {
                    return solveTask<3>(target);
                }
                case (IKMode::Orientation): {
                    return solveTask<3>(target);
                }
                case (IKMode::Pose): {
                    return solveTask<6>(target);
                }
                default: {
                    IKSolution<nJ> invalidOutput;
                    invalidOutput.status = IKStatus::InvalidMode;
                    invalidOutput.iterations = 0;
                    return invalidOutput;
                }
                   
            }
        }
};

} // cobalt::kinematics::solver