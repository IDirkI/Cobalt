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
#include "cobalt/math/linear_algebra/matrix/matrix_util.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_ops.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics::solvers {

// ---------------- Enum ----------------
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

enum class IKSolver : uint8_t {
    Mixed,
    DLS,
    SVD,
};

// ---------------- Definitions ----------------
// Stability
constexpr float IK_MIN_STEP_SIZE = 1e-7;
constexpr float IK_MIN_ERROR_CHANGE = 1e-5;

// Line-search
constexpr float IK_BACKTRACK_FACTOR = 0.5f;
constexpr iter_t IK_BACKTRACK_NUM = 10;
constexpr float IK_ARMIJO_COND_CONST = 0.1f;

// Singularity escape
constexpr float IK_SINGULAR_ESCAPE_FACTOR = 0.1f;

// Status Checks
constexpr iter_t IK_NOPROG_THRESHOLD = 100;
constexpr iter_t IK_NOPROG_SOFT_THRESHOLD = 20;
constexpr iter_t IK_SMALLPROG_THRESHOLD = 10;
constexpr float IK_SINGULAR_ESCAPE_THRESHOLD = 5;

// Defaults
constexpr IKSolver IK_DEFAULT_SOLVER = IKSolver::Mixed;
constexpr iter_t IK_DEFAULT_MAX_ITERATIONS = 200;
constexpr float IK_DEFAULT_THRESHOLD = 1e-3;
constexpr float IK_DEFAULT_STEP = 0.5f;
constexpr float IK_DEFAULT_DAMPING_DLS = 1e-2f;
constexpr float IK_DEFAULT_MARGIN = 0.05f;
constexpr float IK_DEFAULT_SINGULAR_THRESHOLD = 100.0f;
constexpr float IK_DEFAULT_MANIP_THRESHOLD = 1e-4f;
constexpr float IK_DEFAULT_SVD_DIR_DAMPING_ALPHA = 1.0f;
constexpr float IK_DEFAULT_SVD_SIGMA_THRESHOLD = 1e-3f;
constexpr float IK_DEFAULT_SVD_NEAR_THRESHOLD = 1e-2f;
constexpr float IK_DEFAULT_UNREACHABLE_MULT = 5;

// ---------------- Structs ----------------
/**
 *  @brief Frames goal position/orientation/pose after the IK process
 */
struct IKTarget {
    id_t frameId;
    IKMode mode;
    cobalt::math::geometry::Transform<> pose;
    cobalt::math::linear_algebra::Vector<6> weight{1,1,1,1,1,1};
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

/**
 *  @brief Configuration of the IK solver after solving
 *  @param solver Solver type to use
 *  @param maxIterations Maximum number of iterations before termination
 *  @param threshold Error threshold for termination
 *  @param damping Damping factor for DLS solver
 *  @param step Step size for each iteration
 *  @param projectMargin Margin to keep from joint limits when projecting
 *  @param singularityThreshold Threshold for detecting singularities (manipulability measure)
 *  @param manipThreshold Threshold for manipulability measure to consider singular
 *  @param svdSigmaMin Minimum singular value threshold for SVD solver
 *  @param svdSigmaNear Near singular value threshold for SVD solver
 *  @param svdDirDamping Alpha damping factor for near-singular directions in S
 *  @param unreachableMult Multiplier to increase step size when target is unreachable
 *  @note All parameters are optional and have default values
 *  @warning Improper configuration may lead to poor convergence or failure to converge
 */
struct IKConfig {
    // Solver type
    IKSolver solver = IK_DEFAULT_SOLVER;

    // Termination
    iter_t maxIterations = IK_DEFAULT_MAX_ITERATIONS;
    float threshold = IK_DEFAULT_THRESHOLD;

    // Step control
    float dampingDLS = IK_DEFAULT_DAMPING_DLS;
    float step = IK_DEFAULT_STEP;
    float projectMargin = IK_DEFAULT_MARGIN;

    // Singularity handling
    float singularityThreshold = IK_DEFAULT_SINGULAR_THRESHOLD;
    float manipThreshold = IK_DEFAULT_MANIP_THRESHOLD;

    // SVD
    float svdSigmaMin = IK_DEFAULT_SVD_SIGMA_THRESHOLD;
    float svdSigmaNear = IK_DEFAULT_SVD_NEAR_THRESHOLD;
    float svdDirDamping = IK_DEFAULT_SVD_DIR_DAMPING_ALPHA;

    // Unreachability
    float unreachableMult = IK_DEFAULT_UNREACHABLE_MULT;
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
        IKConfig config_;
        JacobianBuilder<nL, nJ, nE> jacobian_;
        ForwardKinematics<nL, nJ, nE> fk_;

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
         *  @brief Compute the damped least-squares (DLS) psuedo-inverse of the task jacobian
         *  @param J Task-jacobian of the current robot state
         *  @param errNorm Last norm of the error to be used in adaptive damping
         *  @return nJx3/6 damped pseudo-inverse of the task jacobian
         *  @note Uses right-inverse for [nJ >= M] and left-inverse for [nJ < M]
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInvDLS(const cobalt::math::linear_algebra::Matrix<M, nJ> &J, float errNorm, bool &isSingular) {
            float lambdaSqr = config_.dampingDLS*config_.dampingDLS;

            if constexpr (nJ >= M) {    // Wide J, J† = Jᵀ(JJᵀ + λ²I)⁻¹
                cobalt::math::linear_algebra::Matrix<M, M> A = J * transpose(J);
                A += cobalt::math::linear_algebra::Matrix<M, M>::eye() * lambdaSqr;

                cobalt::math::linear_algebra::Matrix<M, M> L;
                if(!cobalt::math::linear_algebra::cholesky(A, L)) { 
                    isSingular = true;
                    return cobalt::math::linear_algebra::transpose(J);
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv;
                for(cobalt::math::index_t j = 0; j < nJ; j++) {
                    cobalt::math::linear_algebra::Vector<M> b = cobalt::math::linear_algebra::getRow(cobalt::math::linear_algebra::transpose(J), j);
                    cobalt::math::linear_algebra::Vector<M> x;

                    if(!cobalt::math::linear_algebra::solvePSD(A, b, x)) {
                        isSingular = true;
                        return cobalt::math::linear_algebra::transpose(J);
                    }

                    for(cobalt::math::index_t i = 0; i < M; i++) {
                        J_pinv(j, i) = x[i];
                    }
                }

                return J_pinv;
            }
            else {                      // Tall J, J† = (JᵀJ + λ²I)⁻¹Jᵀ
                cobalt::math::linear_algebra::Matrix<nJ, nJ> A = transpose(J)*J;
                A += cobalt::math::linear_algebra::Matrix<nJ, nJ>::eye() * lambdaSqr;

                cobalt::math::linear_algebra::Matrix<nJ, nJ> L;
                if(!cobalt::math::linear_algebra::cholesky(A, L)) { 
                    isSingular = true;
                    return cobalt::math::linear_algebra::transpose(J);
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv;
                for(cobalt::math::index_t j = 0; j < M; j++) {
                    cobalt::math::linear_algebra::Vector<nJ> b = cobalt::math::linear_algebra::getColumn(cobalt::math::linear_algebra::transpose(J), j);
                    cobalt::math::linear_algebra::Vector<nJ> x;

                    if(!cobalt::math::linear_algebra::solvePSD(A, b, x)) {
                        isSingular = true;
                        return cobalt::math::linear_algebra::transpose(J);
                    }

                    for(cobalt::math::index_t i = 0; i < nJ; i++) {
                        J_pinv(i, j) = x[i];
                    }
                }

                return J_pinv;
            }
        }

        /**
         *  @brief Compute the SVD based psuedo-inverse of the task jacobian
         *  @param J Task-jacobian of the current robot state
         *  @param err Last error to be used in adaptive damping
         *  @return nJx3/6 damped pseudo-inverse of the task jacobian
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInvSVD(const cobalt::math::linear_algebra::Matrix<M, nJ> &J,  cobalt::math::linear_algebra::Vector<M> err, bool &isSingular, bool &isNearSingularity, float &manip) {
            cobalt::math::linear_algebra::Matrix<M, M> U;
            cobalt::math::linear_algebra::Matrix<M, nJ> S;
            cobalt::math::linear_algebra::Matrix<nJ, nJ> V;

            iter_t iter = cobalt::math::linear_algebra::svd(J, U, S, V);

            cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv = cobalt::math::linear_algebra::Matrix<nJ, M>::zero();

            manip = 1.0f;
            const float sigMax = S(0,0);
            cobalt::math::index_t r = (M < nJ) ?M :nJ;
            for(cobalt::math::index_t i = 0; i < r; i++) {
                const float sig = S(i,i);
                manip *= sig;

                if(sig < config_.svdSigmaMin * sigMax) {
                    isSingular = true;
                    continue;
                } 

                if(sig < config_.svdSigmaNear * sigMax) {
                    isNearSingularity = true;
                } 
                if(manip < config_.manipThreshold) {
                    isNearSingularity = true;
                } 
                
                // Directional damping
                float ue = cobalt::math::linear_algebra::dot(cobalt::math::linear_algebra::getColumn(U, i), err);
                float lambda_i = config_.svdDirDamping * ue*ue;
                const float dampedSig = sig/(sig*sig + lambda_i*lambda_i);

                for(cobalt::math::index_t j = 0; j < nJ; j++) {
                    for(cobalt::math::index_t k = 0; k < M; k++) {
                        J_pinv(j, k) += V(j, i) * dampedSig * U(k, i);
                    }
                }
            }

            return J_pinv;
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
         *  @brief Perform backtracking line search to find suitable step size
         *  @param q Current joint values
         *  @param dq proposed joint value steps
         *  @param target IK target being solved
         *  @param err Current error norm
         *  @return Suitable step size
         */
        template<cobalt::math::index_t M>
        inline float getStepSize(const cobalt::math::linear_algebra::Vector<nJ> &q, cobalt::math::linear_algebra::Vector<nJ> &dq, const IKTarget &target, float err) {
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;
            
            float alpha = config_.step;

            for(iter_t i = 0; i < IK_BACKTRACK_NUM; i++) {
                cobalt::math::linear_algebra::Vector<nJ> q_prime = q + alpha * dq;
                robot_.setJoints(q_prime);
                fk_.solve(robot_.state());

                cobalt::math::linear_algebra::Vector<6> err_test = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
                cobalt::math::linear_algebra::Vector<6> err_test_W = cobalt::math::linear_algebra::Matrix<6,6>::diagonal(target.weight) * err_test;
                cobalt::math::linear_algebra::Vector<M> taskErr_test = extractTaskError<M>(err_test_W, target.mode);

                float errNorm = norm(taskErr_test);

                if(errNorm <= err - IK_ARMIJO_COND_CONST*alpha*norm(dq)) {
                    robot_.setJoints(q_init);
                    fk_.solve(robot_.state());
                    return alpha;
                }

                alpha *= IK_BACKTRACK_FACTOR;
            }

            robot_.setJoints(q_init);
            fk_.solve(robot_.state());

            return alpha;
        }

        /**
         *  @brief Precompute joint values and move away from them instead of towards them
         *  @param q Current joint values
         *  @param dq proposed jonit value steps 
         *  @param step Step size to compute for
         */
        inline void projectAwayFromLimits(const cobalt::math::linear_algebra::Vector<nJ> &q, cobalt::math::linear_algebra::Vector<nJ> &dq, float step) {
            for(id_t j = 0; j < nJ; j++) {
                float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                float upperMargin = robot_.model().getJoints()[j].getMaxLimit() - config_.projectMargin * range;
                float lowerMargin = robot_.model().getJoints()[j].getMinLimit() + config_.projectMargin * range;

                if((q[j] + dq[j] * step > upperMargin) && (dq[j] > 0)) { dq[j] = 0.0f; }
                if((q[j] + dq[j] * step < lowerMargin) && (dq[j] < 0)) { dq[j] = 0.0f; }
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

            // Error DOF weights
            cobalt::math::linear_algebra::Matrix<6,6> W = cobalt::math::linear_algebra::Matrix<6,6>::diagonal(target.weight);

            // Main IK loop
            iter_t iter;
            for(iter = 0; iter < config_.maxIterations; iter++) {
                fk_.solve(robot_.state());

                const cobalt::math::geometry::Transform<> &T_curr = robot_.state().frameTransforms[target.frameId];
                
                cobalt::math::linear_algebra::Vector<6> error = computeError(target.pose, T_curr);
                cobalt::math::linear_algebra::Vector<6> error_W = W * error;
                cobalt::math::linear_algebra::Vector<M> taskErr = extractTaskError<M>(error_W, target.mode);
                output.error = error;
                float errNorm = norm(taskErr);

                if(errNorm < config_.threshold) {    // IK terminate check
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
                        if(errNorm > config_.threshold * config_.unreachableMult) {
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
                cobalt::math::linear_algebra::Matrix<6, nJ> J_W = W * J;
                cobalt::math::linear_algebra::Matrix<M, nJ> J_task = extractTaskJacobian<M>(J_W, target.mode);

                bool isSingular = false;
                bool isNearSignularity = false;

                cobalt::math::linear_algebra::Matrix<nJ,M> J_pinv;

                switch(config_.solver) {
                    case(IKSolver::DLS): {
                        J_pinv = computePseudoInvDLS(J_task, errNorm, isSingular);
                        break;
                    }
                    case(IKSolver::SVD): {
                        float manip = 0.0f;
                        J_pinv = computePseudoInvSVD(J_task, taskErr, isSingular, isNearSignularity, manip);
                        break;
                    }
                    case(IKSolver::Mixed): {
                        float manip = 0.0f;
                        cobalt::math::linear_algebra::Matrix<nJ,M> J_dls = computePseudoInvDLS(J_task, errNorm, isSingular);
                        cobalt::math::linear_algebra::Matrix<nJ,M> J_svd = computePseudoInvSVD(J_task, taskErr, isSingular, isNearSignularity, manip);

                        float w = std::clamp(manip/config_.manipThreshold, 0.0f, 1.0f);
                        J_pinv = w*J_dls + (1.0f - w)*J_svd;
                        break;
                    }
                    default: { break; }
                }

                // Handle singulartiy
                if((isSingular || isNearSignularity) && noProgCount > IK_NOPROG_SOFT_THRESHOLD) {
                    if(singularCount >= IK_SINGULAR_ESCAPE_THRESHOLD) {
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

                // Get next iteration & step
                cobalt::math::linear_algebra::Vector<nJ> delta_q = J_pinv*taskErr;
                float stepSize = getStepSize<M>(robot_.state().q, delta_q, target, errNorm);
                projectAwayFromLimits(robot_.state().q, delta_q, stepSize);

                // Handle step-size
                if((norm(delta_q)*stepSize < IK_MIN_STEP_SIZE) && (errNorm > config_.threshold * config_.unreachableMult)) {
                    smallProgCount++;
                    if(smallProgCount >= IK_SMALLPROG_THRESHOLD) {
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
                cobalt::math::linear_algebra::Vector<nJ> q_new = robot_.state().q + delta_q * stepSize;
                robot_.setJoints(q_new);

                // Cache state data
                robot_.state().dq = delta_q;
                robot_.state().J = J;
            }

            if(iter >= config_.maxIterations) {
                if(minErr > config_.threshold * config_.unreachableMult) {
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
        explicit InverseKinematics(Robot<nL, nJ, nE> &robot, IKConfig config) 
                    : robot_(robot), config_(config), jacobian_(robot), fk_(robot)  {}

        // ---------------- Accessors ----------------
        /**
         *  @brief Get reference to IK configuration
         *  @return Reference to IK configuration
         */
        constexpr IKConfig &config() {
            return config_;
        } 

        /**
         *  @brief Get const reference to IK configuration
         *  @return Const reference to IK configuration
         */
        constexpr const IKConfig &config() const {
            return config_;
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