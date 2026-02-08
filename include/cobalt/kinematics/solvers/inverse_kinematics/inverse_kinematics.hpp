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
    InvalidFrame,
};

enum class IKMode : uint8_t {
    Position,
    Orientation,
    Pose,
};

enum class IKSolver : uint8_t {
    DLS,
    SVD,
    Mixed,
};

// ---------------- Definitions ----------------
// --- Convergence & Stability ---
constexpr float IK_MIN_STEP_SIZE = 1e-7f;
constexpr float IK_MIN_ERROR_CHANGE = 1e-5f;

// --- Line Search ---
constexpr float IK_BACKTRACK_FACTOR = 0.5f;
constexpr iter_t IK_BACKTRACK_NUM = 10;
constexpr float IK_ARMIJO_COND_CONST = 0.1f;

// --- Singularity Handling ---
constexpr float IK_SINGULAR_ESCAPE_FACTOR = 0.1f;
constexpr iter_t IK_SINGULAR_ESCAPE_THRESHOLD = 5;
constexpr float IK_DEFAULT_SINGULAR_THRESHOLD = 100.0f;
constexpr float IK_DEFAULT_MANIP_THRESHOLD = 1e-4f;

// --- Progress Tracking ---
constexpr iter_t IK_NOPROG_THRESHOLD = 100;
constexpr iter_t IK_NOPROG_SOFT_THRESHOLD = 20;
constexpr iter_t IK_SMALLPROG_THRESHOLD = 10;

// --- DLS Damping ---
constexpr float IK_DEFAULT_DAMPING_DLS = 1e-2f;
constexpr float IK_DEFAULT_DAMPING_MIN = 1e-5f;
constexpr float IK_DEFAULT_DAMPING_MAX = 1e-1f;
constexpr float IK_DLS_LAMBDA_NOPROG_MULT = 0.1f;
constexpr float IK_DLS_LM_RHO_GOOD = 0.75f;
constexpr float IK_DLS_LM_RHO_BAD = 0.25f;
constexpr float IK_DLS_LM_DECREASE = 0.5f;
constexpr float IK_DLS_LM_INCREASE = 2.0f;

// --- SVD Parameters ---
constexpr float IK_DEFAULT_SVD_DIR_DAMPING_ALPHA = 1.0f;
constexpr float IK_DEFAULT_SVD_SIGMA_THRESHOLD = 1e-3f;
constexpr float IK_DEFAULT_SVD_NEAR_THRESHOLD = 1e-2f;

// --- Default Configuration ---
constexpr IKSolver IK_DEFAULT_SOLVER = IKSolver::Mixed;
constexpr iter_t IK_DEFAULT_MAX_ITERATIONS = 200;
constexpr float IK_DEFAULT_THRESHOLD = 1e-3f;
constexpr float IK_DEFAULT_STEP = 0.5f;
constexpr float IK_DEFAULT_MARGIN = 0.05f;
constexpr float IK_DEFAULT_UNREACHABLE_MULT = 5.0f;
constexpr float IK_DEFAULT_TIMESTEP = 0.1f;
constexpr bool IK_DEFAULT_VELOCITY_LIMIT_ENABLE = true;

// ---------------- Structs ----------------
/**
 *  @brief Configuration of the IK solver
 */
struct IKConfig {
    // Solver type
    IKSolver solver = IK_DEFAULT_SOLVER;

    // Termination criteria
    iter_t maxIterations = IK_DEFAULT_MAX_ITERATIONS;
    float threshold = IK_DEFAULT_THRESHOLD;

    // Step control
    float step = IK_DEFAULT_STEP;
    float dampingDLS = IK_DEFAULT_DAMPING_DLS;
    float dampingMin = IK_DEFAULT_DAMPING_MIN;
    float dampingMax = IK_DEFAULT_DAMPING_MAX;
    float projectMargin = IK_DEFAULT_MARGIN;

    // Velocity Limit
    float timeStep = IK_DEFAULT_TIMESTEP;
    bool enforceVelocityLimits = IK_DEFAULT_VELOCITY_LIMIT_ENABLE;

    // Singularity handling
    float singularityThreshold = IK_DEFAULT_SINGULAR_THRESHOLD;
    float manipThreshold = IK_DEFAULT_MANIP_THRESHOLD;

    // SVD specific
    float svdSigmaMin = IK_DEFAULT_SVD_SIGMA_THRESHOLD;
    float svdSigmaNear = IK_DEFAULT_SVD_NEAR_THRESHOLD;
    float svdDirDamping = IK_DEFAULT_SVD_DIR_DAMPING_ALPHA;

    // Unreachability detection
    float unreachableMult = IK_DEFAULT_UNREACHABLE_MULT;
};

/**
 *  @brief Frame's goal position/orientation/pose for IK
 */
struct IKTarget {
    id_t frameId;
    IKMode mode;
    cobalt::math::geometry::Transform<> pose;
    cobalt::math::linear_algebra::Vector<6> weight{1, 1, 1, 1, 1, 1};
};  

/**
 *  @brief IK solution containing joint configuration and status
 */
template<id_t nJ>
struct IKSolution {
    cobalt::math::linear_algebra::Vector<nJ> q;
    cobalt::math::linear_algebra::Vector<6> error;
    iter_t iterations;
    IKStatus status;
};

/**
 *  @brief Cache for the last valid pseudoinverse
 */
template<id_t nJ>
struct IKCache {
    cobalt::math::linear_algebra::Matrix<nJ, 6> lastJ_pinv6;
    cobalt::math::linear_algebra::Matrix<nJ, 3> lastJ_pinv3;
};


// --------------------------------------
//           Inverse Kinematics
// --------------------------------------

/**
 *  @brief IK solver for robots with arbitrary kinematic structure
 *  @tparam nL Number of links
 *  @tparam nJ Number of joints
 *  @tparam nE Number of end-effector frames
 */
template<id_t nL, id_t nJ, id_t nE>
class InverseKinematics {   
    private:
        // ---- Members ----
        Robot<nL, nJ, nE> &robot_;
        IKConfig config_;
        JacobianBuilder<nL, nJ, nE> jacobian_;
        ForwardKinematics<nL, nJ, nE> fk_;
        IKCache<nJ> cache_;

        float currentLambda_ = IK_DEFAULT_DAMPING_DLS;
        float lastErrNorm_ = std::numeric_limits<float>::max();


        // ---------------- Helper Functions ----------------
        /**
         *  @brief Compute 6-DOF error between target and current frame
         *  @param dst Target transform
         *  @param src Current transform
         *  @return 6-vector error
         */
        inline cobalt::math::linear_algebra::Vector<6> computeError(
            const cobalt::math::geometry::Transform<> &dst, 
            const cobalt::math::geometry::Transform<> &src) {
            
            cobalt::math::linear_algebra::Vector<6> err = cobalt::math::linear_algebra::Vector<6>::zero();

            // Position error
            cobalt::math::linear_algebra::Vector<3> posErr = dst.translation() - src.translation();

            // Orientation error (shortest path quaternion difference)
            cobalt::math::geometry::Quaternion<> dstQ = normalize(dst.rotation());
            cobalt::math::geometry::Quaternion<> srcQ = normalize(src.rotation());
            cobalt::math::geometry::Quaternion<> qErr = cobalt::math::geometry::shortestPath(srcQ, dstQ);
            cobalt::math::linear_algebra::Vector<3> rotErr = cobalt::math::geometry::toRotationVector(qErr);

            // Combine
            err[0] = posErr.x();
            err[1] = posErr.y();
            err[2] = posErr.z();
            err[3] = rotErr.x();
            err[4] = rotErr.y();
            err[5] = rotErr.z();

            return err;
        }

        /**
         *  @brief Extract task-space error for given IK mode
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Vector<M> extractTaskError(
            const cobalt::math::linear_algebra::Vector<6> &err, 
            IKMode mode) {
            
            cobalt::math::linear_algebra::Vector<M> taskErr = cobalt::math::linear_algebra::Vector<M>::zero();

            switch(mode) {
                case(IKMode::Position): {
                    assert(M == 3 && "Position IK uses a 3-Vector error");
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
         *  @brief Extract task-space jacobian for given IK mode
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<M, nJ> extractTaskJacobian(
            const cobalt::math::linear_algebra::Matrix<6, nJ> &J, 
            IKMode mode) {
            
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
         *  @brief Compute adaptive damping using Levenberg-Marquardt strategy
         *  @param errNorm Current error norm
         *  @param prevErrNorm Previous error norm
         *  @param lambda Current damping factor
         *  @return Updated damping factor
         */
        inline float computeAdaptiveDamping(float errNorm, float prevErrNorm, float lambda) {
            float rho = (prevErrNorm - errNorm) / (prevErrNorm * lambda);
            
            if(rho > IK_DLS_LM_RHO_GOOD) {
                return std::max(lambda * IK_DLS_LM_DECREASE, config_.dampingMin);
            }
            else if(rho < IK_DLS_LM_RHO_BAD) {
                return std::min(lambda * IK_DLS_LM_INCREASE, config_.dampingMax);
            }

            return lambda;
        }

        /**
         *  @brief Estimate condition number using Gershgorin circle theorem
         *  @param A Square matrix to analyze
         *  @return Approximate condition number
         *  @note Cond Number approximation without the SVD bloat
         */
        template<cobalt::math::index_t N>
        inline float estimateConditionNumber(const cobalt::math::linear_algebra::Matrix<N, N> &A) {
            float maxRadius = 0.0f;
            float minRadius = std::numeric_limits<float>::max();
            
            for(cobalt::math::index_t i = 0; i < N; i++) {
                // Compute row sum (Gershgorin radius)
                float radius = 0.0f;
                for(cobalt::math::index_t j = 0; j < N; j++) {
                    if(i != j) {
                        radius += std::abs(A(i, j));
                    }
                }
                
                // Eigenvalue bounds: |A(i,i)| ± radius
                float diskMax = std::abs(A(i, i)) + radius;
                float diskMin = std::abs(A(i, i)) - radius;
                
                maxRadius = std::max(maxRadius, diskMax);
                minRadius = std::min(minRadius, std::max(diskMin, 0.0f));
            }
            
            // Condition number estimate
            return (minRadius > cobalt::math::epsilon_<>) ?(maxRadius / minRadius) :(std::numeric_limits<float>::max());
        }

        /**
         *  @brief Compute per joint scaling for damping
         *  @return Scaling factor for each joints motion
         */
        inline cobalt::math::linear_algebra::Vector<nJ> computeJointScaling() const {
            cobalt::math::linear_algebra::Vector<nJ> scale;
            
            for(id_t j = 0; j < nJ; j++) {
                float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                scale[j] = 1.0f / (range * range);
            }
            
            return scale;
        }

        /**
         *  @brief Compute damped least-squares pseudo-inverse
         *  @param J Task jacobian
         *  @param errNorm Current error magnitude
         *  @param noProgCount Iterations without progress
         *  @param isSingular Output flag indicating singularity
         *  @return DLS pseudo-inverse J†
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInvDLS(
            const cobalt::math::linear_algebra::Matrix<M, nJ> &J, 
            float errNorm, 
            iter_t noProgCount,
            bool &isSingular) {
            
            currentLambda_ = computeAdaptiveDamping(errNorm, lastErrNorm_, currentLambda_);
            lastErrNorm_ = errNorm;
            
            float lambda = currentLambda_ * (1.0f + noProgCount * IK_DLS_LAMBDA_NOPROG_MULT);
            lambda = std::clamp(lambda, config_.dampingMin, config_.dampingMax);
            float lambdaSqr = lambda * lambda;

            cobalt::math::linear_algebra::Vector<nJ> scaling = computeJointScaling();

            if constexpr (nJ >= M) {    
                // Wide J: J† = Jᵀ(JJᵀ + λ²I)⁻¹
                cobalt::math::linear_algebra::Matrix<M, M> A = J * transpose(J);
                A += cobalt::math::linear_algebra::Matrix<M, M>::eye() * lambdaSqr;

                float condNum = estimateConditionNumber(A);
                if(condNum > config_.singularityThreshold) {
                    isSingular = true;
                }

                cobalt::math::linear_algebra::Matrix<M, M> L;
                if(!cobalt::math::linear_algebra::cholesky(A, L)) { 
                    isSingular = true;
                    if constexpr (M == 3) { return cache_.lastJ_pinv3; }
                    else                  { return cache_.lastJ_pinv6; }
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv;
                for(cobalt::math::index_t j = 0; j < nJ; j++) {
                    cobalt::math::linear_algebra::Vector<M> b = 
                        cobalt::math::linear_algebra::getRow(cobalt::math::linear_algebra::transpose(J), j);
                    cobalt::math::linear_algebra::Vector<M> x;

                    if(!cobalt::math::linear_algebra::solvePSD(A, b, x)) {
                        isSingular = true;
                        return cobalt::math::linear_algebra::transpose(J);
                    }

                    for(cobalt::math::index_t i = 0; i < M; i++) {
                        J_pinv(j, i) = x[i] * scaling[j];
                    }
                }

                return J_pinv;
            }
            else {                      
                // Tall J: J† = (JᵀJ + λ²I)⁻¹Jᵀ
                cobalt::math::linear_algebra::Matrix<nJ, nJ> A = transpose(J) * J;
                A += cobalt::math::linear_algebra::Matrix<nJ, nJ>::eye() * lambdaSqr;

                float condNum = estimateConditionNumber(A);
                if(condNum > config_.singularityThreshold) {
                    isSingular = true;
                }

                cobalt::math::linear_algebra::Matrix<nJ, nJ> L;
                if(!cobalt::math::linear_algebra::cholesky(A, L)) { 
                    isSingular = true;
                    return cobalt::math::linear_algebra::transpose(J);
                }

                cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv;
                for(cobalt::math::index_t j = 0; j < M; j++) {
                    cobalt::math::linear_algebra::Vector<nJ> b = 
                        cobalt::math::linear_algebra::getColumn(cobalt::math::linear_algebra::transpose(J), j);
                    cobalt::math::linear_algebra::Vector<nJ> x;

                    if(!cobalt::math::linear_algebra::solvePSD(A, b, x)) {
                        isSingular = true;
                        if constexpr (M == 3) { return cache_.lastJ_pinv3; }
                        else                  { return cache_.lastJ_pinv6; }
                    }

                    for(cobalt::math::index_t i = 0; i < nJ; i++) {
                        J_pinv(i, j) = x[i] * scaling[i];
                    }
                }

                return J_pinv;
            }
        }

        /**
         *  @brief Compute SVD-based pseudo-inverse with directional damping
         *  @param J Task-jacobian of the current robot state
         *  @param err Last error to be used in adaptive damping
         *  @param isSingular Output flag indicating if a singularity was detected
         *  @param isNearSingularity Output flag indicating if near-singularity was detected
         *  @param manip Output manipulability measure for current configuration
         *  @return nJx3/6 damped pseudo-inverse of the task jacobian
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInvSVD(
            const cobalt::math::linear_algebra::Matrix<M, nJ> &J,  
            cobalt::math::linear_algebra::Vector<M> err, 
            bool &isSingular, 
            bool &isNearSingularity, 
            float &manip) {
            
            cobalt::math::linear_algebra::Matrix<M, M> U;
            cobalt::math::linear_algebra::Matrix<M, nJ> S;
            cobalt::math::linear_algebra::Matrix<nJ, nJ> V;

            cobalt::math::linear_algebra::svd(J, U, S, V);

            cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv = cobalt::math::linear_algebra::Matrix<nJ, M>::zero();

            manip = 1.0f;
            const float sigMax = S(0, 0);
            cobalt::math::index_t r = (M < nJ) ? M : nJ;
            
            for(cobalt::math::index_t i = 0; i < r; i++) {
                const float sig = S(i, i);
                manip *= sig;

                if(sig < config_.svdSigmaMin * sigMax) {
                    isSingular = true;
                    continue;
                } 

                if(sig < config_.svdSigmaNear * sigMax || manip < config_.manipThreshold) {
                    isNearSingularity = true;
                } 
                
                // Directional damping
                float ue = cobalt::math::linear_algebra::dot(cobalt::math::linear_algebra::getColumn(U, i), err);
                float lambda_i = config_.svdDirDamping * ue * ue;
                const float dampedSig = sig / (sig * sig + lambda_i * lambda_i);

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
         *  @param escapeAttempt Current escape attempt count for deterministic perturbation
         *  @return Perturbed joint configuration to escape singularity
         */
        inline cobalt::math::linear_algebra::Vector<nJ> escapeSingularity(
            const cobalt::math::linear_algebra::Vector<nJ> &q, 
            iter_t escapeAttempt) {
            
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
         *  @param q Current joint configuration
         *  @param dq Proposed joint velocity step
         *  @param target IK target for error evaluation
         *  @param err Current error magnitude to compare against for sufficient decrease
         *  @return Step size alpha that satisfies Armijo condition or minimum step size if none found
         */
        template<cobalt::math::index_t M>
        inline float getStepSize(
            const cobalt::math::linear_algebra::Vector<nJ> &q, 
            cobalt::math::linear_algebra::Vector<nJ> &dq, 
            const IKTarget &target, 
            float err) {
            
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;
            float alpha = config_.step;

            for(iter_t i = 0; i < IK_BACKTRACK_NUM; i++) {
                cobalt::math::linear_algebra::Vector<nJ> q_prime = q + alpha * dq;
                robot_.setJoints(q_prime);
                fk_.solve(robot_.state());

                cobalt::math::linear_algebra::Vector<6> err_test = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
                cobalt::math::linear_algebra::Vector<6> err_test_W = cobalt::math::linear_algebra::Matrix<6, 6>::diagonal(target.weight) * err_test;
                cobalt::math::linear_algebra::Vector<M> taskErr_test = extractTaskError<M>(err_test_W, target.mode);

                float errNorm = norm(taskErr_test);

                if(errNorm <= err - IK_ARMIJO_COND_CONST * alpha * norm(dq)) {
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
         *  @brief Enforce velocity limits on proposed joint velocity step
         *  @param dq Proposed joint velocity step
         *  @param dt Time step to evaluate velocity against limits
         *  @return Scaled joint velocity step that respects velocity limits
         *  @note If velocity limits are disabled or dt is too small, returns original dq without modification
         */
        inline cobalt::math::linear_algebra::Vector<nJ> enforceVelocityLimits(const cobalt::math::linear_algebra::Vector<nJ> &dq, float dt) const {
            if(dt <= cobalt::math::epsilon_<> || !config_.enforceVelocityLimits) {
                return dq;
            }

            cobalt::math::linear_algebra::Vector<nJ> dq_lim = dq;

            for(cobalt::math::index_t j = 0; j < nJ; j++) {
                const Joint &joint = robot_.model().getJoints()[j];

                if(!joint.isVelocityLimitEnabled()) { continue; }

                float velocity = dq[j]/dt;

                float maxVel = joint.getVelocityLimit();
                float velClamp = std::clamp(velocity, -maxVel, maxVel);

                dq_lim[j] = velClamp * dt;
            }

            return dq_lim;
        }

        /**
         *  @brief Project joint velocities away from limits
         *  @param q Current joint configuration
         *  @param dq Proposed joint velocity step to be modified
         *  @param step Proposed step size to evaluate future configuration against limits
         */
        inline void projectAwayFromLimits( const cobalt::math::linear_algebra::Vector<nJ> &q, cobalt::math::linear_algebra::Vector<nJ> &dq, float step) {
            
            for(id_t j = 0; j < nJ; j++) {
                float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                float upperMargin = robot_.model().getJoints()[j].getMaxLimit() - config_.projectMargin * range;
                float lowerMargin = robot_.model().getJoints()[j].getMinLimit() + config_.projectMargin * range;

                if((q[j] + dq[j] * step > upperMargin) && (dq[j] > 0)) { dq[j] = 0.0f; }
                if((q[j] + dq[j] * step < lowerMargin) && (dq[j] < 0)) { dq[j] = 0.0f; }
            }
        }

        /**
         *  @brief Solve IK for given task mode
         *  @param target IK target containing desired pose and mode
         *  @tparam M Task dimension
         */
        template<cobalt::math::index_t M>
        inline IKSolution<nJ> solveTask(const IKTarget &target) {
            // Save initial state
            cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;

            // Initialize output
            IKSolution<nJ> output;
            output.status = IKStatus::MaxIterations;
            output.q = robot_.state().q;
            output.error = cobalt::math::linear_algebra::Vector<6>::zero();
            output.iterations = 0;

            // Validate frame ID
            if(target.frameId >= nE) {
                output.status = IKStatus::InvalidFrame;
                return output;
            }

            // Reset damping state
            currentLambda_ = config_.dampingDLS;
            lastErrNorm_ = std::numeric_limits<float>::max();

            // Progress tracking
            float prevErr = std::numeric_limits<float>::max();
            float minErr = std::numeric_limits<float>::max();
            iter_t noProgCount = 0;
            iter_t smallProgCount = 0;
            iter_t singularCount = 0;

            // Error weighting
            cobalt::math::linear_algebra::Matrix<6, 6> W = cobalt::math::linear_algebra::Matrix<6, 6>::diagonal(target.weight);

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

                // Check convergence
                if(errNorm < config_.threshold) {
                    output.status = IKStatus::Success;
                    output.iterations = iter;
                    output.q = robot_.state().q;
                    break;
                }
                
                // Track progress
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

                // Compute Jacobian with weights
                cobalt::math::linear_algebra::Matrix<6, nJ> J = jacobian_.compute(robot_.state(), target.frameId);
                cobalt::math::linear_algebra::Matrix<6, nJ> J_W = W * J;
                cobalt::math::linear_algebra::Matrix<M, nJ> J_task = extractTaskJacobian<M>(J_W, target.mode);

                bool isSingular = false;
                bool isNearSingularity = false;
                cobalt::math::linear_algebra::Matrix<nJ, M> J_pinv;

                // Compute pseudo-inverse
                switch(config_.solver) {
                    case(IKSolver::DLS): {
                        J_pinv = computePseudoInvDLS(J_task, errNorm, noProgCount, isSingular);
                        break;
                    }
                    case(IKSolver::SVD): {
                        float manip = 0.0f;
                        J_pinv = computePseudoInvSVD(J_task, taskErr, isSingular, isNearSingularity, manip);
                        break;
                    }
                    case(IKSolver::Mixed): {    // TODO: Janky and wrong math
                        float manip = 0.0f;
                        cobalt::math::linear_algebra::Matrix<nJ, M> J_dls = 
                            computePseudoInvDLS(J_task, errNorm, noProgCount, isSingular);
                        cobalt::math::linear_algebra::Matrix<nJ, M> J_svd = 
                            computePseudoInvSVD(J_task, taskErr, isSingular, isNearSingularity, manip);

                        float w = std::clamp(manip / config_.manipThreshold, 0.0f, 1.0f);
                        J_pinv = w * J_dls + (1.0f - w) * J_svd;
                        break;
                    }
                    default: { break; }
                }

                // Handle singularity
                if((isSingular || isNearSingularity) && noProgCount > IK_NOPROG_SOFT_THRESHOLD) {
                    if(singularCount >= IK_SINGULAR_ESCAPE_THRESHOLD) {
                        output.status = IKStatus::Singular;
                        output.iterations = iter;
                        output.q = robot_.state().q;
                        break;
                    }

                    escapeSingularity(robot_.state().q, singularCount);
                    singularCount++;
                    continue;
                }

                // Compute step
                cobalt::math::linear_algebra::Vector<nJ> delta_q = J_pinv * taskErr;
                float stepSize = getStepSize<M>(robot_.state().q, delta_q, target, errNorm);

                delta_q = enforceVelocityLimits(delta_q * stepSize, config_.timeStep);

                stepSize = 1.0f;
                projectAwayFromLimits(robot_.state().q, delta_q, stepSize);

                // Check step size
                if((norm(delta_q) * stepSize < IK_MIN_STEP_SIZE) && 
                   (errNorm > config_.threshold * config_.unreachableMult)) {
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

                // Update configuration
                cobalt::math::linear_algebra::Vector<nJ> q_new = robot_.state().q + delta_q * stepSize;
                robot_.setJoints(q_new);

                // Cache results
                if constexpr (M == 3) { cache_.lastJ_pinv3 = J_pinv; }
                else                  { cache_.lastJ_pinv6 = J_pinv; }
                robot_.state().dq = delta_q;
                robot_.state().J = J;
            }

            // Handle max iterations
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
                const cobalt::math::geometry::Transform<> T_final = 
                    robot_.state().frameTransforms[target.frameId];
                output.error = computeError(target.pose, T_final);
            }

            // Restore initial state
            robot_.setJoints(q_init);
            fk_.solve(robot_.state());

            return output;
        }

    public:
        /**
         *  @brief Construct inverse kinematics solver
         *  @param robot Robot to solve IK for
         *  @param config Solver configuration
         */
        explicit InverseKinematics(Robot<nL, nJ, nE> &robot, IKConfig config) 
            : robot_(robot), config_(config), jacobian_(robot), fk_(robot), cache_({}) {}

        /**
         *  @brief Get mutable reference to solver configuration
         */
        constexpr IKConfig &config() { return config_; } 

        /**
         *  @brief Get const reference to solver configuration
         */
        constexpr const IKConfig &config() const { return config_; } 

        /**
         *  @brief Solve inverse kinematics for target
         *  @param target Target pose and mode
         *  @return Solution containing joint angles, error, and status
         */
        IKSolution<nJ> solve(const IKTarget &target) {
            switch (target.mode) {
                case (IKMode::Position): 
                    return solveTask<3>(target);
                case (IKMode::Orientation): 
                    return solveTask<3>(target);
                case (IKMode::Pose): 
                    return solveTask<6>(target);
                default: {
                    IKSolution<nJ> invalidOutput;
                    invalidOutput.status = IKStatus::InvalidMode;
                    invalidOutput.iterations = 0;
                    return invalidOutput;
                }
            }
        }
};

} // cobalt::kinematics::solvers