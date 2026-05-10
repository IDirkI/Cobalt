#pragma once

#include <limits>
#include <cmath>
#include <algorithm>

#include "jacobian_builder.hpp"

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/solvers/forward_kinematics.hpp"
#include "cobalt/kinematics/solvers/inverse_kinematics/jacobian_builder.hpp"

namespace cobalt::kinematics::solvers {

// ---------------- Enums ----------------
/**
 *  @brief Status of IK solver result
 */
enum class IKStatus : uint8_t {
    Success,          // Converged successfully
    MaxIterations,    // Hit iteration limit
    Unreachable,      // Target is unreachable
    Singular,         // Stuck in singularity
    InvalidInput,     // Bad inputs
};

/**
 *  @brief IK solving mode
 */
enum class IKMode : uint8_t {
    Position,     // Position-only (3-DOF)
    Pose,         // Full pose (6-DOF)
};

/**
 *  @brief IK solver algorithm
 */
enum class IKSolver : uint8_t {
    DLS,    // Damped Least Squares
    SVD,    // Singular Value Decomposition
};

// ---------------- Constants ----------------
// Convergence 
constexpr float IK_MIN_STEP_SIZE    = 1e-7f;
constexpr float IK_MIN_ERROR_CHANGE = 1e-5f;

// Armijo Line Search
constexpr float IK_BACKTRACK_FACTOR     = 0.5f;
constexpr iter_t IK_BACKTRACK_NUM       = 10;
constexpr float IK_ARMIJO_COND_CONST    = 0.1f;

// Tracking
constexpr iter_t IK_NOPROG_THRESHOLD        = 100;
constexpr iter_t IK_NOPROG_SOFT_THRESHOLD   = 20;
constexpr iter_t IK_SMALLPROG_THRESHOLD     = 10;

// Singularities
constexpr float IK_SINGULARITY_THRESHOLD    = 100.0f;
constexpr float IK_MANIP_THRESHOLD          = 1e-4f;
constexpr float IK_ESCAPE_FACTOR            = 0.1f;
constexpr iter_t IK_ESCAPE_TRIES            = 5;

// DLS
constexpr float IK_DAMPING_INIT         = 1e-2f;
constexpr float IK_DAMPING_MIN          = 1e-5f;
constexpr float IK_DAMPING_MAX          = 1e-1f;
constexpr float IK_DAMPING_NOPROG_MULT  = 0.1f;
constexpr float IK_LM_RHO_GOOD          = 0.75f;
constexpr float IK_LM_RHO_BAD           = 0.25f;
constexpr float IK_LM_DECREASE          = 0.5f;
constexpr float IK_LM_INCREASE          = 2.0f;

// SVD
constexpr float IK_SVD_SIGMA_MIN        = 1e-3f;
constexpr float IK_SVD_SIGMA_NEAR       = 1e-2f;
constexpr float IK_SVD_DIR_DAMPING      = 1.0f;

// Config Defaults
constexpr IKSolver  IK_DEFAULT_SOLVER       = IKSolver::DLS;
constexpr iter_t    IK_DEFAULT_MAX_ITER     = 50;
constexpr float     IK_DEFAULT_THRESHOLD    = 1e-3f;
constexpr float     IK_DEFAULT_STEP         = 1.0f;
constexpr float     IK_DEFAULT_MARGIN       = 0.05f;
constexpr float     IK_DEFAULT_UNREACH      = 5.0f;
constexpr float     IK_DEFAULT_TIMESTEP     = 0.0;

// ---------------- Structs ----------------
/**
 *  @brief Struct representing an IK target for a specific frame in the robot
 *  @param frameId ID of the target frame
 *  @param mode IKMode specifying whether this target is position-only or full pose
 *  @param pose Desired target pose for the frame
 *  @param weight Optional weighting for the target error (defaults to all 1's)
 */
struct IKTarget {
    id_t frameId;
    IKMode mode;
    cobalt::math::geometry::Transform<> pose;
    cobalt::math::linear_algebra::Vector<6> weight{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
};

/**
 *  @brief Struct representing the result of an IK solve
 *  @param q Configuration vector solution for the robot joints
 *  @param err Final error vector for the target (position and orientation)
 *  @param iterations Number of iterations taken to converge
 *  @param status IKStatus indicating the result of the solve
 *  @tparam nJ Number of joints in the robot (size of configuration vector)
 */
template<id_t nJ>
struct IKSolution {
    cobalt::math::linear_algebra::Vector<nJ> q;
    cobalt::math::linear_algebra::Vector<6> err;
    iter_t iterations;
    IKStatus status;
};

/**
 *  @brief Struct representing the configuration parameters for the IK solver
 *  @param solver IKSolver algorithm to use
 * 
 *  @param maxIterations Maximum number of iterations before giving up
 *  @param threshold Error norm threshold for convergence
 * 
 *  @param step Step size multiplier for each iteration
 *  @param dampingDLS Initial damping factor for DLS solver
 *  @param dampingMin Minimum damping factor for DLS solver
 *  @param dampingMax Maximum damping factor for DLS solver
 *  @param projectMargin Margin to keep from joint limits when projecting
 * 
 *  @param timestep Optional timestep for velocity control (if 0, velocity limits are ignored)
 *  @param enforceVelocityLimits Whether to enforce velocity limits when timestep is set
 * 
 *  @param singularityThreshold Threshold for detecting singularities (based on Jacobian condition number)
 *  @param manipThreshold Threshold for detecting loss of manipulability (based on Jacobian determinant)
 * 
 *  @param svdSigmaMin Minimum singular value for SVD damping
 *  @param svdSigmaNear Singular value threshold for near-singularity in SVD damping
 *  @param svdDirectionalDamp Damping factor to apply in the direction of the smallest singular value in SVD damping
 *  
 *  @param unreachableMult Multiplier to apply to step size when target is detected as unreachable to encourage escape from local minima
 */
struct IKConfig {
    IKSolver solver = IK_DEFAULT_SOLVER;

    iter_t maxIterations   = IK_DEFAULT_MAX_ITER;
    float  threshold       = IK_DEFAULT_THRESHOLD;

    float step          = IK_DEFAULT_STEP;
    float dampingDLS    = IK_DAMPING_INIT;
    float dampingMin    = IK_DAMPING_MIN;
    float dampingMax    = IK_DAMPING_MAX;
    float projectMargin = IK_DEFAULT_MARGIN;

    float timestep                = IK_DEFAULT_TIMESTEP;
    bool  enforceVelocityLimits   = false;

    float singularityThreshold  = IK_SINGULARITY_THRESHOLD;
    float manipThreshold        = IK_MANIP_THRESHOLD;

    float svdSigmaMin        = IK_SVD_SIGMA_MIN;
    float svdSigmaNear       = IK_SVD_SIGMA_NEAR;
    float svdDirectionalDamp = IK_SVD_DIR_DAMPING;

    float unreachableMult = IK_DEFAULT_UNREACH;
};


// --------------------------------------
//          Inverse Kinematics
// --------------------------------------
/**
 *  @brief Class for solving inverse kinematics for a robot using iterative numerical methods
 *  @tparam nL Number of links in the robot
 *  @tparam nJ Number of joints in the robot
 *  @tparam nE Number of end effectors (frames with targets) in the robot
 */
template<id_t nL, id_t nJ, id_t nE>
class InverseKinematics {
    private:
        Robot<nL, nJ, nE> &robot_;
        ForwardKinematics<nL, nJ, nE> fk_;
        JacobianBuilder<nL, nJ, nE> jacobian_;
        IKConfig config_;

        float lambda_;
        float lastErrNorm_;

        cobalt::math::linear_algebra::Matrix<nJ, 6> pinv6Cached_;
        cobalt::math::linear_algebra::Matrix<nJ, 3> pinv3Cached_;

        // ---------------- Helper Functions ----------------
        // --- Error ---
        /**
         *  @brief Compute the error vector between the current pose and target pose for a given frame
         *  @param target Desired target pose for the frame
         *  @param curr Current pose of the frame based on the current robot configuration
         *  @return 6D error vector where the first 3 elements are position error and the last 3 elements are orientation error represented as a rotation vector 
         */
        inline cobalt::math::linear_algebra::Vector<6> computeError(const cobalt::math::geometry::Transform<> &target, const cobalt::math::geometry::Transform<> &curr) const {
            cobalt::math::linear_algebra::Vector<6> err;

            cobalt::math::linear_algebra::Vector<3> posErr = target.translation() - curr.translation();

            cobalt::math::geometry::Quaternion rotTarget =  cobalt::math::geometry::normalize(target.rotation());
            cobalt::math::geometry::Quaternion rotCurr =  cobalt::math::geometry::normalize(curr.rotation());
            cobalt::math::geometry::Quaternion rotDiff =  cobalt::math::geometry::shortestPath(rotCurr, rotTarget);
            cobalt::math::linear_algebra::Vector<3> rotErr = cobalt::math::geometry::toRotationVector(rotDiff);

            err[0] = posErr[0];
            err[1] = posErr[1];
            err[2] = posErr[2];
            err[3] = rotErr[0];
            err[4] = rotErr[1];
            err[5] = rotErr[2];

            return err;
        }

        /**
         *  @brief Extract the relevant portion of the error vector based on the IKMode of the target (position-only or full pose)
         *  @param err Full 6D error vector for the target
         *  @tparam M Dimensionality of the extracted error vector (3 for position-only, 6 for full pose)
         *  @return Extracted error vector of size M
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Vector<M> extractError(const cobalt::math::linear_algebra::Vector<6> &err) const {
            if constexpr (M == 6) { return err; }
            else { return cobalt::math::linear_algebra::Vector<3> { err[0], err[1], err[2] }; }
        }

        /**
         *  @brief Extract the relevant portion of the Jacobian matrix based on the IKMode of the target (position-only or full pose)
         *  @param J Full 6xN Jacobian matrix for the target
         *  @param M Dimensionality of the extracted Jacobian matrix (3 for position-only, 6 for full pose)
         *  @return Extracted Jacobian matrix of size MxN
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<M, nJ> extractJacobian(const cobalt::math::linear_algebra::Matrix<6, nJ> &J) const {
            if constexpr (M == 6) { return J; }
            else { return J.template block<3, nJ>(0,0); }
        }

        // --- Damping ---
        /**
         *  @brief Compute the adaptive damping factor for DLS solver based on the progress of the error norm reduction
         *  @param errNorm Current error norm
         *  @param prevErrNorm Previous error norm from the last iteration
         *  @param lambda Current damping factor
         *  @return Updated damping factor for the next iteration
         */
        inline float adaptiveDamp(float errNorm, float prevErrNorm, float lambda) {
            if(prevErrNorm >= std::numeric_limits<float>::max()) { return lambda; }

            const float rho = (prevErrNorm - errNorm) / (prevErrNorm * lambda);

            if(rho > IK_LM_RHO_GOOD) {
                return std::max(lambda * IK_LM_DECREASE, config_.dampingMin);
            }
            if(rho < IK_LM_RHO_BAD) {
                return std::min(lambda * IK_LM_INCREASE, config_.dampingMax);
            }

            return lambda;
        }

        /**
         *  @brief Estimate the condition number of a matrix using the infinity norm and its inverse (if available) to detect singularities in the Jacobian
         *  @param A Matrix to estimate the condition number of
         *  @return Estimated condition number of A
         *  @note This is a computationally cheaper alternative to computing the full SVD for condition number estimation, but it is less accurate. It relies on the fact that for a matrix A, cond(A) = ||A|| * ||A^-1||, and uses the infinity norm for ||A||. If the inverse is not available (e.g. due to singularity), it returns a very large number to indicate a high condition number.
         */
        template<cobalt::math::index_t N>
        inline float conditionNum(const cobalt::math::linear_algebra::Matrix<N,N> &A) const {
            float maxR = 0.0f;
            float minR = std::numeric_limits<float>::max();

            for(cobalt::math::index_t i = 0; i < N; i++) {
                float offDiag = 0.0f;
                for(cobalt::math::index_t j = 0; j < N; j++) {
                    if(i != j) { offDiag += std::abs(A(i,j)); }
                }

                const float dMax = std::abs(A(i,i)) + offDiag;
                const float dMin = std::abs(A(i,i)) - offDiag;
                maxR = std::max(maxR, dMax);
                minR = std::min(minR, std::max(dMin, 0.0f));
            }

            return (minR > cobalt::math::epsilon_<>) ?(maxR/minR) :(std::numeric_limits<float>::max());
        }

        // --- Cache ---
        /**
         *  @brief Get the cached pseudoinverse matrix for the given dimensionality (3 for position-only, 6 for full pose) to speed up repeated solves when the Jacobian structure does not change significantly
         *  @tparam M Dimensionality of the pseudoinverse matrix (3 for position-only, 6 for full pose)
         *  @return Cached pseudoinverse matrix of size nJxM
         *  @note The cache is updated after each successful computation of the pseudoinverse, so it may not always be valid if the Jacobian changes significantly between iterations. It is intended as a performance optimization for cases where the Jacobian structure remains relatively stable across iterations, such as when the robot is near a solution or when the targets do not change drastically.
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ,M> &getCachedPseudoInv() {
            if constexpr (M == 6)   { return pinv6Cached_; }
            else                    { return pinv3Cached_; }
        }

        /**
         *  @brief Set the cached pseudoinverse matrix for the given dimensionality (3 for position-only, 6 for full pose) after a successful computation to speed up future iterations
         *  @param pinv Newly computed pseudoinverse matrix to cache
         *  @tparam M Dimensionality of the pseudoinverse matrix (3 for position-only, 6 for full pose)
         *  @note The cache should only be updated after a successful computation of the pseudoinverse, and it may not always be valid if the Jacobian changes significantly between iterations. It is intended as a performance optimization for cases where the Jacobian structure remains relatively stable across iterations, such as when the robot is near a solution or when the targets do not change drastically.
         */
        template<cobalt::math::index_t M>
        inline void setCachedPseudoInv(const cobalt::math::linear_algebra::Matrix<nJ, M> &pinv) {
            if constexpr (M == 6)   { pinv6Cached_ = pinv; }
            else                    { pinv3Cached_ = pinv; }
        }

        // --- Pseudoinverse(DLS) ---
        /**
         *  @brief Compute the pseudoinverse of the Jacobian matrix using Damped Least Squares (DLS) method with adaptive damping based on the error norm reduction to handle singularities and improve convergence
         *  @param J Jacobian matrix for the current target and robot configuration 
         *  @param errNorm Current error norm for the target
         *  @param noProgCount Number of consecutive iterations with little to no error reduction to increase damping and encourage escape from local minima
         *  @param isSingular Output flag indicating whether the Jacobian is near singular based on the condition number estimation, which can be used by the solver to take appropriate actions such as increasing damping or escaping local minima
         *  @return Pseudoinverse of the Jacobian matrix computed using DLS method, which can be used to compute the joint updates for the IK iteration
         *  @note The damping factor is adapted based on the progress of error norm reduction, and it is also increased when there are consecutive iterations with little to no error reduction to encourage escape from local minima. The function also estimates the condition number of the matrix involved in the DLS computation to detect singularities, and it returns a flag indicating whether the Jacobian is near singular. If the matrix is detected as singular or if the Cholesky decomposition fails, it returns the cached pseudoinverse from the previous iteration to provide a fallback solution.
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInv_DLS(
            const cobalt::math::linear_algebra::Matrix<M, nJ> &J, 
            float errNorm,
            iter_t noProgCount,
            bool &isSingular) {
                lambda_ = adaptiveDamp(errNorm, lastErrNorm_, lambda_);
                lastErrNorm_ = errNorm;

                float lambdaNoProg = lambda_ * (1.0f + noProgCount * IK_DAMPING_NOPROG_MULT);
                lambdaNoProg = std::clamp(lambdaNoProg, config_.dampingMin, config_.dampingMax);
                const float lambdaNoProg_sqr = lambdaNoProg * lambdaNoProg;

                const cobalt::math::linear_algebra::Matrix<nJ, M> &cachedPseudoInv = getCachedPseudoInv<M>();
                const cobalt::math::linear_algebra::Matrix<nJ, M> Jt = cobalt::math::linear_algebra::transpose(J);

                if constexpr (nJ >= M) {    // Wide
                    cobalt::math::linear_algebra::Matrix<M, M> A = J * Jt;
                    A += cobalt::math::linear_algebra::Matrix<M, M>::eye() * lambdaNoProg_sqr;

                    if(conditionNum(A) > config_.singularityThreshold) { isSingular = true; }

                    cobalt::math::linear_algebra::Matrix<M, M> L;
                    if(!cobalt::math::linear_algebra::cholesky(A,L)) {
                        isSingular = true;
                        return cachedPseudoInv;
                    }

                    cobalt::math::linear_algebra::Matrix<nJ, M> pinvJ;
                    
                    for(cobalt::math::index_t j = 0; j < nJ; j++) {
                        cobalt::math::linear_algebra::Vector<M> b = cobalt::math::linear_algebra::getRow(Jt, j);
                        cobalt::math::linear_algebra::Vector<M> x;

                        if(!cobalt::math::linear_algebra::solveCholesky(L,b,x)) {
                            isSingular = true;
                            return cachedPseudoInv;
                        }

                        for(cobalt::math::index_t i = 0; i < M; i++) {
                            pinvJ(j,i) = x[i];
                        }
                    }

                    return pinvJ;
                }
                else {      // Tall
                    cobalt::math::linear_algebra::Matrix<nJ, nJ> A = Jt*J;
                    A += cobalt::math::linear_algebra::Matrix<nJ, nJ>::eye() * lambdaNoProg_sqr;

                    if(conditionNum(A) > config_.singularityThreshold) { isSingular = true; }

                    cobalt::math::linear_algebra::Matrix<nJ, nJ> L;
                    if(!cobalt::math::linear_algebra::cholesky(A,L)) {
                        isSingular = true;
                        return Jt;
                    }

                    cobalt::math::linear_algebra::Matrix<nJ, M> pinvJ;

                    for(cobalt::math::index_t j = 0; j < M; j++) {
                        cobalt::math::linear_algebra::Vector<nJ> b = cobalt::math::linear_algebra::getColumn(Jt, j);
                        cobalt::math::linear_algebra::Vector<nJ> x;

                        if(!cobalt::math::linear_algebra::solveCholesky(L,b,x)) {
                            isSingular = true;
                            return cachedPseudoInv;
                        }

                        for(cobalt::math::index_t i = 0; i < nJ; i++) {
                            pinvJ(i,j) = x[i];
                        }
                    }

                    return pinvJ;
                }
            }
        // --- Pseudoinverse(SVD) ---
        /**
         *  @brief Compute the pseudoinverse of the Jacobian matrix using Singular Value Decomposition (SVD) method with damping based on the singular values to handle singularities and improve convergence
         *  @param J Jacobian matrix for the current target and robot configuration
         *  @param err Full 6D error vector for the target, which is used to compute the directional damping based on the projection of the error onto the singular vectors corresponding to the small singular values
         *  @param isSingular Output flag indicating whether the Jacobian is near singular based on the smallest singular value, which can be used by the solver to take appropriate actions such as increasing damping or escaping local minima
         *  @param isNearSingularity Output flag indicating whether the Jacobian is near a singularity based on the ratio of the smallest singular value to the largest singular value and the manipulability measure, which can be used by the solver to take appropriate actions such as increasing damping or escaping local minima
         *  @param manip Output variable to store the manipulability measure (product of singular values) of the Jacobian, which can be used by the solver to detect loss of manipulability and take appropriate actions such as increasing damping or escaping local minima
         *  @return Pseudoinverse of the Jacobian matrix computed using SVD method, which can be used to compute the joint updates for the IK iteration
         *  @note The function computes the SVD of the Jacobian matrix to obtain its singular values and corresponding singular vectors. It then applies damping to the pseudoinverse computation based on the magnitude of the singular values, with more damping applied to directions corresponding to smaller singular values. The function also checks for singularities based on the smallest singular value and the ratio of the smallest to largest singular value, as well as the manipulability measure, and it sets output flags accordingly. The computed pseudoinverse is returned for use in computing joint updates for the IK iteration.
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInv_SVD(
            const cobalt::math::linear_algebra::Matrix<M, nJ> &J,
            const cobalt::math::linear_algebra::Vector<M> &err,
            bool &isSingular,
            bool &isNearSingularity,
            float &manip) {
                cobalt::math::linear_algebra::Matrix<M, M> U;
                cobalt::math::linear_algebra::Matrix<M,nJ> S;
                cobalt::math::linear_algebra::Matrix<nJ,nJ> V;
                cobalt::math::linear_algebra::svd(J, U, S, V);

                cobalt::math::linear_algebra::Matrix<nJ, M> pinvJ =  cobalt::math::linear_algebra::Matrix<nJ, M>::zero();

                manip = 1.0f;
                const float sigMax = S(0,0);
                const cobalt::math::index_t r = (M < nJ) ?M :nJ;

                for(cobalt::math::index_t i = 0; i < r; i++) {
                    const float sig = S(i,i);
                    manip *= sig;

                    if(sig < config_.svdSigmaMin * sigMax) {
                        isSingular = true;
                        continue;
                    }

                    if((sig < config_.svdSigmaNear * sigMax) || (manip < config_.manipThreshold)) {
                        isNearSingularity = true;
                    }

                    const cobalt::math::linear_algebra::Vector<M> ui = cobalt::math::linear_algebra::getColumn(U, i);
                    const float ue = cobalt::math::linear_algebra::dot(ui, err);
                    const float lambdaI = config_.svdDirectionalDamp * ue*ue;
                    const float dampedSigma = sig / (sig*sig + lambdaI*lambdaI);

                    for(cobalt::math::index_t j = 0; j < nJ; j++) {
                        for(cobalt::math::index_t k = 0; k < M; k++) {
                            pinvJ(j, k) += V(j,i) * dampedSigma * U(k, i);
                        }
                    }
                }

                return pinvJ;
            }

        // --- Pseudoinverse Picker ---
        /**
         *  @brief Compute the pseudoinverse of the Jacobian matrix using either DLS or SVD method based on the solver configuration, and also handle singularity detection and fallback to cached pseudoinverse if necessary
         *  @param J Jacobian matrix for the current target and robot configuration
         *  @param err Full 6D error vector for the target, which is used for computing the error norm and for directional damping in SVD method
         *  @param errNorm Current error norm for the target, which is used for adaptive damping in DLS method and for convergence checking
         *  @param noProgCount Number of consecutive iterations with little to no error reduction, which is used for increasing damping in DLS method to encourage escape from local minima
         *  @param isSingular Output flag indicating whether the Jacobian is near singular based on the condition number estimation for DLS method or the smallest singular value for SVD method, which can be used by the solver to take appropriate actions such as increasing damping or escaping local minima
         *  @param isNearSingularity Output flag indicating whether the Jacobian is near a singularity based on the ratio of the smallest singular value to the largest singular value and the manipulability measure for SVD method, which can be used by the solver to take appropriate actions such as increasing damping or escaping local minima
         *  @return Pseudoinverse of the Jacobian matrix computed using the selected method, which can be used to compute the joint updates for the IK iteration
         *  @note The function checks the solver configuration to determine whether to use DLS or SVD method for computing the pseudoinverse. It then calls the corresponding function to compute the pseudoinverse, while also handling singularity detection and fallback to cached pseudoinverse if necessary. The computed pseudoinverse is returned for use in computing joint updates for the IK iteration.
         */
        template<cobalt::math::index_t M>
        inline cobalt::math::linear_algebra::Matrix<nJ, M> computePseudoInv(
            const cobalt::math::linear_algebra::Matrix<M, nJ> &J,
            const cobalt::math::linear_algebra::Vector<M> &err,
            float errNorm,
            iter_t noProgCount,
            bool &isSingular,
            bool &isNearSignularity) {

                float manip = 0.0f;

                if(config_.solver == IKSolver::DLS) {
                    cobalt::math::linear_algebra::Matrix<nJ, M> pinvJ = computePseudoInv_DLS<M>(J, errNorm, noProgCount, isSingular);

                    if(isSingular) {    // SVD fallback on singular state
                        return computePseudoInv_SVD<M>(J, err, isSingular, isNearSignularity, manip);
                    }

                    return pinvJ;
                }
                else {
                    return computePseudoInv_SVD<M>(J, err, isSingular, isNearSignularity, manip);
                }
            }

        // --- Constraints ---
        /**
         *  @brief Project the joint updates away from the joint limits to prevent the solver from getting stuck at the limits and to encourage exploration of the configuration space, especially when the current configuration is near the limits
         *  @param q Current joint configuration vector
         *  @param dq Proposed joint update vector computed from the pseudoinverse and error
         *  @param step Step size multiplier for the joint update, which is used to determine how far along the proposed update the projection should consider for checking against the limits
         *  @note The function iterates through each joint and checks if applying the proposed update (scaled by the step size) would move the joint beyond a certain margin from its limits. If it would, and if the update is in the direction that would violate the limit, the update for that joint is set to zero to prevent moving further towards the limit. This helps to keep the solver away from joint limits and encourages it to explore other configurations that may lead to a solution, especially when the current configuration is near the limits.
         */
        inline void projectAwayFromLimits(const cobalt::math::linear_algebra::Vector<nJ> &q, cobalt::math::linear_algebra::Vector<nJ> &dq, float step) const {
            for(id_t j = 0; j < nJ; j++) {
                const float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                const float upperMargin = robot_.model().getJoints()[j].getMaxLimit() - config_.projectMargin * range;
                const float lowerMargin = robot_.model().getJoints()[j].getMinLimit() + config_.projectMargin * range;

                if((q[j] + dq[j]*step > upperMargin) && dq[j] > 0.0f) { dq[j] = 0.0f; }
                if((q[j] + dq[j]*step < lowerMargin) && dq[j] < 0.0f) { dq[j] = 0.0f; }
            }
        }

        /**
         *  @brief Apply velocity limits to the proposed joint updates based on the configured timestep and the velocity limits of each joint to ensure that the solver does not propose updates that would require exceeding the joint velocity limits, which can help to maintain safe and realistic motion of the robot, especially when the solver is used in a real-time control loop
         *  @param dq Proposed joint update vector computed from the pseudoinverse and error
         *  @param dt Timestep for the velocity limits, which is used to convert the proposed joint updates into velocities and check against the joint velocity limits
         *  @return Joint update vector after applying velocity limits, which may have some updates scaled down or set to zero if they would require exceeding the joint velocity limits
         *  @note The function checks if the timestep is greater than a small epsilon value and if velocity limit enforcement is enabled in the configuration. If so, it iterates through each joint and checks if the proposed update, when converted to a velocity by dividing by the timestep, would exceed the joint's velocity limits. If it would, the update for that joint is scaled down to the maximum allowed velocity (in the direction of the proposed update) multiplied by the timestep. This ensures that the proposed joint updates do not require exceeding the joint velocity limits, which can help to maintain safe and realistic motion of the robot, especially when the solver is used in a real-time control loop.
         */
        inline cobalt::math::linear_algebra::Vector<nJ> applyVelocityLimits(const cobalt::math::linear_algebra::Vector<nJ> &dq, float dt) const {
            if (dt <= cobalt::math::epsilon_<> || !config_.enforceVelocityLimits) { return dq; }

            cobalt::math::linear_algebra::Vector<nJ> dq_lim = dq;

            for(cobalt::math::index_t j = 0; j < nJ; j++) {
                const Joint &joint = robot_.model().getJoints()[j];
                if(!joint.isVelocityLimitEnabled()) { continue; }

                const float vel = dq[j] / dt;
                const float maxVel = joint.getVelocityLimit();
                dq_lim[j] = std::clamp(vel, -maxVel, maxVel) * dt;
            }

            return dq_lim;
        }

        // --- Line Search ---
        /**
         *  @brief Perform a backtracking line search to find an appropriate step size along the proposed joint update direction that satisfies the Armijo condition for sufficient decrease in the error norm, which can help to improve convergence and stability of the solver by ensuring that each iteration makes meaningful progress towards reducing the error
         *  @param q Current joint configuration vector
         *  @param dq Proposed joint update vector computed from the pseudoinverse and error,
         *  @param target Current IK target being solved for, which is used to compute the error at the proposed new configuration during the line search
         *  @param errNorm Current error norm for the target, which is used as the reference for checking the Armijo condition during the line search
         *  @return Step size multiplier along the proposed joint update direction that satisfies the Armijo condition, which can be used to scale the joint updates for the current iteration to ensure sufficient decrease in the error norm
         */
        template<cobalt::math::index_t M>
        inline float lineSearch(
            const cobalt::math::linear_algebra::Vector<nJ> &q,
            const cobalt::math::linear_algebra::Vector<nJ> &dq,
            const IKTarget &target,
            float errNorm) {
                const cobalt::math::linear_algebra::Vector<nJ> q_saved = robot_.state().q;
                float alpha = config_.step;

                for(iter_t i = 0; i < IK_BACKTRACK_NUM; i++) {
                    robot_.setJoints(q + alpha*dq);
                    fk_.solve(robot_.state());

                    const cobalt::math::linear_algebra::Vector<6> err = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
                    const cobalt::math::linear_algebra::Vector<M> taskErr = extractError<M>(cobalt::math::linear_algebra::Matrix<6,6>::diagonal(target.weight)*err);

                    const float testNorm = cobalt::math::linear_algebra::norm(taskErr);

                    if(testNorm < errNorm - IK_ARMIJO_COND_CONST*cobalt::math::linear_algebra::norm(dq)*alpha) {
                        robot_.setJoints(q_saved);
                        fk_.solve(robot_.state());
                        return alpha;
                    }

                    alpha *= IK_BACKTRACK_FACTOR;
                }

                robot_.setJoints(q_saved);
                fk_.solve(robot_.state());
                return alpha;
            }

        // --- Singularity Escape ---
        /**
         *  @brief Apply a perturbation to the joint configuration to escape from singularities or local minima when the solver detects that it is stuck, which can help to improve the robustness of the solver by allowing it to explore different configurations and potentially find a path towards a solution when it is stuck in a singularity or local minimum
         *  @param q Current joint configuration vector
         *  @param attempt Current attempt count for escaping singularity, which can be used to vary the perturbation applied to the joint configuration across different attempts to increase the chances of escaping from the singularity or local minimum
         *  @return New joint configuration vector after applying the perturbation to escape from the singularity or local minimum, which can be used as the starting point for the next iteration of the solver
         *  @note The function applies a sinusoidal perturbation to each joint based on its index and the attempt count, scaled by a factor of the joint's range and a configurable escape factor. This creates a varying perturbation across different joints and attempts, which can help to increase the chances of escaping from a singularity or local minimum. The new joint configuration is set on the robot, and the updated joint configuration is returned for use in the next iteration of the solver.
         */
        inline cobalt::math::linear_algebra::Vector<nJ> escapeSingularity(
            const cobalt::math::linear_algebra::Vector<nJ> &q,
            iter_t attempt) {
                cobalt::math::linear_algebra::Vector<nJ> q_esc = q;

                for(id_t j = 0; j < nJ; j++) {
                    const float range = robot_.model().getJoints()[j].getMaxLimit() - robot_.model().getJoints()[j].getMinLimit();
                    const float phase = static_cast<float>(j)*2.0f + static_cast<float>(attempt);
                    const float perturb = std::sin(phase*1.618033988f) * std::cos(phase*2.718281828f);

                    q_esc[j] += perturb * IK_ESCAPE_FACTOR * range;
                }

                robot_.setJoints(q_esc);
                return robot_.state().q;

            }

        // --- Utility ---
        /**
         *  @brief Build an invalid IK solution with the given status to return when the solver fails to find a valid solution, which can be used to provide feedback on the reason for failure (e.g. unreachable target, singularity, max iterations reached) while still returning a consistent solution structure
         *  @param status IKStatus enum value indicating the reason for failure, which is set in the returned IKSolution to provide feedback on the failure
         *  @return IKSolution object with the given status, zero iterations, current joint configuration, and zero error, which can be returned by the solver when it fails to find a valid solution to indicate the reason for failure while still providing a consistent solution structure
         *  @note The function initializes an IKSolution object with the provided status, sets iterations to zero, uses the current joint configuration from the robot's state, and sets the error to a zero vector. This provides a consistent structure for the solution even in failure cases, allowing the caller to check the status and understand the reason for failure while still having access to the current joint configuration and a defined error vector.
         */
        inline IKSolution<nJ> buildInvalidSolution(IKStatus status) const {
            IKSolution<nJ> sol;
            sol.status = status;
            sol.iterations = 0;
            sol.q = robot_.state().q;
            sol.err = cobalt::math::linear_algebra::Vector<6>::zero();

            return sol;
        }

        // --- Solver ---
        /**
         *  @brief Solve the inverse kinematics problem for a given target using an iterative method with adaptive damping, line search, and singularity handling to find a joint configuration that achieves the desired end-effector pose within a specified error threshold
         *  @param target IKTarget object containing the desired end-effector pose, target frame ID, weight for the error components, and IK mode (position-only or full pose) to solve for
         *  @return IKSolution object containing the resulting joint configuration, error vector, number of iterations taken, and status indicating success or reason for failure (e.g. unreachable target, singularity, max iterations reached)
         *  @note The function initializes the solver state, including the initial joint configuration, error norms, and counters for progress and singularity handling. It then enters an iterative loop where it computes the forward kinematics, calculates the error between the current end-effector pose and the target pose, checks for convergence, computes the Jacobian and its pseudoinverse using either DLS or SVD method based on the configuration, applies velocity limits and projects away from joint limits, and updates the joint configuration. The function also includes checks for lack of progress to determine if the
         *  target is unreachable, and it applies a perturbation to escape from singularities or local minima when detected. If the solver converges to a solution within the error threshold, it returns a success status; if it reaches the maximum number of iterations without convergence, it returns a status indicating whether the target is unreachable or if the max iterations were reached based on the minimum error observed.
         */
        template<cobalt::math::index_t M>
        inline IKSolution<nJ> solveTask(const IKTarget &target) {
            const cobalt::math::linear_algebra::Vector<nJ> q_init = robot_.state().q;

            IKSolution<nJ> sol = buildInvalidSolution(IKStatus::MaxIterations);
            sol.q = robot_.state().q;

            
            lambda_ = config_.dampingDLS;
            lastErrNorm_ = std::numeric_limits<float>::max();

            float prevErr = std::numeric_limits<float>::max();
            float minErr = std::numeric_limits<float>::max();
            iter_t noProgCount = 0;
            iter_t smallProgCount = 0;
            iter_t singularCount = 0;

            const cobalt::math::linear_algebra::Matrix<6,6> W = cobalt::math::linear_algebra::Matrix<6,6>::diagonal(target.weight);

            iter_t iter;
            for(iter = 0; iter < config_.maxIterations; iter++) {
                fk_.solve(robot_.state());

                const cobalt::math::linear_algebra::Vector<6> err = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
                const cobalt::math::linear_algebra::Vector<M> taskErr = extractError<M>(W*err);

                sol.err = err;
                const float errNorm  = cobalt::math::linear_algebra::norm(taskErr);

                // Convergence check
                if(errNorm < config_.threshold) {
                    sol.status = IKStatus::Success;
                    sol.iterations = iter;
                    sol.q = robot_.state().q;
                    break;
                }

                // Prog. Check
                if(errNorm < minErr) { minErr = errNorm; }
                if((prevErr - errNorm) < IK_MIN_ERROR_CHANGE) {
                    noProgCount++;
                    if(noProgCount >= IK_NOPROG_THRESHOLD) {
                        if(errNorm > config_.threshold * config_.unreachableMult) {
                            sol.status = IKStatus::Unreachable;
                            sol.iterations = iter;
                            sol.q = robot_.state().q;
                            break;
                        }
                    }
                } 
                else { noProgCount = 0; }
                prevErr = errNorm;


                const cobalt::math::linear_algebra::Matrix<6, nJ> J = jacobian_.compute(robot_.state(), target.frameId);
                const cobalt::math::linear_algebra::Matrix<M, nJ> J_task = extractJacobian<M>(W * J);

                bool isSingular = false;
                bool isNearSingularity = false;
                const cobalt::math::linear_algebra::Matrix<nJ, M> pinvJ = computePseudoInv<M>(J_task, taskErr, errNorm, noProgCount, isSingular, isNearSingularity);

                if((isSingular || isNearSingularity) && (noProgCount > IK_NOPROG_SOFT_THRESHOLD)) {
                    if(singularCount >= IK_ESCAPE_TRIES) {
                        sol.status = IKStatus::Singular;
                        sol.iterations = iter;
                        sol.q = robot_.state().q;
                        break;
                    } 
                    escapeSingularity(robot_.state().q, singularCount);
                    singularCount++;
                    continue;
                }

                cobalt::math::linear_algebra::Vector<nJ> dq = pinvJ * taskErr;
                const float stepSize = lineSearch<M>(robot_.state().q, dq, target, errNorm);
                dq = applyVelocityLimits(dq * stepSize, config_.timestep);
                projectAwayFromLimits(robot_.state().q, dq, 1.0f);


                if((cobalt::math::linear_algebra::norm(dq) < IK_MIN_STEP_SIZE) && (errNorm > config_.threshold * config_.unreachableMult)) {
                    smallProgCount++;
                    if(smallProgCount >= IK_SMALLPROG_THRESHOLD) {
                        sol.status = IKStatus::Unreachable;
                        sol.iterations = iter;
                        sol.q = robot_.state().q;
                        break;
                    }
                }
                else { smallProgCount = 0; }

                robot_.setJoints(robot_.state().q + dq);
                setCachedPseudoInv<M>(pinvJ);
                robot_.state().dq = dq;
                robot_.state().J = J;
            }

            if(iter >= config_.maxIterations) { // Max iterations reached
                sol.status = ((minErr > config_.threshold * config_.unreachableMult) ?IKStatus::Unreachable :IKStatus::MaxIterations);
                sol.iterations = iter;
                sol.q = robot_.state().q;

                fk_.solve(robot_.state());
                sol.err = computeError(target.pose, robot_.state().frameTransforms[target.frameId]);
            }


            robot_.setJoints(q_init);
            fk_.solve(robot_.state());

            return sol;
        }

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct an InverseKinematics solver for the given robot and configuration, initializing internal state and caches for the pseudoinverse computations
         *  @param robot Reference to the robot for which the IK solver will compute solutions, which is used for accessing the robot's state, model, and kinematic computations
         *  @param config Configuration parameters for the IK solver, which include settings for the solver type (DLS or SVD), damping factors, convergence thresholds, maximum iterations, and other parameters that affect the behavior of the solver. The configuration is stored internally and can be accessed or modified through the config() member function.
         *  @note The constructor initializes the internal state of the IK solver, including setting the reference to the robot, storing the configuration, initializing the Jacobian and forward kinematics solvers with the robot reference, setting the initial damping factor for DLS method, initializing the last error norm to a large value, and initializing the caches for the pseudoinverse computations. This setup allows the solver to be ready for solving IK problems for the given robot using the specified configuration.
         */
        explicit InverseKinematics(Robot<nL, nJ, nE> &robot, IKConfig config = IKConfig{})
            : robot_(robot), config_(config), jacobian_(robot), fk_(robot), lambda_(IK_DAMPING_INIT), lastErrNorm_(std::numeric_limits<float>::max()), pinv6Cached_(), pinv3Cached_() {}

        /**
         *  @brief Access IK configurations
         *  @return Reference to internal IK configuration
         */
        constexpr IKConfig &config() { return config_; }
        /**
         *  @brief Const access to IK configurations
         *  @return Const reference to internal IK configuration
         */
        constexpr const IKConfig &config() const { return config_; }

        /**
         *  @brief Solve the inverse kinematics problem for a given target, which can specify either position-only or full pose targets, and return the resulting joint configuration, error, and status of the solution
         *  @param target IKTarget object containing the desired end-effector pose, target frame ID, weight for the error components, and IK mode (position-only or full pose) to solve for
         *  @return IKSolution object containing the resulting joint configuration, error vector, number of iterations taken, and status indicating success or reason for failure (e.g. unreachable target, singularity, max iterations reached)
         *  @note The function checks the target's frame ID for validity and then dispatches to the appropriate solveTask function based on the target's IK mode (position-only or full pose). If the target's frame ID is invalid or if the IK mode is not recognized, it returns an invalid solution with the corresponding status. Otherwise, it calls solveTask with the appropriate template parameter for the task dimension (3 for position-only, 6 for full pose) to perform the iterative IK solving process and return the resulting solution.
         */
        IKSolution<nJ> solve(const IKTarget &target) {
            if(target.frameId >= nE) {
                return buildInvalidSolution(IKStatus::InvalidInput);
            }

            switch(target.mode) {
                case (IKMode::Position):    { return solveTask<3>(target); }
                case (IKMode::Pose):    { return solveTask<6>(target); }
                default:                    { return buildInvalidSolution(IKStatus::InvalidInput); }
            }
        }
};

} // cobalt::kinematics::solvers