#pragma once

#include <array>
#include <stdint.h>

#include "joint.hpp"
#include "link.hpp"

#include "../math/linear_algebra/vector/vector.hpp"
#include "../math/linear_algebra/vector/vector_util.hpp"
#include "../math/linear_algebra/matrix/matrix.hpp"
#include "../math/linear_algebra/matrix/matrix_ops.hpp"
#include "../math/geometry/transform/transform.hpp"
#include "../math/geometry/transform/transform_ops.hpp"

namespace cobalt::kinematics {

constexpr float ROBOTCHAIN__IK_THRESHOLD = 1e-4;
constexpr uint8_t ROBOTCHAIN_IK_LINESEARCH_COUNT = 6;
constexpr uint8_t ROBOTCHAIN_IK_RESTARTS = 6;

constexpr float ROBOTCHAIN_DEFAULT_NOISE_MAG = 0.1;
constexpr uint8_t ROBOTCHAIN_DEFAULT_IK_MAX_ITERATION = 20;
constexpr float ROBOTCHAIN_DEFAULT_IK_ALPHA = 0.8f;

// --------------------------------------
//              Robot Chain    
// --------------------------------------
/**
 *  @brief Chain of links and joints modeling the robot
 *  @tparam L Number of links in the chain
 *  @tparam J Number of joints in the chain
 */
template<uint8_t L, uint8_t J>
struct RobotChain {
    private:
        std::array<Link, L> links_;
        std::array<Joint, J> joints_;

        Link *base_;
        Link *end_;

        uint32_t seed = 1638178391u;

        // ---------------- Helper ----------------
        cobalt::math::geometry::Transform<> getJointMotion(const Joint &j) const {
            switch(j.getType()) {
                case (JointType::Revolute):     { return cobalt::math::geometry::Transform<>::fromAxisAngle(j.getAxis()*j.getValue()); }
                case (JointType::Prismatic):    { return cobalt::math::geometry::Transform<>::fromTranslation(j.getAxis()*j.getValue()); }
                default:                        { return cobalt::math::geometry::Transform<>::eye(); }
            }
        }

        float getNoise(float min = ROBOTCHAIN_DEFAULT_NOISE_MAG, float max = ROBOTCHAIN_DEFAULT_NOISE_MAG) {
            seed = 67395 * seed + 6741013;
            float rand = (seed & 0x00ffffff) / float(0x01000000);
            return min + (max-min)*rand;
        }

    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Default RobotChain constrcutor
         */
        RobotChain() : links_{}, joints_{}, base_(nullptr), end_(nullptr) {}

         /**
         *  @brief RobotChain constructor with link and joint array intializers
         */
        RobotChain(std::array<Link, L> links, std::array<Joint, J> joints) : links_(links), joints_(joints), base_(nullptr), end_(nullptr) {
            findLinks();
            forwardKinematics();
        }


        // ---------------- Getters ----------------
        /**
         *  @brief Get the number of joins in the chain
         */
        constexpr uint8_t getJointNum() const { return J; }

        /**
         *  @brief Get the number of links in the chain
         */
        constexpr uint8_t getLinkNum() const { return L; }

        /**
         *  @brief Get the const reference to frame/pose of the end-effector
         *  @return World frame of the end-effector in the chain
         */
        const cobalt::math::geometry::Transform<> &endEffector() const {
            return end_->worldFrame(); 
        }

        // ---------------- Accessors Functions ----------------

        /**
         *  @brief Get the reference to a joint
         *  @param id is the unique Id of the joint
         *  @return Const refrence to the joint with the id `id`. If `id` is invalid, returns the last joint in the list.
         */
        constexpr Joint &joint(uint8_t id) {
            if(id <= J) {
                return joints_[id]; 
            }

            return joints_[J-1];
        }

        /**
         *  @brief Get the const reference to a joint
         *  @param id is the unique Id of the joint
         *  @return Const refrence to the joint with the id `id`. If `id` is invalid, returns the last joint in the list.
         */
        const Joint &joint(uint8_t id) const {
            if(id <= J) {
                return joints_[id]; 
            }

            return joints_[J-1];
        }

        /**
         *  @brief Get a refrence to an intermediary link value
         *  @param id is the unique Id of the link
         *  @return Const refrence to the link with the id `id`. If `id` is invalid, returns the enf-effector link
         */
        constexpr Link &link(uint8_t id) {
            if(id <= L) {
                return links_[id]; 
            }

            return links_[L-1];
        }

        /**
         *  @brief Get a const refrence to an intermediary link value
         *  @param id is the unique Id of the link
         *  @return Const refrence to the link with the id `id`. If `id` is invalid, returns the enf-effector link
         */
        const Link &link(uint8_t id) const {
            if(id <= L) {
                return links_[id]; 
            }

            return links_[L-1];
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Serach through the link array and set the base and endeffector pointers
         *  @return `true` if a 'base'-link & endeffector was found, `false` otherwise
         */
        constexpr bool findLinks() {
            bool foundBase = false;
            bool foundEndEffector = false;

            for(Link &l: links_) {
                if(l.getName() == "base") {
                    base_ = &l;
                    foundBase = true;
                }
                
                if(l.getChild() == -1) {
                    end_ = &l;
                    foundEndEffector = true;
                }
            }

            return foundBase && foundEndEffector;
        }

        /**
         *  @brief Set a joint value of the robot and update its body
         *  @return `true` if joint value was set successfully, `false` otherwise or if `index` was out of bounds
         */
        constexpr bool setJoint(uint8_t index, float value) {
            if(index < J) {
                if(joints_[index].setValue(value)) {
                    forwardKinematics();
                    return true;
                }
                else { return false; }
            }
            else { return false; }
        }

        /**
         *  @brief Set the joint values of the robot and update its body
         *  @return `true` if joint values were set successfully, `false` otherwise
         */
        constexpr bool setJoints(std::array<float, J> values) {
            bool result = true;
            for(Joint &j : joints_) {
                if(!j.setValue(values[j.getId()])) { result = false; }
            }
            forwardKinematics();
            return result;
        }


        /**
         *  @brief Calculate the jacobian matrix associated with the robot at the moment 
         *  @return Instantanious jacobian matrix(6xJ) of the robot
         */
        cobalt::math::linear_algebra::Matrix<6, J> getJacobian() {
            cobalt::math::linear_algebra::Matrix<6, J> Jac = cobalt::math::linear_algebra::Matrix<6, J>::zero();

            for( Joint &j : joints_) {
                switch(j.getType()) {
                    case (JointType::Revolute): {
                        cobalt::math::linear_algebra::Vector<3> axis = j.getAxis();
                        cobalt::math::geometry::Transform<> motion = getJointMotion(j);
                        cobalt::math::linear_algebra::Vector<3> diff = (end_->worldFrame()).translation();

                        // Jacobian columns:
                        //   Jac_j  = | Jv_j | = | b_j x (x_e - l_{j-1}) |
                        //            | Jw_j |   |          b_j          |     
                        cobalt::math::linear_algebra::Vector<3> Jv = cross(axis, diff - links_[j.getParent()].worldFrame().translation());

                        Jac(0, j.getId()) = Jv[0];
                        Jac(1, j.getId()) = Jv[1];
                        Jac(2, j.getId()) = Jv[2];

                        Jac(3, j.getId()) = axis[0];
                        Jac(4, j.getId()) = axis[1];
                        Jac(5, j.getId()) = axis[2];

                        break;
                    }
                    case (JointType::Prismatic): {
                        cobalt::math::linear_algebra::Vector<3> axis = j.getAxis();

                        // Jacobian columns:
                        //   Jac_j  = | Jv_j | = | b_j | = | axis |
                        //            |   0  |   |  0  |   |   0  |     
                        Jac(0, j.getId()) = axis[0];
                        Jac(1, j.getId()) = axis[1];
                        Jac(2, j.getId()) = axis[2];
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }

            return Jac;
        }

        /**
         *  @brief Update the world frames of all links
         *  @note `updateLinks()` must have been called before hand to set `base_` and `end_` pointers
         *  @return The world frame of the end-effector of the robot after the motion is complete. If `base_` or `end_` wasn't set beforehand, return the identity transform
         */
        cobalt::math::geometry::Transform<> forwardKinematics() {
            if(!base_) { return cobalt::math::geometry::Transform<>::eye(); }
            if(!end_) { return cobalt::math::geometry::Transform<>::eye(); }

            Joint *j = &joints_[base_->getChild()];
            Link *parentLink = base_;
            Link *childLink = &links_[j->getChild()];

            while(true) {
                cobalt::math::geometry::Transform<> motion = getJointMotion(*j);
                cobalt::math::geometry::Transform<> offset = cobalt::math::geometry::Transform<>::fromTranslation({childLink->getLength(), 0.0f, 0.0f});

                childLink->frame() = motion * offset;
                childLink->worldFrame() = parentLink->worldFrame() * childLink->frame();
                
                if(childLink->getChild() == -1) { break; }  // End-effector reached

                j = &joints_[childLink->getChild()];
                parentLink = childLink;
                childLink = &links_[j->getChild()];
            }

            return endEffector();
        }

        /**
         *  @brief Calculate the reqired joint values 
         *  @note `updateLinks()` must have been called before hand to set `base_` and `end_` pointers
         *  @param q Output array of the required joint values to achieve the goal pose
         *  @param goal Desired 6-vector end-effector pose
         *  @param maxIter The number of maximum iterations the IK algorithm will run for. Defaults to 20
         *  @param alpha Initial step size for line search that determines convergence speed and accuracy
         *  @return The total amount of iterations the IK algorithm ran for. -1 if the state is unreachable.
         */
         size_t inverseKinematics(std::array<float, J> &q, cobalt::math::linear_algebra::Vector<3> goal, uint8_t maxIter = ROBOTCHAIN_DEFAULT_IK_MAX_ITERATION, float alpha = ROBOTCHAIN_DEFAULT_IK_ALPHA) {
            size_t iter = 0;
            bool converged = false;
            
            cobalt::math::linear_algebra::Vector<3> err{};
            
            std::array<float, J> q_hold{};
            for(Joint &j : joints_) { q_hold[j.getId()] = j.getValue(); }
            // q_0 = q_current + random noise
            cobalt::math::linear_algebra::Vector<J> dq{};
            cobalt::math::linear_algebra::Vector<J> q_vec = cobalt::math::linear_algebra::Vector<J>::fromArray(q_hold);

            for(uint8_t i = 0; i < J; i++) { 
                q_vec[i] += getNoise();
            }


            for(uint8_t restarts = 0; (restarts < ROBOTCHAIN_IK_RESTARTS) && !converged; restarts++) {
                if(restarts != 0) {
                    for(uint8_t i = 0; i < J; i++) { 
                        q_vec[i] = q_hold[i] + restarts*getNoise();
                    }
                }
                for(uint8_t i = 0; i < maxIter; i++) {
                    iter++;
                    printf("[ %3.4f    %3.4f ]\n", endEffector().translation()[0], endEffector().translation()[1]);
                    //printf("[ Angle: %3.4f    %3.4f ]\n", q_vec[0], q_vec[1]);

                    for(uint8_t i = 0; i < J; i++) { 
                        if(q_vec[i] > joints_[i].getMaxLimit()) { q_vec[i] = joints_[i].getMaxLimit(); }
                        if(q_vec[i] < joints_[i].getMinLimit()) { q_vec[i] = joints_[i].getMinLimit(); }
                    } 
                    setJoints(cobalt::math::linear_algebra::toArray(q_vec));

                    cobalt::math::linear_algebra::Matrix<6, J> Jac = getJacobian();
                    cobalt::math::linear_algebra::Matrix<J, 3> J_pseudo;

                    if(!cobalt::math::linear_algebra::pseudoL(Jac.template block<3, J>(), J_pseudo)) {
                        printf("PSEUDO FAIL\n");
                        setJoints(q_hold);
                        return iter;
                    }

                    err = goal - endEffector().translation();
                    dq = J_pseudo*err;

                    // ---- Line seach ----
                    float alphaTest = alpha;

                    bool improved = false;
                    for(uint8_t j = 0; j < ROBOTCHAIN_IK_LINESEARCH_COUNT; j++) {
                        cobalt::math::linear_algebra::Vector<J> q_test = q_vec + alphaTest*dq;
                        setJoints(cobalt::math::linear_algebra::toArray(q_test));

                        cobalt::math::linear_algebra::Vector<3> errTest{};
                        errTest = goal - endEffector().translation();

                        if(cobalt::math::linear_algebra::norm(errTest) < cobalt::math::linear_algebra::norm(err)) {
                            q_vec = q_test;
                            err = errTest;
                            improved = true;
                            break;
                        }

                        alphaTest *= 0.5;
                    }

                    if(!improved) { break; }
                    // --------------------

                    if( (fabsf(err[0]) < ROBOTCHAIN__IK_THRESHOLD  &&
                         fabsf(err[1]) < ROBOTCHAIN__IK_THRESHOLD) &&
                         fabsf(err[2]) < ROBOTCHAIN__IK_THRESHOLD) { converged = true; break; }
                }
            }

            q = cobalt::math::linear_algebra::toArray(q_vec);
            setJoints(q_hold);
            return iter;
        }
};

} // cobatl::kinematics