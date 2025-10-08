#pragma once

#include <array>
#include <stdint.h>

#include "joint.hpp"
#include "link.hpp"

#include "../math/geometry/transform/transform_ops.hpp"

namespace cobalt::kinematics {

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

        // ---------------- Helper ----------------
        cobalt::math::geometry::Transform<> getJointMotion(const Joint &j) const {
            switch(j.getType()) {
                case (JointType::Revolute):     { return cobalt::math::geometry::Transform<>::fromAxisAngle(j.getAxis()*j.getValue()); }
                case (JointType::Prismatic):    { return cobalt::math::geometry::Transform<>::fromTranslation(j.getAxis()*j.getValue()); }
                default:                        { return cobalt::math::geometry::Transform<>::eye(); }
            }
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
            updateLinks();
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
         *  @brief Get the frame/pose of the end-effector
         */
        const cobalt::math::geometry::Transform<> endEffector() const {
            return end_->worldFrame(); 
        }

        // ---------------- Accessors Functions ----------------

        /**
         *  @brief Get the reference to frame/pose of an intermediary joint
         */
        constexpr Joint &joint(uint8_t index) {
            if(index <= J) {
                return joints_[index]; 
            }

            return joints_[J-1];
        }

        /**
         *  @brief Get a const intermediary link value
         */
        const Joint joint(uint8_t index) const {
            if(index <= J) {
                return joints_[index]; 
            }

            return joints_[J-1];
        }

        /**
         *  @brief Get the reference to frame/pose of an intermediary link
         */
        constexpr Link &link(uint8_t index) {
            if(index <= L) {
                return links_[index]; 
            }

            return links_[L-1];
        }

        /**
         *  @brief Get a const intermediary link value
         */
        const Link link(uint8_t index) const {
            if(index <= L) {
                return links_[index]; 
            }

            return links_[L-1];
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Serach through the link array and set the base and endeffector pointers
         *  @return `true` if a 'base'-link & endeffector was found, `false` otherwise
         */
        constexpr bool updateLinks() {
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
         *  @brief Set a joint value of the robot
         *  @return `true` if joint value was set successfully, `false` otherwise
         */
        constexpr bool setJoint(uint8_t index, float value) {
            if(index < J) {
                return (joints_[index].setValue(value));
            }
            else { return false;}
        }

        /**
         *  @brief Set the joint values of the robot
         *  @return `true` if joint values were set successfully, `false` otherwise
         */
        constexpr bool setJoints(std::array<float, J> values) {
            for(uint8_t i = 0; i < J; i++) {
                if(!setJoint(i, values[i])) { return false; }
            }
            return true;
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

                childLink->worldFrame() = parentLink->worldFrame() * (motion * offset * childLink->frame());
                
                if(childLink->getChild() == -1) { break; }  // End-effector reached

                j = &joints_[childLink->getChild()];
                parentLink = childLink;
                childLink = &links_[j->getChild()];
            }

            return endEffector();
        }
};

} // cobatl::kinematics