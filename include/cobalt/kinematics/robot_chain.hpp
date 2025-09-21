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
 *  @tparam J Number of joints in the chain
 *  @tparam L Number of links in the chain
 */
template<uint8_t J, uint8_t L>
struct RobotChain {
    private:
        std::array<Joint, J> joints_;
        std::array<Link, L> links_;

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
        RobotChain() : joints_{}, links_{} {}


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
            return links_[L-1].worldFrame(); 
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
         *  @brief Set the frame/pose of the end-effector
         *  @return `true` if joint value was set successfully, `false` otherwise
         */
        bool setJoint(uint8_t index, float value) {
            if(index < J) {
                return (joints_[index].setValue(value));
            }
            else { return false;}
        }

        /**
         *  @brief Update the world frames of all links
         */
        void forwardKinematics() {
            for(Joint &j : joints_) {
                if(j.parentIndex() < 0) { continue; }
                if(j.childIndex() < 0)  { continue; }

                cobalt::math::geometry::Transform<> motion = getJointMotion(j);
                
                links_[j.childIndex()].worldFrame() = links_[j.parentIndex()].worldFrame() * (motion * links_[j.childIndex()].frame());
            }
        }
        
};

} // cobatl::kinematics