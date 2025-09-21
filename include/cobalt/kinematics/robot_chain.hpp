#pragma once

#include <array>
#include <stdint.h>

#include "joint.hpp"
#include "link.hpp"

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
        void updateWorld(uint8_t index, float value) {
            links_[0].worldFrame() = links_[0].localFrame();    // Set base frame

            for(Joint &j : joints_) {
                if(j.getParentIndex() < 0) { continue; }
                if(j.getChildIndex() < 0)  { continue; }

                
            }
        }
        
};

} // cobatl::kinematics