#pragma once

#include <string>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr uint8_t LINK_DEFAULT_LENGTH = -1;

constexpr uint8_t LINK_DEFAULT_ID = 0;
constexpr uint8_t LINK_DEFAULT_CHILD = -1;

constexpr float LINK_DEFAULT_MASS = 0.0f;
constexpr cobalt::math::geometry::Transform<> LINK_DEFAULT_TRANSFORM = cobalt::math::geometry::Transform<>::eye();
constexpr cobalt::math::geometry::Transform<> LINK_DEFAULT_COM = cobalt::math::geometry::Transform<>::eye();

// --------------------------------------
//              Robot Link    
// --------------------------------------
/**
 *  @brief Link of a robot in a robot chain
 */
struct Link {
    private:
        std::string name_;
        uint8_t id_;
        int8_t child_;

        float length_;

        cobalt::math::geometry::Transform<> localTransform_;  // From parent's frame
        cobalt::math::geometry::Transform<> worldTransform_;  // Absolute world frame

        float mass_;
        cobalt::math::geometry::Transform<> centerMass_;

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a link in a robot
         *  @param name Unique name associated with the link
         *  @param length Length of the link
         *  @param linkId Unique id associated with the link
         *  @param childJoint Id of the child joint link is attached to
         *  @param mass Local position + orientation of the joint relative to its parent
         */
        Link(const std::string &name = "", float length = LINK_DEFAULT_LENGTH, uint8_t linkId = LINK_DEFAULT_ID, uint8_t childJoint = LINK_DEFAULT_CHILD, float mass = LINK_DEFAULT_MASS)
            :  name_(name), length_(length), id_(linkId), child_(childJoint), mass_(mass), localTransform_(LINK_DEFAULT_TRANSFORM), worldTransform_(LINK_DEFAULT_TRANSFORM), centerMass_(LINK_DEFAULT_COM) {} 
        

        // ---------------- Getters ----------------
        /**
         *  @brief Get the name of the link
         */
        std::string getName() { return name_; }

        /**
         *  @brief Get the link's child joint's id 
         */
        int8_t getChild() { return child_; }

        /**
         *  @brief Get the link's length
         */
        float getLength() { return length_; }
        
        // ---------------- Setters ----------------
        /**
         *  @brief Set the name of the link
         */
        void setName(std::string name) { name_ = name; }
        
        /**
         *  @brief Set the unique Id of the link
         */
        constexpr void setId(uint8_t id) { id_ = id; }

        /**
         *  @brief Set the link's child joint's id 
         */
        constexpr void setChild(uint8_t childJoint) { child_ = childJoint; }


        // ---------------- Accessors ----------------
        template<typename T = float>
            constexpr cobalt::math::geometry::Transform<T> &frame() {
                return localTransform_;
            }

        template<typename T = float>
            const cobalt::math::geometry::Transform<T> &frame() const {
                return localTransform_;
            }

         template<typename T = float>
            constexpr cobalt::math::geometry::Transform<T> &worldFrame() {
                return worldTransform_;
            }

        template<typename T = float>
            const cobalt::math::geometry::Transform<T> &worldFrame() const {
                return worldTransform_;
            }
};

} //cobalt::kinematics