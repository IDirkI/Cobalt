#pragma once

#include <string>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr uint8_t LINK_DEFAULT_ID = 0;

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

        cobalt::math::geometry::Transform<> localTransform_;  // From parent's frame
        cobalt::math::geometry::Transform<> worldTransform_;  // Absolute world frame

        float mass_;
        cobalt::math::geometry::Transform<> centerMass_;

    public:
        // ---------------- Constructors ----------------
        Link(const std::string &name = "", uint8_t linkId = LINK_DEFAULT_ID, float mass = LINK_DEFAULT_MASS, const cobalt::math::geometry::Transform<> &localT = LINK_DEFAULT_TRANSFORM)
            :  name_(name), id_(linkId), localTransform_(localT), worldTransform_(localT), mass_(mass), centerMass_(LINK_DEFAULT_COM) {} 
        

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
        
        // ---------------- Setters ----------------
        void setName(std::string name) {
            name_ = name;
        }
        
        /**
         *  @brief Set the unique Id of the joint
         */
        constexpr void setId(uint8_t id) {
            id_ = id;
        }
};

} //cobalt::kinematics