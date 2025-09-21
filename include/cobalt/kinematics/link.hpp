#pragma once

#include <string>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr float LINK_DEFAULT_MASS = 0.0f;
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

        cobalt::math::geometry::Transform<> localTransform_;  // From parent's frame
        cobalt::math::geometry::Transform<> worldTransform_;  // Absolute world frame

        float mass_;
        cobalt::math::geometry::Transform<> centerMass_;

    public:
        // ---------------- Constructors ----------------
        Link(const cobalt::math::geometry::Transform<> &localT = cobalt::math::geometry::Transform<>::eye(), float mass = LINK_DEFAULT_MASS, const std::string &name = "")
            : localTransform_(localT), worldTransform_(localT), mass_(mass), centerMass_(LINK_DEFAULT_COM), name_(name)  {} 
        

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