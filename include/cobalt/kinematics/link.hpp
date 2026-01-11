#pragma once

#include <string>

#include "config.hpp"

#include "cobalt/math/config.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr id_t LINK_DEFAULT_ID = invalidID_;

constexpr float LINK_DEFAULT_MASS = 0.0f;
constexpr cobalt::math::linear_algebra::Matrix<3,3> LINK_DEFAULT_INERTIA = cobalt::math::linear_algebra::Matrix<3,3>::eye();

const cobalt::math::geometry::Transform<> LINK_DEFAULT_COM = cobalt::math::geometry::Transform<>::eye();

// --------------------------------------
//              Robot Link    
// --------------------------------------
/**
 *  @brief Link of a robot in a robot chain
 */
struct Link {
    private:
        id_t id_{LINK_DEFAULT_ID};

        std::string name_{""};

        float mass_{LINK_DEFAULT_MASS};
        cobalt::math::linear_algebra::Matrix<3,3> inertia_{LINK_DEFAULT_INERTIA};
        cobalt::math::geometry::Transform<> origin_{LINK_DEFAULT_COM};

        // ---------------- Helper Function ----------------
        constexpr void enforceConstraints() {
            assert(mass_ >= 0.0f && "[LINK Error] : Link mass cannot be negative.");

            for(cobalt::math::index_t i = 0; i < 3; i++) {  //TODO: Replace with isPSD check
                for(cobalt::math::index_t j = 0; j < 3; j++) {
                    assert(inertia_(i,j) >= 0.0f && "[LINK Error] : Link inertia matrix cannot have negative elements.");
                }
            }
        }

    public:
        // ---------------- Constructors ----------------
        explicit Link(id_t id = LINK_DEFAULT_ID,
                      const std::string &name = "",
                      float mass = LINK_DEFAULT_MASS,
                      const cobalt::math::linear_algebra::Matrix<3,3> &inertia = LINK_DEFAULT_INERTIA,
                      const cobalt::math::geometry::Transform<> &origin = LINK_DEFAULT_COM)
            : id_(id), name_(name), mass_(mass), inertia_(inertia), origin_(origin) {
                enforceConstraints();
            }

        // ---------------- Getters ----------------
        constexpr id_t getId() const { return id_; }
        constexpr float getMass() const { return mass_; }

        const cobalt::math::linear_algebra::Matrix<3,3> &getInertia() const { return inertia_; }
        const cobalt::math::geometry::Transform<> &getOrigin() const { return origin_; }  
        const std::string &getName() const { return name_; }
};

} //cobalt::kinematics