#pragma once

#include <string>

#include "cobalt/kinematics/config.hpp"

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

        bool isVirtual_{false};

        // ---------------- Helper Function ----------------
        constexpr void validate() {
            assert(mass_ >= 0.0f && "[LINK Error] : Link mass cannot be negative.");

            for(cobalt::math::index_t i = 0; i < 3; i++) {  //TODO: Replace with isPSD check
                for(cobalt::math::index_t j = 0; j < 3; j++) {
                    assert(inertia_(i,j) >= 0.0f && "[LINK Error] : Link inertia matrix cannot have negative elements.");
                }
            }
        }

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a robot link
         *  @param id ID of the link
         *  @param name Name of the link
         *  @param mass Mass of the link
         *  @param inertia Inertia matrix of the link about the center of mass
         *  @param origin Transform of the link's center of mass relative to the link frame
         *  @param isVirtual Flag to set a link as virtual (part of a compound joint)
         *  @note Validates the link parameters upon construction
         *  @throws AssertionError if the link parameters are invalid
         */
        explicit Link(id_t id = LINK_DEFAULT_ID,
                      const std::string &name = "",
                      float mass = LINK_DEFAULT_MASS,
                      const cobalt::math::linear_algebra::Matrix<3,3> &inertia = LINK_DEFAULT_INERTIA,
                      const cobalt::math::geometry::Transform<> &origin = LINK_DEFAULT_COM,
                      bool isVirtual = false)
            : id_(id), name_(name), mass_(mass), inertia_(inertia), origin_(origin), isVirtual_(isVirtual) {
                validate();
            }

        // ---------------- Getters ----------------
        /**
         *  @brief Get the ID of the link
         *  @return ID of the link
         */
        constexpr id_t getId() const { return id_; }
        /**
         *  @brief Get the mass of the link
         *  @return Mass of the link
         */
        constexpr float getMass() const { return mass_; }
        /**
         *  @brief Get if the link is virtual
         *  @return Virtualness of link
         */
        constexpr bool getVirtual() const { return isVirtual_; }


        /**
         *  @brief Get the name of the link
         *  @return Name of the link
         */
        const std::string &getName() const { return name_; }
        /**
         *  @brief Get the inertia matrix of the link
         *  @return Inertia matrix of the link about the center of mass
         */
        const cobalt::math::linear_algebra::Matrix<3,3> &getInertia() const { return inertia_; }
        /**
         *  @brief Get the origin transform of the link
         *  @return Transform of the link's center of mass relative to the link frame
         */
        const cobalt::math::geometry::Transform<> &getOrigin() const { return origin_; }  
};

} //cobalt::kinematics