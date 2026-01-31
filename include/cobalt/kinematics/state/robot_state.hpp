#pragma once

#include <array>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

// --------------------------------------
//             Robot State    
// --------------------------------------
/**
 *  @brief Robot state holding the current robot configuration
 */
template<id_t nL, id_t nJ, id_t nE>
struct RobotState {
    public:
        cobalt::math::linear_algebra::Vector<nJ> q{};
        cobalt::math::linear_algebra::Vector<nJ> dq{};

        cobalt::math::linear_algebra::Matrix<6, nJ> J{};

        std::array<cobalt::math::geometry::Transform<>, nL> linkTransforms{};
        std::array<cobalt::math::geometry::Transform<>, nE> frameTransforms{};

        bool validJ{false};
        bool validLinks{false};
        bool validFrames{false};
}; 

} // cobalt::kinematics