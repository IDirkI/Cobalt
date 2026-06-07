#pragma once

#include <array>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics::logging {

// --------------------------------------
//            Robot Snapshot    
// --------------------------------------
/**
 * @brief Snapshot of a robot's state at a given time containing joint configuration and link/frame transforms
 */
template<id_t nL, id_t nJ, id_t nE>
struct RobotSnapshot {
    public:
        uint64_t timestamp_us;

        cobalt::math::linear_algebra::Vector<nJ> q{};
        std::array<cobalt::math::geometry::Transform<>, nL> linkTransforms{};
        std::array<cobalt::math::geometry::Transform<>, nJ> jointTransforms{};
        std::array<cobalt::math::geometry::Transform<>, nE> frameTransforms{};

        /**
         * @brief Capture a snapshot of a robot's state at a given time
         * @param robot Robot to capture a snapshot of
         * @param timestamp_us Timestamp to associate with the snapshot in microseconds
         * @return Snapshot of the robot's state at the given time
         */
        static RobotSnapshot capture(const Robot<nL, nJ, nE> &robot, uint64_t timestamp_us = 0) {
            RobotSnapshot s;
            s.timestamp_us = timestamp_us;
            s.q = robot.state().q;
            for(id_t i = 0; i < nL; i++) { s.linkTransforms[i]   = robot.state().linkTransforms[i];  }
            for(id_t i = 0; i < nJ; i++) { s.jointTransforms[i]  = robot.state().jointTransforms[i]; }
            for(id_t i = 0; i < nE; i++) { s.frameTransforms[i]  = robot.state().frameTransforms[i]; }
            return s;
        }
};


}; // cobalt::kinematics::logging