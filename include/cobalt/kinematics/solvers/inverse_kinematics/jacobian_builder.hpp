#pragma once

#include <array>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"

#include "cobalt/kinematics/model/robot_model.hpp"

namespace cobalt::kinematics::solvers {

// --------------------------------------
//          Jacobian Builder
// --------------------------------------
/**
 *  @brief Class for building the Jacobian matrix for a robot given its model and current state
 *  @tparam nL Number of links in the robot
 *  @tparam nJ Number of joints in the robot
 *  @tparam nE Number of end effectors (frames with targets) in the robot
 *  @note JacobianBuilder is designed to be used as a helper class within IK solvers and is not intended for direct use by external code. It relies on the robot model and state to compute the Jacobian matrix for a specific target frame.
 */
template<id_t nL, id_t nJ, id_t nE>
class JacobianBuilder {
    private:
        const RobotModel<nL, nJ, nE> &model_;
        std::array<KinematicPath<nJ>, nE> framePaths_;

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a jacobian factory with RobotModel caching
         *  @param robot Robot to create the factory for
         */
        explicit JacobianBuilder(const Robot<nL, nJ, nE> &robot) : model_(robot.model()), framePaths_(robot.model().getFramePaths()) {}

        // ---------------- Member Functions ----------------
        cobalt::math::linear_algebra::Matrix<6, nJ> compute(const RobotState<nL, nJ, nE> &state, id_t frameId) const {
            cobalt::math::linear_algebra::Matrix<6, nJ> J = cobalt::math::linear_algebra::Matrix<6, nJ>::zero();

            const cobalt::math::linear_algebra::Vector<3> o_frame = state.frameTransforms[frameId].translation();
            const KinematicPath<nJ> &path = framePaths_[frameId];

            for(id_t i = 0; i < path.length; i++) {
                const id_t j = path.joints[i];
                const Joint &joint = model_.getJoints()[j];

                const cobalt::math::geometry::Transform<> &T_joint = state.jointTransforms[j];
                const cobalt::math::linear_algebra::Vector<3> a = cobalt::math::geometry::rotate(T_joint.rotation(), joint.getAxis()); // world axis

                switch(joint.getType()) {
                    case (JointType::Revolute): {
                        const cobalt::math::linear_algebra::Vector<3> r = o_frame - T_joint.translation();
                        const cobalt::math::linear_algebra::Vector<3> v = cobalt::math::linear_algebra::cross(a, r);

                        J(0, j) = v[0];
                        J(1, j) = v[1];
                        J(2, j) = v[2];
                        J(3, j) = a[0];
                        J(4, j) = a[1];
                        J(5, j) = a[2];

                        break;
                    }
                    case (JointType::Prismatic): {
                        J(0, j) = a[0];
                        J(1, j) = a[1];
                        J(2, j) = a[2];

                        break;
                    }
                    case (JointType::Fixed): { break; }
                    default: { break; }
                }
            }

            return J;
        }
};

} // cobalt::kinematics::solvers