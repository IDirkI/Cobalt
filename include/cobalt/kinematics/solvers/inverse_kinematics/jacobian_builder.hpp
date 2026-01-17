#pragma once

#include "stdio.h"

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"
#include "cobalt/kinematics/state/robot_state.hpp"
#include "cobalt/kinematics/robot.hpp"

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"

namespace cobalt::kinematics::solvers {

template<id_t nL, id_t nJ, id_t nE>
class JacobianBuilder {
    public:
        const RobotModel<nL, nJ, nE> &model_;

        // ---------------- Helper Functions ----------------
        const cobalt::math::linear_algebra::Vector<3> getWorldJointAxis(const RobotState<nL, nJ, nE> &state, id_t jointId) const {
            const cobalt::math::geometry::Transform<> T_link = state.linkTransforms[model_.getJoints()[jointId].getParentId()];
            const cobalt::math::geometry::Transform<> T_joint = model_.getJoints()[jointId].getOrigin();

            cobalt::math::geometry::Transform<> T_joint_world = T_link * T_joint;
            return  rotate(T_joint_world.rotation(), model_.getJoints()[jointId].getAxis());
        }

        const cobalt::math::linear_algebra::Vector<3> getWorldJointPosition(const RobotState<nL, nJ, nE> &state, id_t jointId) const {
            const cobalt::math::geometry::Transform<> T_link = state.linkTransforms[model_.getJoints()[jointId].getParentId()];
            const cobalt::math::geometry::Transform<> T_joint = model_.getJoints()[jointId].getOrigin();

            cobalt::math::geometry::Transform<> T_joint_world = T_link * T_joint;
            return  T_joint_world.translation();
        }

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a jacobian factory with RobotModel caching
         *  @param robot Robot to create the factory for
         */
        JacobianBuilder(Robot<nL, nJ, nE> &robot) : model_(robot.model()) {}

        /**
         *  @brief Construct a jacobian factory with RobotModel caching
         *  @param robot RobotModel to create the factory for
         */
        JacobianBuilder(RobotModel<nL, nJ, nE> &model) : model_(model) {}

        // ---------------- Member Functions ----------------
        cobalt::math::linear_algebra::Matrix<6,nJ> compute(RobotState<nL, nJ, nE> &state, id_t frameId) {
            cobalt::math::linear_algebra::Matrix<6,nJ> J = cobalt::math::linear_algebra::Matrix<6,nJ>::zero();

            const cobalt::math::geometry::Transform<> &T_frame = state.frameTransforms[frameId];
            cobalt::math::linear_algebra::Vector<3> p_frame = T_frame.translation();

            const KinematicPath<nJ> &path = model_.getFramePaths()[frameId];

            for(id_t i = 0; i < path.length; i++) {
                id_t j = path.joints[i];
                const Joint &joint = model_.getJoints()[j];

                const cobalt::math::linear_algebra::Vector<3> a_joint = getWorldJointAxis(state, j);
                const cobalt::math::linear_algebra::Vector<3> p_joint = getWorldJointPosition(state, j);

                switch(joint.getType()) {
                    case(JointType::Prismatic): {
                        J(0,j) = a_joint.x();
                        J(1,j) = a_joint.y();
                        J(2,j) = a_joint.z();

                        J(3,j) = 0.0f;
                        J(4,j) = 0.0f;
                        J(5,j) = 0.0f;
                        break;
                    }
                    case(JointType::Revolute): {
                        cobalt::math::linear_algebra::Vector<3> r = p_frame - p_joint;
                        cobalt::math::linear_algebra::Vector<3> v = cross(a_joint, r);

                        J(0,j) = v.x();
                        J(1,j) = v.y();
                        J(2,j) = v.z();

                        J(3,j) = a_joint.x();
                        J(4,j) = a_joint.y();
                        J(5,j) = a_joint.z();
                        break;
                    }
                    case(JointType::Fixed): { break; }
                    default: { break; }
                }
            }
            
            return J;
        }


};

} // cobalt::kinematics::solvers