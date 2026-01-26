#pragma once

#include <array>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"
#include "cobalt/kinematics/state/robot_state.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"

namespace cobalt::kinematics::solvers {

/**
 *  @brief FK Solver output format containing link & frame transforms 
 */
template<id_t nL, id_t nE>
struct FKSolution {
    std::array<cobalt::math::geometry::Transform<>, nL> linkTransforms{};
    std::array<cobalt::math::geometry::Transform<>, nE> frameTransforms{};

    bool valid{false};
};

// --------------------------------------
//          Forward Kinematics
// --------------------------------------
/**
 *  @brief FK solver for a robot made up of RobotModel and RobotState
 */
template<id_t nL, id_t nJ, id_t nE>
class ForwardKinematics {
    private:
        struct JointFKData {
            id_t idParent;
            id_t idChild;

            JointType type;

            cobalt::math::linear_algebra::Vector<3> axis;
            cobalt::math::geometry::Transform<> origin;
        };

        std::array<JointFKData, nJ> jointData_;
        std::array<id_t, nJ> jointOrder_;

        const RobotModel<nL, nJ, nE> &model_;

        // ---------------- Helper Functions ----------------
        void fillJointData(const RobotModel<nL, nJ, nE> &model) {
            struct Edge {
                id_t parent;
                id_t child; 
            };

            std::array<Edge, nJ> edges{};
            std::array<id_t, nL> childCount{};
            childCount.fill(0);

            // Fill JointFKData
            for(id_t j = 0; j < nJ; j++) {
                const Joint &joint = model.getJoints()[j];
                JointFKData &data = jointData_[j];

                data.idParent = joint.getParentId();
                data.idChild = joint.getChildId();
                data.type = joint.getType();
                data.axis = joint.getAxis();
                data.origin = joint.getOrigin();

                edges[j] = {data.idParent, data.idChild};
                childCount[data.idParent]++;
            }

            // Compute traversal order
            id_t order = 0;
            id_t size = 0;
            std::array<id_t, nL> stack{};

            std::array<bool, nL> isRoot{};
            isRoot.fill(true);
            for(const Edge &e : edges) {
                isRoot[e.child] = false;
            }

            for(id_t i = 0; i < nL; i++) {
                if(isRoot[i]) stack[size++] = i;
            }

            while(size > 0) {   // dfs
                size--;
                id_t link = stack[size];

                for(id_t j = 0; j < nJ; j++) {
                    if(jointData_[j].idParent == link) {
                        jointOrder_[order++] = j;
                        stack[size++] = jointData_[j].idChild;
                    }
                }
            }
        }

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a forward kinematics solver for an arbitrary robot
         *  @param robot Robot to solve FK for
         */
        explicit ForwardKinematics(const Robot<nL, nJ, nE> &robot) : model_(robot.model()) { fillJointData(model_); }

        /**
         *  @brief Construct a forward kinematics solver for an arbitrary robot
         *  @param model Model of a robot to solve FK for
         */
        explicit ForwardKinematics(const RobotModel<nL, nJ, nE> &model) : model_(model) { fillJointData(model_); }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Given a current robot state solve the FK for the known RobotModel to solve for link & frame transformations
         *  @param state Current RobotState of a robot to solve FK for
         */
        void solve(RobotState<nL, nJ, nE> &state) const {
            state.validLinks = false;
            for(id_t j : jointOrder_) { // Link Transforms
                const JointFKData &jd = jointData_[j];

                const cobalt::math::geometry::Transform<> T_parent = state.linkTransforms[jd.idParent];
                const cobalt::math::geometry::Transform<> T_link = model_.getLinks()[jd.idChild].getOrigin();
                cobalt::math::geometry::Transform<> &T_child = state.linkTransforms[jd.idChild];

                cobalt::math::geometry::Transform<> T_joint = jd.origin;
                cobalt::math::geometry::Transform<> T_motion = cobalt::math::geometry::Transform<>::eye();

                switch(jd.type) {
                    case (JointType::Prismatic): {
                        T_motion.translate( // Issue with transformation
                            cobalt::math::geometry::rotate(inv(T_joint.rotation()), jd.axis) * (state.q[j] + model_.getJoints()[j].getHome())
                           //jd.axis * (state.q[j] + model_.getJoints()[j].getHome())
                        );
                        break;
                    }
                    case (JointType::Revolute): {
                        T_motion.rotate(
                            cobalt::math::geometry::Quaternion<>::fromAxisAngle(
                                jd.axis, (state.q[j] + model_.getJoints()[j].getHome())
                            )
                        );
                        break;
                    }
                    case (JointType::Fixed): {
                        break;
                    }
                    default: {  // Default: Fixed
                        break;
                    }
                }

                T_child = T_parent * T_joint * T_motion * T_link;
            }
            state.validLinks = true;

            state.validFrames = false;
            for(const FrameAttachment &f : model_.getFrames()) { // Frame Transforms
                const cobalt::math::geometry::Transform<> T_parent = state.linkTransforms[f.getLinkId()];
                cobalt::math::geometry::Transform<> &T_frame = state.frameTransforms[f.getId()];

                cobalt::math::geometry::Transform<> T_origin = f.getOrigin();

                T_frame = T_parent * T_origin;
            }
            state.validFrames = true;
        }

        /**
         *  @brief Given a current robot configuration solve the FK for the known RobotModel to solve for link & frame transformations
         *  @param q Current configuration vector for a robot without full RobotState bloat
         *  @return Struct containing link and frame transformation results as well as a valid solution falg
         */
        FKSolution<nL, nE> solve(cobalt::math::linear_algebra::Vector<nJ> &q) const {
            FKSolution<nL, nE> output;

            RobotState<nL, nJ, nE> tmp;
            tmp.q() = q;
            solve(tmp);

            output.linkTransforms = tmp.linkTransforms;
            output.frameTransforms = tmp.frameTransforms;
            output.valid = (tmp.validLinks && tmp.validFrames);

            return output;
        }
};

}; // cobalt::kinematics::solver