#pragma once

#include <array>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"
#include "cobalt/kinematics/state/robot_state.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

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
    public:
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
            std::array<id_t, nJ> childCount{};
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
        void solve(RobotState<nL, nJ, nE> &state) const;
        FKSolution<nL, nE> solve(cobalt::math::linear_algebra::Vector<nJ> &q) const;
};

}; // cobalt::kinematics::solver