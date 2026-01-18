#pragma once

#include <utility>

#include "config.hpp"
#include "model/robot_model.hpp"
#include "state/robot_state.hpp"

namespace cobalt::kinematics {

// --------------------------------------
//               Robot    
// --------------------------------------
/**
 *  @brief Wrapper struct for a robot described by a RobotModel and RobotState
 */
template<id_t nL, id_t nJ, id_t nE>
struct Robot {
    private:
        const RobotModel<nL, nJ, nE> &model_;
        RobotState<nL, nJ, nE> state_;

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a robot from a robot model and robot state
         *  @param model Reference to the robot model
         *  @param state Reference to the robot state
         *  @note Wrapper stores reference to model and a robot owned state
         */
        explicit Robot(const RobotModel<nL, nJ, nE> &model, RobotState<nL, nJ, nE> state) noexcept
            : model_(model), state_(state) {}

        /**
         *  @brief Constructor a robot from a copied robot
         *  @param robot Robot to copy
         */
        Robot(Robot &&robot) noexcept
            : model_(robot.model_), state_(std::move(robot.state_)) {}

        // ---------------- Accessors ----------------
        /**
         * @brief Access to reference to the robot state
         * @return Reference to the robot state
         */
        constexpr RobotState<nL, nJ, nE> &state() { return state_; }

        /**
         * @brief Const access to reference to the robot model
         * @return Const reference to the robot model
         */
        const RobotModel<nL, nJ, nE> &model() const { return model_; }
        /**
         * @brief Const access to reference to the robot state
         * @return Const reference to the robot state
         */
        const RobotState<nL, nJ, nE> &state() const { return state_; }
};

}  // cobalt::kinematics