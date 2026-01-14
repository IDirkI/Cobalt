#pragma once

#include "config.hpp"
#include "robot_model.hpp"
#include "robot_state.hpp"

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
        RobotState<nL, nJ, nE> &state_;

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a robot from a robot model and robot state
         *  @param model Reference to the robot model
         *  @param state Reference to the robot state
         *  @note Wrapper stores reference to both model and state
         */
        explicit Robot(RobotModel<nL, nJ, nE> &model,
                       RobotState<nL, nJ, nE> &state)
            : model_(model), state_(state) {}

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