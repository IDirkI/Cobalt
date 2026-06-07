#pragma once

#include <utility>

#include "config.hpp"
#include "model/robot_model.hpp"
#include "state/robot_state.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"

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
        Robot(const Robot &&robot) noexcept
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

        // ---------------- Member Functions ----------------
        /**
         *  @brief Set the joints of the robot with a configuration vector
         *  @param q Configuration vector to set the joints to
         *  @return `true` if none of the joints hit their limit, `false` otherwise
         *  @note Joint value and limits are always relative to the joints home value
         *  @warning Values exceeding a joints limit will be clamped 
         */
        inline bool setJoints(const cobalt::math::linear_algebra::Vector<nJ> q) {
            bool withinLimits = true;
            
            for(id_t j = 0; j < nJ; j++) {
                float value = q[j];

                if(value < model_.getJoints()[j].getMinLimit()) { 
                    value = model_.getJoints()[j].getMinLimit(); 
                    withinLimits = false;
                }
                else if (model_.getJoints()[j].getMaxLimit() < value) { 
                    value = model_.getJoints()[j].getMaxLimit(); 
                    withinLimits = false;
                }

                state_.q[j] = value;
            }

            state_.validLinks = false;
            state_.validJoints = false;
            state_.validFrames= false;
            state_.validJ = false;

            return withinLimits;
        }
};

}  // cobalt::kinematics