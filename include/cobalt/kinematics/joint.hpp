#pragma once

#include <cmath>
#include <string>
#include <array>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

#include "../math/linear_algebra/vector/vector.hpp"
#include "../math/linear_algebra/vector/vector_ops.hpp"

namespace cobalt::kinematics {

enum class JointType {
    Revolute,
    Prismatic
};

constexpr JointType JOINT_DEFAULT_TYPE = JointType::Revolute;
constexpr int8_t JOINT_DEFAULT_ID = -1;

constexpr int8_t JOINT_DEFAULT_PARENT_INDEX = -1;
constexpr int8_t JOINT_DEFAULT_CHILD_INDEX = -1;

constexpr float JOINT_DEFAULT_MIN_VALUE = -M_PI;
constexpr float JOINT_DEFAULT_MAX_VALUE = M_PI;
constexpr float JOINT_DEFAULT_INITIAL_VALUE = 0.0f;
constexpr float JOINT_DEFAULT_HOME_VALUE = 0.0f;

const cobalt::math::linear_algebra::Vector<3> JOINT_DEFAULT_MOTION_AXIS = cobalt::math::linear_algebra::Vector<3>::unitZ();

// --------------------------------------
//              Robot Joint    
// --------------------------------------
/**
 *  @brief Single joint of a robot in a robot chain
 */
struct Joint  {
    private:
        JointType type_;
        int8_t id_;

        cobalt::math::linear_algebra::Vector<3> axis_;

        int8_t parent_;
        int8_t child_;

        float value_;       // Relative to homeValue_
        float valueMin_;    // Relative to homeValue_
        float valueMax_;    // Relative to homeValue_
        float homeValue_;

        // ---------------- Helpers  ----------------
        constexpr bool clampVal() {
            bool notSaturated = true;
            if(value_ > valueMax_) { value_ = valueMax_; notSaturated = false; }
            if(value_ < valueMin_) { value_ = valueMin_; notSaturated = false; }
            return notSaturated;
        }


    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a joint of a robot
         *  @param jointType Type of the joint. Revolute or Prismatic
         *  @param minValue Minimum joint value (angle or length) allowed
         *  @param maxValue Maximum joint value (angle or length) allowed
         *  @param initialVal Initial value the joint will be at
         *  @param localTransform Local position + orientation of the joint relative to its parent
         */
        Joint(JointType jointType = JOINT_DEFAULT_TYPE, uint8_t jointId = JOINT_DEFAULT_ID, uint8_t parentIndex = JOINT_DEFAULT_PARENT_INDEX, uint8_t childIndex = JOINT_DEFAULT_CHILD_INDEX,
              cobalt::math::linear_algebra::Vector<3> axis = JOINT_DEFAULT_MOTION_AXIS, float minValue = JOINT_DEFAULT_MIN_VALUE, float maxValue = JOINT_DEFAULT_MAX_VALUE,
              float initialVal = JOINT_DEFAULT_INITIAL_VALUE, float homeVal = JOINT_DEFAULT_HOME_VALUE)
         : type_(jointType), id_(jointId), parent_(parentIndex), child_(childIndex), axis_(axis), valueMin_(minValue), valueMax_(maxValue), value_(initialVal), homeValue_(homeVal) {
            axis_ = normalize(axis_);
            clampVal();
         }


        // ---------------- Getters ----------------
        /**
         *  @brief Get the type of the joint. 
         *  @return `Revolute` or `Prismatic`
         */
        constexpr JointType getType() const { return type_; }

        /**
         *  @brief Get the id of the joint. 
         */
        constexpr uint8_t getId() const { return id_; }

        /**
         *  @brief Get the axis of motion of the joint.
         *  @return Rotation axis or translation axis
         */
        constexpr cobalt::math::linear_algebra::Vector<3> getAxis() const { return axis_; }

        /**
         *  @brief Get the parent link index of the joint.
         */
        constexpr int8_t getParent() const { return parent_; }

        /**
         *  @brief Set the child link index of the joint.
         */
        constexpr int8_t getChild() const { return child_; }

        /**
         *  @brief Get the world value of the joint.
         *  @return `rotation` or `length`
         */
        constexpr float getValue() const { return value_ + homeValue_; }

        /**
         *  @brief Get the minimum value the joint can go to
         *  @return Minimum `rotation` or `length`
         */
        constexpr float getMinLimit() const { return valueMin_; }

        /**
         *  @brief Get the maximum value the joint can go to
         *  @return Maximum `rotation` or `length`
         */
        constexpr float getMaxLimit() const { return valueMax_; }

        /**
         *  @brief Get the home/offset value of the joint which its value is relative to
         */
        constexpr float getHome() const { return homeValue_; }

        // ---------------- Setters ----------------
        /**
         *  @brief Set the type of the joint
         *  @param type The type of the joint. Either `Revolute` or `Prismatic`
         */
        constexpr void setType(JointType type) { type_ = type; }

        /**
         *  @brief Set the unique Id of the joint
         *  @param id Unique Id of the joint. Should match its index i the RobotChain array
         */
        constexpr void setId(float id) { id_ = id; }

        /**
         *  @brief Set the axis of motion of the joint.
         *  @param axis Normalized Vector<3> axis that the joint movement happens in
         */
        constexpr void setAxis(cobalt::math::linear_algebra::Vector<3> axis) { axis_ = normalize(axis); }

        /**
         *  @brief Set the parent link index of the joint.
         *  @param parentIndex The index of the parent link of the joint
         */
        constexpr void setParent(int8_t parentIndex) { parent_ = parentIndex; }

        /**
         *  @brief Set the child link index of the joint.
         *  @param childIndex The index of the child link of the joint
         */
        constexpr void setChild(int8_t childIndex) { child_ = childIndex; }

        /**
         *  @brief Set the value(angle or length) of the joint safely
         *  @param val Value to set the joint value(angle or length) to
         *  @return `true` if the value was valid, `false` if the value saturated the joint (exceeds min or max)
         */
        constexpr bool setValue(float val) {
            value_ = val;

            return clampVal();
        }

        /**
         *  @brief Set the minimum and maximum value the joint can go to
         *  @note `min` < `max` must be true(valid min/max) otherwise values will not be set
         *  @return `true` if the limits are set successfully, `false` otherwise
         */
        constexpr bool setLimits(float min, float max) { 
            if(min < max) {
                valueMax_ = max;
                valueMin_ = min;
                return true;
            }
            return false;
        }

        /**
         *  @brief Set the home/offset value of the joint which its value is relative to
         *  @param home Home/offset value the joint moves around
         */
        constexpr void setHome(float home) { 
            homeValue_ = home;
        }
};

} // cobalt::kinematics