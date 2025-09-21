#pragma once

#include <cmath>
#include <string>
#include <array>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr uint8_t JOINT_DEFAULT_PARENT_INDEX = -1;
constexpr uint8_t JOINT_DEFAULT_CHILD_INDEX = -1;

constexpr float JOINT_DEFAULT_MIN_VALUE = -M_PI;
constexpr float JOINT_DEFAULT_MAX_VALUE = M_PI;
constexpr float JOINT_DEFAULT_INITIAL_VALUE = 0.0f;
constexpr float JOINT_DEFAULT_HOME_VALUE = 0.0f;

const std::string JOINT_DEFAULT_NAME = "";

enum class JointType {
    Revolute,
    Prismatic
};

// --------------------------------------
//              Robot Joint    
// --------------------------------------
/**
 *  @brief Single joint of a robot in a robot chain
 */
struct Joint  {
    private:
        std::string name_; 
        JointType type_;

        float value_;
        float valueMin_;
        float valueMax_;

        float homeValue_;

        uint8_t parentLinkIndex_;
        uint8_t childLinkIndex_;

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
         *  @param amxValue Maximum joint value (angle or length) allowed
         *  @param initialVal Initial value the joint will be at
         *  @param localTransform Local position + orientation of the joint relative to its parent
         */
        Joint(JointType jointType, uint8_t parentIndex = JOINT_DEFAULT_PARENT_INDEX, uint8_t childIndex = JOINT_DEFAULT_CHILD_INDEX,
              float minValue = JOINT_DEFAULT_MIN_VALUE, float maxValue = JOINT_DEFAULT_MAX_VALUE,
              float initialVal = JOINT_DEFAULT_INITIAL_VALUE, float homeVal = JOINT_DEFAULT_HOME_VALUE, std::string name = JOINT_DEFAULT_NAME)
         : type_(jointType), parentLinkIndex_(parentIndex), childLinkIndex_(childIndex), valueMin_(minValue), valueMax_(maxValue), value_(initialVal), homeValue_(homeVal), name_(name) {
            clampVal();
         }


        // ---------------- Getters ----------------
        /**
         *  @brief Get the name of the joint
         */
        std::string getName() const { return name_; }

        /**
         *  @brief Get the type of the joint. 
         *  @return `Revolute` or `Prismatic`
         */
        constexpr JointType getType() const { return type_; }

        /**
         *  @brief Get the value of the joint.
         *  @return `rotation` or `length`
         */
        constexpr float getValue() const { return value_; }

        /**
         *  @brief Get the parent link index of the joint.
         */
        constexpr uint8_t getParentIndex() const { return parentLinkIndex_; }

        /**
         *  @brief Get the child link index of the joint.
         */
        constexpr uint8_t getChildIndex() const { return childLinkIndex_; }

        // ---------------- Setters ----------------
        /**
         *  @brief Set the joint name.
         */
        void setName(std::string name) { name_ = name; }

        /**
         *  @brief Set the value(angle or length) of the joint safely
         *  @param val Value to set the joint value(angle or length) to
         *  @return `true` if the value was valid, `false` if the value saturated the joint (exceeds min or max)
         */
        constexpr bool setValue(float val) {
            value_ = val;

            return clampVal();
        }
};

} // cobalt::kinematics