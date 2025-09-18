#pragma once

#include <cmath>
#include <string>
#include <array>
#include <stdint.h>

#include "../math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

constexpr float JOINT_DEFAULT_MIN_VALUE = M_PI;
constexpr float JOINT_DEFAULT_MIN_VALUE = -M_PI;
constexpr float JOINT_DEFAULT_INITIAL_VALUE = 0.0f;
constexpr float JOINT_DEFAULT_HOME_VALUE = 0.0f;

enum class JointType {
    Revolute,
    Prismatic
};

// --------------------------------------
//              Robot Joint    
// --------------------------------------
/**
 *  @brief Single joint of a robot in a kinematic chain
 */
struct Joint  {
    private:
        std::string name_; 

        JointType type_;
        float value_;

        cobalt::math::geometry::Transform<float> localTransform_{};
        cobalt::math::geometry::Transform<float> worldTransform_{};

        float valueMin_;
        float valueMax_;

        float homeVal_;

        // ---------------- Helpers  ----------------
        bool clampVal() {
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
         *  @param homeVal Home or "zero" of the joints
         *  @param localTransform Local position + orientation of the joint relative to its parent
         */
        Joint(JointType jointType, float minValue = JOINT_DEFAULT_MIN_VALUE, float maxValue = JOINT_DEFAULT_MIN_VALUE,
              float initialVal = JOINT_DEFAULT_INITIAL_VALUE, float homeVal = JOINT_DEFAULT_HOME_VALUE,
              cobalt::math::geometry::Transform<float> localTransform = cobalt::math::geometry::Transform<float>::eye())
         : type_(jointType), valueMin_(minValue), valueMax_(maxValue), name_(""), localTransform_(localTransform), worldTransform_(localTransform) {}


        // ---------------- Member Functions ----------------
        /**
         *  @brief Set the value(angle or length) of the joint safely
         *  @param val Value to set the joint value(angle or length) to
         *  @return `true` if the value was valid, `false` if the value saturated the joint (exceeds min or max)
         */
        bool setValue(float val) {
            value_ = val;

            return clampVal();
        }
};



// --------------------------------------
//         Robot Joint Chain  
// --------------------------------------
/**
 *  @brief Kinematic chain representation in a robot
 *  @tparam N Number of joints in the chain
 */
template<uint8_t N>
struct JointChain  {
    private:
        std::array<Joint, N> joints_{};

        // ---------------- Helpers  ----------------
        bool clampVal() {
            bool notSaturated = true;

            for(Joint &j : joints_) {
                if(!j.clampVal()) { notSaturated = false; }
            }

            return notSaturated;
        }


    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a kinematic chain of joints
         */
        JointChain() {}


        // ---------------- Accessors ----------------
        /**
         *  @brief Access joint at the given index.
         *  @param n Index of the accessed joint.
         *  @return Reference to the joint.
         */
        Joint &operator[](uint8_t n) { if(n >= N) n = N-1; return data_[n]; }

        /**
         *  @brief Const access to joint at the given index.
         *  @param n Index of the accessed joint.
         *  @return Const reference to the joint.
         */
        const Joint &operator[](uint8_t n) const { if(n >= N) n = N-1; return data_[n]; }


        // ---------------- Getters ----------------
        constexpr uint8_t size() const { return N; }


        // ---------------- Member Functions ----------------
        /**
         *  @brief Set the values(angle or length) of the joints in the joint-chain safely
         *  @param vals Values(angle or length) to set the joints to
         *  @return `true` if the values were valid, `false` if the a value saturated a joint (exceeded min or max)
         */
        bool setValue(const std::array<Joint, N> &vals) {
            for(uint8_t i = 0; i < N; i++) {
                joints_[i].setValue(values[i]);
            }

            return clampVal();
        }
};

} // cobalt::kinematics