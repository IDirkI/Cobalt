#pragma once

#include "cobalt/kinematics/config.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_util.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

/**
 *  @brief Type of a joint in a robot. (fixed, revolute, prismatic, etc.)
 */
enum class JointType : uint8_t {
    Fixed,
    Revolute,
    Prismatic
};

/**
 *  @brief Wrapper for representing the limits and limitedness of a joint
 */
struct JointLimits {
    float min = 0.0f;
    float max = 0.0f;
    bool enabled = false;
};

constexpr id_t JOINT_DEFAULT_ID = invalidID_;
constexpr float JOINT_DEFAULT_HOME = 0.0f;
constexpr JointType JOINT_DEFAULT_TYPE = JointType::Fixed;

const cobalt::math::geometry::Transform<> JOINT_DEFAULT_ORIGIN = cobalt::math::geometry::Transform<>::eye();
const cobalt::math::linear_algebra::Vector<3> JOINT_DEFAULT_AXIS = cobalt::math::linear_algebra::Vector<3>::unitZ();


// --------------------------------------
//              Robot Joint    
// --------------------------------------
/**
 *  @brief Joint of a robot in a robot chain
 */
struct Joint  {
    private:
        id_t id_{JOINT_DEFAULT_ID};
        id_t idParent_{JOINT_DEFAULT_ID};
        id_t idChild_{JOINT_DEFAULT_ID};

        JointType type_{JOINT_DEFAULT_TYPE};

        cobalt::math::geometry::Transform<> origin_{JOINT_DEFAULT_ORIGIN};
        cobalt::math::linear_algebra::Vector<3> axis_{JOINT_DEFAULT_AXIS};
        
        JointLimits limits_{};
        float home_{JOINT_DEFAULT_HOME};

        // ---------------- Helper Function ----------------
        constexpr void validate() {
            if(type_ != JointType::Fixed) { 
                    assert((isZero(axis_) == false) && "[JOINT Error] : Joint axis cannot be zero vector.");
                    axis_ = normalize(axis_); 
                }
                else {
                    axis_ = cobalt::math::linear_algebra::Vector<3>::zero();
                    limits_.enabled = false; 
                }

                if(limits_.enabled) { assert((limits_.min <= limits_.max) && "[JOINT Error] : Joint limits are invalid."); }

                if(idParent_ != invalidID_ && idChild_ != invalidID_) {
                    assert((idParent_ != idChild_) && "[JOINT Error] : Joint parent and child link IDs cannot be the same.");
                }
        }

    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a robot joint
         *  @param id ID of the joint
         *  @param idParent ID of the parent link
         *  @param idChild ID of the child link
         *  @param type Type of the joint
         *  @param origin Transform of the joint relative to the parent link frame
         *  @param axis Axis of rotation/translation for revolute/prismatic joints
         *  @param limits Joint limits for revolute/prismatic joints
         *  @param home Home position of the joint
         *  @note Validates the joint parameters upon construction
         *  @throws AssertionError if the joint parameters are invalid
         */
        explicit Joint(id_t id = JOINT_DEFAULT_ID,
                       id_t idParent = JOINT_DEFAULT_ID,
                       id_t idChild = JOINT_DEFAULT_ID,
                       JointType type = JOINT_DEFAULT_TYPE,
                       const cobalt::math::geometry::Transform<> &origin = JOINT_DEFAULT_ORIGIN,
                       const cobalt::math::linear_algebra::Vector<3> &axis = JOINT_DEFAULT_AXIS,
                       const JointLimits &limits = JointLimits(),
                       float home = JOINT_DEFAULT_HOME)
            : id_(id), idParent_(idParent), idChild_(idChild), type_(type), origin_(origin), axis_(axis), limits_(limits), home_(home) {
                validate();
            }

        // ---------------- Getters ----------------
        /**
         *  @brief Get the ID of the joint
         *  @return ID of the joint
         */
        constexpr id_t getId() const { return id_;}
        /**
         *  @brief Get the ID of the parent link
         *  @return ID of the parent link
         */
        constexpr id_t getParentId() const { return idParent_; }
        /**
         *  @brief Get the ID of the child link
         *  @return ID of the child link
         */
        constexpr id_t getChildId() const { return idChild_; }
        /**
         *  @brief Get the type of the joint
         *  @return JointType enum indicating the type of joint
         */
        constexpr JointType getType() const { return type_; }
        /**
         *  @brief Get the joint limits
         *  @return JointLimits struct containing joint limit information
         */
        constexpr float getMinLimit() const { return limits_.min; }
        /**
         *  @brief Get the maximum joint limit
         *  @return Maximum joint limit
         */
        constexpr float getMaxLimit() const { return limits_.max; }
        /**
         *  @brief Get the home position of the joint
         *  @return Home position of the joint
         */
        constexpr float getHome() const { return home_; }
        /**
         *  @brief Check if joint limits are enabled
         *  @return True if joint limits are enabled, false otherwise
         */
        constexpr bool areLimitsEnabled() const { return limits_.enabled; }

        /**
         *  @brief Get the origin transform of the joint
         *  @return Transform of the joint relative to the parent link frame
         */
        const cobalt::math::geometry::Transform<> &getOrigin() const { return origin_; }
        /**
         *  @brief Get the axis of the joint
         *  @return Axis of rotation/translation for revolute/prismatic joints
         */
        const cobalt::math::linear_algebra::Vector<3> &getAxis() const { return axis_; }
    };

} // cobalt::kinematics