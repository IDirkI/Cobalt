#pragma once

#include "config.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_util.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

enum class JointType {
    Fixed,
    Revolute,
    Prismatic
};

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
        void enforceConstraints() {
            if(type_ != JointType::Fixed) { 
                    assert((isZero(axis_) == false) && "[JOINT Error] : Joint axis cannot be zero vector.");
                    axis_ = normalize(axis_); 
                }
                else {
                    axis_ = cobalt::math::linear_algebra::Vector<3>::zero();
                    limits_.enabled = false; 
                }

                if(limits_.enabled) { assert((limits_.min <= limits_.max) && "[JOINT Error] : Joint limits are invalid."); }
        }

    public: 
        // ---------------- Constructors ----------------
        explicit Joint(id_t id = JOINT_DEFAULT_ID,
              id_t idParent = JOINT_DEFAULT_ID,
              id_t idChild = JOINT_DEFAULT_ID,
              JointType type = JOINT_DEFAULT_TYPE,
              const cobalt::math::geometry::Transform<> &origin = JOINT_DEFAULT_ORIGIN,
              const cobalt::math::linear_algebra::Vector<3> &axis = JOINT_DEFAULT_AXIS,
              const JointLimits &limits = JointLimits(),
              float home = JOINT_DEFAULT_HOME)
            : id_(id), idParent_(idParent), idChild_(idChild), type_(type), origin_(origin), axis_(axis), limits_(limits), home_(home) {
                enforceConstraints();
            }

        // ---------------- Getters ----------------
        constexpr id_t getId() const { return id_;}
        constexpr id_t getParentId() const { return idParent_; }
        constexpr id_t getChildId() const { return idChild_; }
        constexpr JointType getType() const { return type_; }
        constexpr float getMinLimit() const { return limits_.min; }
        constexpr float getMaxLimit() const { return limits_.max; }
        constexpr float getHome() const { return home_; }
        constexpr bool areLimitsEnabled() const { return limits_.enabled; }

        const cobalt::math::geometry::Transform<> &getOrigin() const { return origin_; }
        const cobalt::math::linear_algebra::Vector<3> &getAxis() const { return axis_; }

        

    };

} // cobalt::kinematics