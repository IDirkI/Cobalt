#pragma once

#include "config.hpp"

namespace cobalt::kinematics {

enum class JointType {
    None,
    Fixed,
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
        id_t id_;
        

    public: 

};

} // cobalt::kinematics