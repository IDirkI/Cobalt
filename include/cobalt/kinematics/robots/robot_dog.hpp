#pragma once

#include "../joint.hpp"
#include "../link.hpp"
#include "../robot_chain.hpp"

//------------------------
//        robot_dog       
//------------------------
/** 
 * This is an automatically generated 'robot_dog' RobotChain from its .rob file.
 * Include 'robot_dog.hpp' in your project upon calling it to access and use the created 'robot_dog' chain.
 * Do not touch this file for any edits and use the .rob file for any edits.
 */ 
namespace cobalt::kinematics::robot {

  const std::array<Link, 3> robot_dog_links = {
      Link("base", -1.0f, 0, 0, 0.0f),
      Link("dog/leg1_high", 0.3f, 1, 1, 0.4f),
      Link("dog/leg1_low", 0.2f, 2, -1, 0.3f),
  };

  const std::array<Joint, 2> robot_dog_joints = {
      Joint(JointType::Revolute, 0, 0, 1,
            cobalt::math::linear_algebra::Vector<3>(0.0f, 0.0f, -1.0f),
            -1.5707963267948966f, 1.5707963267948966f, 0.0f, 0.0f),
      Joint(JointType::Revolute, 1, 1, 2,
            cobalt::math::linear_algebra::Vector<3>(0.0f, 0.0f, -1.0f),
            -1.5707963267948966f, 1.5707963267948966f, 0.0f, 1.5707963267948966f),
  };

  inline RobotChain<3, 2> robot_dog(robot_dog_links, robot_dog_joints);

}; // cobalt::kinematics::robot