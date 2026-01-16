#pragma once

#include "config.hpp"           // Kinematics config

#include "robot.hpp"                // Robot wrapper struct
#include "state/robot_state.hpp"        // -- Robot state holding the current robot configuration
#include "model/robot_model.hpp"        // -- Robot model holding robot structure and topology
#include "core/joint.hpp"                   // ---- Joints of a robot 
#include "core/link.hpp"                    // ---- Link of a robot
#include "core/frame_attachment.hpp"        // ---- Tools/Frames attached to links

#include "solvers/forward_kinematics.hpp"   // Forward Kinematics solver for general robots

#include "util/robot_logger.hpp"    // RobotState CVS file generator