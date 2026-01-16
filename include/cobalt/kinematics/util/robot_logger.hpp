#pragma once

#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"
#include "cobalt/kinematics/state/robot_state.hpp"

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics::util {

template <id_t nL, id_t nJ, id_t nE>
/**
 *  @brief Log the current robot state into a CSV file for inspection & debugging
 *  @param robot Robot to log the state of
 *  @param filename Output filename
 *  @note This is only supported on the PC end, not supported in embedded hardware. Intended to developer test
 */
void logRobotState(const Robot<nL, nJ, nE> &robot, const std::string &filename) {
    // Base folder: <projRoot>/results/kinematics/
    std::filesystem::path projRoot = std::filesystem::current_path();
    std::filesystem::path logDir = projRoot / "../.." / "results" / "kinematics";

    // Create directories if they don't exist
    std::filesystem::create_directories(logDir);

    // Full file path
    std::filesystem::path filePath = logDir / (filename + ".csv");

    std::ofstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filePath.string());
    }

    // CSV header
    file << "type,name,parent,child,x,y,z,qw,qx,qy,qz,axis_x,axis_y,axis_z,value,joint_type\n";
    
    const RobotState<nL, nJ, nE> &state = robot.state();
    const RobotModel<nL, nJ, nE> &model = robot.model();

    // --- Links ---
    for(id_t i = 0; i < nL; i++) {
        const cobalt::math::geometry::Transform<> &T = state.linkTransforms[i];
        
        file << "link,"
             << model.getLinks()[i].getName() << ","
             << ",,"        // parent, child (empty for links)
             << std::fixed << std::setprecision(6)
             << T.translation()[0] << ","
             << T.translation()[1] << ","
             << T.translation()[2] << ","
             << T.rotation().w() << ","
             << T.rotation().x() << ","
             << T.rotation().y() << ","
             << T.rotation().z() << ","
             << ",,,"       // axis_x, axis_y, axis_z (empty for links)
             << ","         // value (empty for links)
             << "\n";       // joint_type (empty for links)
    }

    // --- Joints ---
    for(id_t j = 0; j < nJ; j++) {
        const Joint &joint = model.getJoints()[j];
        
        // Debug output to console
        std::cout << "Joint " << j << ": type = " << static_cast<int>(joint.getType()) 
                  << " (0=Fixed, 1=Revolute, 2=Prismatic)" << std::endl;
        
        // Compute joint position in world frame
        const cobalt::math::geometry::Transform<> &T_parent = state.linkTransforms[joint.getParentId()];
        cobalt::math::geometry::Transform<> T_joint = T_parent * joint.getOrigin();
        
        // Transform joint axis to world frame
        cobalt::math::linear_algebra::Vector<3> axis_world = 
            cobalt::math::geometry::rotate(T_parent.rotation(), joint.getAxis());
        
        file << "joint,"
             << ","         // name (empty for joints)
             << model.getLinks()[joint.getParentId()].getName() << ","
             << model.getLinks()[joint.getChildId()].getName() << ",";
        
        // Position with fixed precision
        file << std::fixed << std::setprecision(6)
             << T_joint.translation()[0] << ","
             << T_joint.translation()[1] << ","
             << T_joint.translation()[2] << ","
             << ",,,,"       // qw, qx, qy, qz (empty for joints)
             << axis_world[0] << ","
             << axis_world[1] << ","
             << axis_world[2] << ","
             << state.q[j] << ",";
        
        // Joint type as plain integer (no fixed precision)
        file << static_cast<int>(joint.getType()) << "\n";
    }
    
    // --- Frames ---
    for(id_t i = 0; i < nE; i++) {
        const cobalt::math::geometry::Transform<> &T = state.frameTransforms[i];
        const FrameAttachment &frame = model.getFrames()[i];
        
        file << "frame,"
             << frame.getName() << ","
             << model.getLinks()[frame.getLinkId()].getName() << ","
             << ","         // child (empty for frames)
             << std::fixed << std::setprecision(6)
             << T.translation()[0] << ","
             << T.translation()[1] << ","
             << T.translation()[2] << ","
             << T.rotation().w() << ","
             << T.rotation().x() << ","
             << T.rotation().y() << ","
             << T.rotation().z() << ","
             << ",,,"       // axis_x, axis_y, axis_z (empty for frames)
             << ","         // value (empty for frames)
             << "\n";       // joint_type (empty for frames)
    }

    file.close();
    
    std::cout << "Robot state logged to: " << filePath << std::endl;
}

} // cobalt::kinematics::util