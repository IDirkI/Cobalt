#pragma once

#include "cobalt/util/meta/platform_info.hpp"

#if defined(COBALT_PLATFORM_DESKTOP)

#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/robot.hpp"
#include "cobalt/kinematics/logging/snapshot.hpp"
#include "cobalt/kinematics/logging/log_format.hpp"

namespace cobalt::kinematics::logging {

// --------------------------------------
//            Desktop Logger    
// --------------------------------------
/**
 * @brief Logger for recording robot state snapshots to a binary file on desktop platforms
 */
template<id_t nL, id_t nJ, id_t nE>
class DesktopLogger {
    private:
        std::ofstream file_;
        std::string   sessionName_;
        uint32_t      frameCount_;

        /**
         * @brief Resolve the output directory for log files, creating it if it doesn't exist
         * @return Path to the output directory for log files
         * @note Output directory is set to "<project_root>/results/kinematics" and will be created if it doesn't exist
         */
        static std::filesystem::path resolveOutputDir() {
            std::filesystem::path dir = std::filesystem::path(COBALT_PROJECT_ROOT) / "results" / "kinematics";
            std::filesystem::create_directories(dir);
            return dir;
        }

        /**
         * @brief Build the JSON header containing robot model information to be written at the start of the log file
         * @param model RobotModel to extract information from to build the JSON header
         */
        void writeFileHeader(const RobotModel<nL, nJ, nE> &model) {
            const std::string json    = buildJsonHeader<nL, nJ, nE>(model);
            const uint32_t    jsonLen = static_cast<uint32_t>(json.size());

            logFileHeader header = buildFileHeader<nL, nJ, nE>(jsonLen);

            file_.write(reinterpret_cast<const char*>(&header), sizeof(header));
            file_.write(json.data(), jsonLen);
        }

        /**
         * @brief Write a single frame of log data containing link transforms, joint transforms + q values, and end-effector frame transforms to the log file
         * @param snap RobotSnapshot containing the data to write for the frame
         */
        void writeTransformRecord(const cobalt::math::geometry::Transform<> &T) {
            logTransformRecord rec{};
            rec.pos[0] = T.translation()[0];
            rec.pos[1] = T.translation()[1];
            rec.pos[2] = T.translation()[2];
            rec.quat[0] = T.rotation().w();
            rec.quat[1] = T.rotation().x();
            rec.quat[2] = T.rotation().y();
            rec.quat[3] = T.rotation().z();
            file_.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
        }

        /**
         * @brief Write a single joint record containing the joint's transform and q value to the log file
         * @param T Transform of the joint to write
         * @param q Configuration value of the joint to write
         */
        void writeJointRecord(const cobalt::math::geometry::Transform<> &T, float q) {
            logJointRecord rec{};
            rec.pos[0]  = T.translation()[0];
            rec.pos[1]  = T.translation()[1];
            rec.pos[2]  = T.translation()[2];
            rec.quat[0] = T.rotation().w();
            rec.quat[1] = T.rotation().x();
            rec.quat[2] = T.rotation().y();
            rec.quat[3] = T.rotation().z();
            rec.q       = q;

            file_.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
        }

        /**
         * @brief Write a single frame of log data containing link transforms, joint transforms + q values, and end-effector frame transforms to the log file
         * @param snap RobotSnapshot containing the data to write for the frame
         */
        void writeFrame(const RobotSnapshot<nL, nJ, nE> &snap) {
            // Frame header
            logFrameHeader fh = buildFrameHeader<nL, nJ, nE>(frameCount_, snap.timestamp_us);
            file_.write(reinterpret_cast<const char*>(&fh), sizeof(fh));

            // Link transforms
            for(id_t i = 0; i < nL; i++) {
                writeTransformRecord(snap.linkTransforms[i]);
            }

            // Joint transforms + q value
            for(id_t j = 0; j < nJ; j++) {
                writeJointRecord(snap.jointTransforms[j], snap.q[j]);
            }

            // End-effector frame transforms
            for(id_t i = 0; i < nE; i++) {
                writeTransformRecord(snap.frameTransforms[i]);
            }

            frameCount_++;
        }


    public:
        /**
         * @brief Construct a DesktopLogger for a given robot, creating a new log file with the provided name and writing the file header containing the robot model information
         * @param name Name of the log file to create (without extension)
         * @param robot Robot to extract model information from to write in the file header
         * @throws std::runtime_error if the log file cannot be created or opened
         */
        explicit DesktopLogger(const char *name, const Robot<nL, nJ, nE> &robot) : sessionName_(name), frameCount_(0) {
            std::filesystem::path path = resolveOutputDir() / (sessionName_ + ".clog");
            file_.open(path, std::ios::binary | std::ios::trunc);

            if(!file_.is_open()) {
                throw std::runtime_error(std::string("Desktop Logger: failed to open '") + path.string() + "'");
            }

            writeFileHeader(robot.model());
        }

        ~DesktopLogger() {
            if(file_.is_open()) {
                file_.flush();
                file_.close();
            }
        }

        DesktopLogger(const DesktopLogger &) = delete;
        DesktopLogger(DesktopLogger &&) = default;
        DesktopLogger &operator=(const DesktopLogger &) = delete;
        DesktopLogger &operator=(DesktopLogger &&) = default;


        /**
         * @brief Log a snapshot of a robot's state to the log file as a new frame
         * @param snapshot RobotSnapshot containing the data to log for the frame
         */
        void log(const RobotSnapshot<nL, nJ, nE> &snapshot) {
            writeFrame(snapshot);
        }

        /**
         * @brief Log the current state of a robot to the log file as a new frame, capturing a snapshot of the robot's state at the given timestamp
         * @param robot Robot to log the state of
         * @param timestamp_us Timestamp to associate with the logged snapshot in microseconds
         */
        void log(const Robot<nL, nJ, nE> &robot, uint64_t timestamp_us = 0) {
            writeFrame(RobotSnapshot<nL, nJ, nE>::capture(robot, timestamp_us));
        }

        /**
         * @brief Get the number of frames that have been logged so far
         * @return Number of frames that have been logged so far
         */
        uint32_t frameCount() const { return frameCount_; }

        /**
         * @brief Get the name of the logging session (log file name without extension)
         * @return Name of the logging session
         */
        const std::string &name() const { return sessionName_; }


        /**
         * @brief Static helper function to log the current state of a robot to a new log file with the given name
         * @param robot Robot to log the state of
         * @param name Name of the log file to create (without extension)
         * @param timestamp_us Timestamp to associate with the logged snapshot in microseconds
         */
        static void snapshot(const Robot<nL, nJ, nE> &robot, const char *name, uint64_t timestamp_us = 0) {
            DesktopLogger logger(name, robot);
            logger.log(robot, timestamp_us);
        }
};

} // cobalt::kinematics::logger

#endif