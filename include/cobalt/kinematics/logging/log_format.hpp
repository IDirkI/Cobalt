#pragma once

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/model/robot_model.hpp"

#include "cobalt/util/meta/platform_info.hpp"

namespace cobalt::kinematics::logging {

constexpr uint8_t  CLOG_MAGIC[4] = {'C', 'O', 'B', 'L'};
constexpr uint8_t  CLOG_VERSION  = 1;
constexpr uint16_t CLOG_FRAME_START = 0xCBCB;

// ---------------- Structs ----------------
/**
 * @brief Log file header format containing magic number, version, robot model information, and JSON header length
 */
struct logFileHeader {
    uint8_t magic[4];
    uint8_t version;
    uint8_t nLinks;
    uint8_t nJoints;
    uint8_t nFrames;
    uint32_t jsonLength;
};

/**
 * @brief Log frame header format containing magic number, frame index, and timestamp
 */
struct logFrameHeader {
    uint16_t magic;
    uint32_t frameIndex;
    uint64_t timestamp;
};

/**
 * @brief Log record format for a link or end-effector frame transform containing position and orientation as a quaternion
 */
struct logTransformRecord {
    float pos[3];
    float quat[4];
};

/**
 * @brief Log record format for a joint containing the joint's transform (position + orientation) and q value
 */
struct logJointRecord {
    float pos[3];
    float quat[4];
    float q;
};


// ---------------- Helper Functions ----------------
/**
 * @brief Get the size in bytes of a single log frame
 * @return Size in bytes of a single log frame
 */
template<id_t nL, id_t nJ, id_t nE>
    constexpr size_t getlogFrameSize() {
        return (sizeof(logFrameHeader) 
            + nL*sizeof(logTransformRecord)
            + nJ*sizeof(logJointRecord)
            + nE*sizeof(logTransformRecord));
    }

/**
 * @brief Build the binary file header to be written at the start of the log fil
 * @param jsonLength Length in bytes of the JSON header to be written after the binary file header
 * @return Binary file header to be written at the start of the log file
 */
template<id_t nL, id_t nJ, id_t nE>
    constexpr inline logFileHeader buildFileHeader(uint32_t jsonLength) {
        logFileHeader h{};
        h.magic[0] = CLOG_MAGIC[0];
        h.magic[1] = CLOG_MAGIC[1];
        h.magic[2] = CLOG_MAGIC[2];
        h.magic[3] = CLOG_MAGIC[3];
        h.version  = CLOG_VERSION;
        h.nLinks  = static_cast<uint8_t>(nL);
        h.nJoints = static_cast<uint8_t>(nJ);
        h.nFrames = static_cast<uint8_t>(nE);
        h.jsonLength = jsonLength;

        return h;
    }

/**
 * @brief Build the binary frame header to be written at the start of each log frame
 * @param frameIndex Index of the frame to be written (starting from 0)
 * @param timestamp Timestamp to associate with the frame in microseconds
 * @return Binary frame header to be written at the start of each log frame
 */
template<id_t nL, id_t nJ, id_t nE>
    constexpr inline logFrameHeader buildFrameHeader(uint32_t frameIndex, uint64_t timestamp) {
        logFrameHeader h{};
        h.magic = CLOG_FRAME_START;
        h.frameIndex = frameIndex;
        h.timestamp = timestamp;

        return h;
    }

// ---------------- Logger Implementation ----------------
#if defined(COBALT_PLATFORM_DESKTOP)
    #include <string>
    #include <sstream>
    #include <iomanip>

    /**
     * @brief Build the JSON header containing robot model information to be written at the start of the log file
     * @param model RobotModel to extract information from to build the JSON header
     * @return JSON header as a string to be written at the start of the log file
     */
    template<id_t nL, id_t nJ, id_t nE>
    std::string buildJsonHeader(const RobotModel<nL, nJ, nE> &model) {
        std::ostringstream j;

        j << "{\n";
        j << "  \"version\": "  << static_cast<int>(CLOG_VERSION) << ",\n";
        j << "  \"robot\": \""  << model.getName() << "\",\n";
        j << "  \"nLinks\": "   << static_cast<int>(nL) << ",\n";
        j << "  \"nJoints\": "  << static_cast<int>(nJ) << ",\n";
        j << "  \"nFrames\": "  << static_cast<int>(nE) << ",\n";

        j << "  \"links\": [";
        for(id_t i = 0; i < nL; i++) {
            j << "\"" << model.getLinks()[i].getName() << "\"";
            if(i < nL - 1) { j << ", "; }
        }
        j << "],\n";

        j << "  \"joints\": [\n";
        for(id_t i = 0; i < nJ; i++) {
            const auto &jt = model.getJoints()[i];
            j << "    {"
            << "\"parent\": \"" << model.getLinks()[jt.getParentId()].getName() << "\", "
            << "\"child\": \""  << model.getLinks()[jt.getChildId()].getName()  << "\", "
            << "\"type\": "     << static_cast<int>(jt.getType())               << ", "
            << "\"axis\": ["
                << std::fixed << std::setprecision(6)
                << jt.getAxis()[0] << ", "
                << jt.getAxis()[1] << ", "
                << jt.getAxis()[2]
            << "]"
            << "}";
            if(i < nJ - 1) { j << ","; }
            j << "\n";
        }
        j << "  ],\n";

        j << "  \"frames\": [\n";
        for(id_t i = 0; i < nE; i++) {
            const auto &f = model.getFrames()[i];
            j << "    {"
            << "\"name\": \""   << f.getName() << "\", "
            << "\"parent\": \"" << model.getLinks()[f.getLinkId()].getName() << "\""
            << "}";
            if(i < nE - 1) { j << ","; }
            j << "\n";
        }
        j << "  ],\n";

        j << "  \"frameLayout\": [\n";
        j << "    {\"field\": \"frameHeader\",      \"bytes\": " << sizeof(logFrameHeader)      << "},\n";
        j << "    {\"field\": \"linkTransforms\",   \"count\": " << static_cast<int>(nL) << ", \"bytesEach\": " << sizeof(logTransformRecord) << "},\n";
        j << "    {\"field\": \"jointRecords\",     \"count\": " << static_cast<int>(nJ) << ", \"bytesEach\": " << sizeof(logJointRecord)     << "},\n";
        j << "    {\"field\": \"frameTransforms\",  \"count\": " << static_cast<int>(nE) << ", \"bytesEach\": " << sizeof(logTransformRecord) << "}\n";
        j << "  ],\n";

        j << "  \"frameSizeBytes\": " << getlogFrameSize<nL, nJ, nE>() << "\n";
        j << "}\n";

        return j.str();
    }

#endif

}; // cobalt::kinematics::logging