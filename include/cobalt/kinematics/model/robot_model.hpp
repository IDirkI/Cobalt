#pragma once

#include <array>
#include <algorithm>

#include "cobalt/kinematics/config.hpp"
#include "cobalt/kinematics/core/link.hpp"
#include "cobalt/kinematics/core/joint.hpp"
#include "cobalt/kinematics/core/frame_attachment.hpp"

namespace cobalt::kinematics {

/**
 *  @brief Type of a robot based on its structure
 */
enum class RobotType : std::uint8_t {
    Invalid,
    Serial,
    Tree,
    Parallel,
};

// --------------------------------------
//         Kinematic Path   
// --------------------------------------
/**
 *  @brief Traversal order of a path, usually from base -> end effector frame
 */
template<id_t nJ>
struct KinematicPath {
    public:
        std::array<id_t, nJ> joints{};
        id_t length{0};

        bool contains(id_t jointId) const {
            for(id_t i = 0; i < length; i++) {
                if(jointId == joints[i]) { return true; }
            }
            return false;
        }
};

// --------------------------------------
//         Robot Model    
// --------------------------------------
/**
 *  @brief Robot model detailing the topology of a robot
 */
template<id_t nL, id_t nJ, id_t nE>
struct RobotModel {
    private:
        const char *name_{""};
        RobotType type_{RobotType::Invalid};

        std::array<Link, nL> links_{};
        std::array<Joint, nJ> joints_{};
        std::array<FrameAttachment, nE> frames_{};

        std::array<KinematicPath<nJ>, nL> linkPaths_{};
        std::array<KinematicPath<nJ>, nE> framePaths_{};

        // ---------------- Private Helpers ----------------
        enum class Color : uint8_t { 
            White, 
            Grey, 
            Black 
        };

        RobotType buildTopology() {
            std::array<id_t, nL> head{};
            std::array<id_t, nJ> next{};
            std::array<id_t, nL> childCount{};
            std::array<uint8_t, nL> inDegree{};

            head.fill(invalidID_);
            next.fill(invalidID_);
            childCount.fill(0);
            inDegree.fill(0);

            for(id_t j = 0; j < nJ; j++) {
                const id_t p = joints_[j].getParentId();
                const id_t c = joints_[j].getChildId();

                // TODO
                // assert p < nL
                // assert c < nL
                // assert p != c

                next[j] = head[p];
                head[p] = j;
                childCount[p]++;
                inDegree[c]++;
            }


            id_t rootCount = 0;
            for(id_t i = 0; i < nL; i++) {
                if((inDegree[i] == 0) && (childCount[i] > 0)) { rootCount++; }
            }


            std::array<Color, nL> visit{};
            visit.fill(Color::White);

            std::array<id_t, nL> stack{};
            bool isCyclic = false;

            for(id_t root = 0; (root < nL) && (!isCyclic); root++) {
                if(visit[root] != Color::White) { continue; }

                id_t size = 0;
                stack[size++] = root;

                while((size > 0) && !isCyclic) {
                    const id_t node = stack[--size];

                    if(visit[node] == Color::Grey) {
                        visit[node] = Color::Black;
                        continue;
                    }

                    if(visit[node] != Color::White) { continue; }

                    visit[node] = Color::Grey;
                    stack[size++] = node;

                    for(id_t j = head[node]; j != invalidID_; j = next[j]) {
                        const id_t child = joints_[j].getChildId();

                        if(visit[child] == Color::White) { stack[size++] = child; }
                        else if (visit[child] == Color::Grey) { 
                            isCyclic = true;
                            break;
                        }
                    }
                }
            }


            if(!isCyclic) {
                for(id_t i = 0; i < nL; i++) {
                    // TODO
                    // assert visit[i] != white
                }
            }


            if(isCyclic) { return RobotType::Parallel; }
            if(rootCount != 1) { return RobotType::Invalid; }
            
            for(id_t i = 0; i < nL; i++) {
                if(childCount[i] > 1) { return RobotType::Tree; }
            }

            return RobotType::Serial;
        }

        void computeKinematicPaths() {
            std::array<id_t, nL> linkToJoint{};
            linkToJoint.fill(invalidID_);

            for(id_t j = 0; j < nJ; j++) {
                linkToJoint[joints_[j].getChildId()] = j;
            }

            for(id_t i = 0; i < nL; i++) {
                KinematicPath<nJ> &path = linkPaths_[i];
                path.length = 0;

                id_t curr = i;
                while(curr != invalidID_) {
                    const id_t j = linkToJoint[curr];
                    if(j == invalidID_) { break; }
                    path.joints[path.length++] = j;
                    curr = joints_[j].getParentId();
                }

                for(id_t k = 0; k < path.length/2; k++) {
                    std::swap(path.joints[k], path.joints[path.length - 1 - k]);
                }
            }

            for(id_t i = 0; i < nE; i++) {
                framePaths_[i] = linkPaths_[frames_[i].getLinkId()];
            }
        }
           

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a robot model
         *  @param name Name of the robot
         *  @param links Array of links in the robot
         *  @param joints Array of joints in the robot
         *  @param frames Array of tool/frame attachments in the robot
         *  @note Validates the robot model topology upon construction
         *  @throws AssertionError if the robot model topology is invalid
         */
        explicit RobotModel(const char *name = "",
                            const std::array<Link, nL> &links = {},
                            const std::array<Joint, nJ> &joints = {},
                            const std::array<FrameAttachment, nE> &frames = {})
            : name_(name), links_(links), joints_(joints), frames_(frames) {
                type_ = buildTopology();
                // TODO assert: type != invalid

                computeKinematicPaths();
            }

        // ---------------- Getters ----------------
        /**
         *  @brief Get the robot type
         *  @return RobotType enum indicating the type of robot
         */
        constexpr RobotType getType() const { return type_; }
        /**
         *  @brief Get the number of defined links in the robot model
         *  @return Number of links
         */
        constexpr id_t getLinkNum() const { return nL; }
       /**
         *  @brief Get the number of defined joints in the robot model
         *  @return Number of joints
         */
        constexpr id_t getJointNum() const { return nJ; }
        /**
         *  @brief Get the number of defined frames in the robot model
         *  @return Number of frames/tools
         */
        constexpr id_t getFrameNum() const { return nE; }
        
        
        /**
         *  @brief Get the name of the robot
         *  @return Name of the robot
         */
        const char *getName() const { return name_; }
        /**
         *  @brief Get the links of the robot
         *  @return Array of links in the robot
         */
        const std::array<Link, nL> &getLinks() const { return links_; }
        /**
         *  @brief Get the joints of the robot
         *  @return Array of joints in the robot
         */
        const std::array<Joint, nJ> &getJoints() const { return joints_; }
        /**
         *  @brief Get the frames/tools of the robot
         *  @return Array of frames/tools in the robot
         */
        const std::array<FrameAttachment, nE> &getFrames() const { return frames_; }
        /**
         *  @brief Get the kinematic paths from base to each links
         *  @return Array of KinematicPath to each link
         */
        const std::array<KinematicPath<nJ>, nL> &getLinkPaths() const { return linkPaths_; }
        /**
         *  @brief Get the kinematic paths from base to each frame
         *  @return Array of KinematicPath to each frame
         */
        const std::array<KinematicPath<nJ>, nE> &getFramePaths() const { return framePaths_; }
};

}  // cobalt::kinematics