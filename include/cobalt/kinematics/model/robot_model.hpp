#pragma once

#include <string>
#include <array>
#include <cassert>

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

        bool contains(id_t jointId) {
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
        std::string name_{""};
        RobotType type_{RobotType::Invalid};

        std::array<Link, nL> links_{};
        std::array<Joint, nJ> joints_{};
        std::array<FrameAttachment, nE> frames_{};

        std::array<KinematicPath<nJ>, nL> linkPaths_{};
        std::array<KinematicPath<nJ>, nE> framePaths_{};

        // ---------------- Private Helpers ----------------
        enum class VisitColor : uint8_t { 
            White, 
            Grey, 
            Black 
        };

        struct Adjacency {
            std::array<id_t, nL> head{};
            std::array<id_t, nJ> next{};
            std::array<id_t, nJ> child{};
            std::array<id_t, nL> childCount{};
            id_t edgeCount{0};

            void init() {
                head.fill(invalidID_);
                next.fill(invalidID_);
                child.fill(invalidID_);
                childCount.fill(0);
                edgeCount = 0;
            }

            void addEdge(id_t p, id_t c) {
                id_t edge = edgeCount++;
                child[edge] = c;
                next[edge] = head[p];
                head[p] = edge;
                childCount[p]++;
            }
        };

        void validate(std::array<VisitColor, nL> &color) {
            // Check ID limits
            for(const Joint &j: joints_) {
                assert((j.getParentId() < nL) && "[ROBOT MODEL Error] : Joint has invalid parent link ID.");
                assert((j.getChildId() < nL) && "[ROBOT MODEL Error] : Joint has invalid child link ID.");
            }

            // Check Disconnection
            for(id_t i = 0; i < nL; i++) {
                assert((color[i] != VisitColor::White) && "[ROBOT MODEL Error] : Robot model is disconnected.");
            }
        }

        RobotType decideType(Adjacency &adj, std::array<VisitColor, nL> &color, id_t &rootCount, bool &hasCycle) const {
            // Decide type
            if(hasCycle) { 
                return RobotType::Parallel;
            }

            if(rootCount != 1) {
                return RobotType::Invalid;
            }

            for(id_t i = 0; i < nL; i++) {
                if(adj.childCount[i] > 1) {
                    return RobotType::Tree;
                }
            }

            return RobotType::Serial;
        }

        void dfs(Adjacency &adj, std::array<VisitColor, nL> &color, id_t &rootCount, bool &hasCycle) {            
            adj.init();

            std::array<uint8_t, nL> inDegree{};
            inDegree.fill(0);

            for(const Joint &j: joints_) {
                adj.addEdge(j.getParentId(), j.getChildId());
                inDegree[j.getChildId()]++;
            }

            // Count roots
            rootCount = 0;
            for(id_t i = 0; i < nL; i++) {
                if((inDegree[i] == 0) && adj.childCount[i] > 0) rootCount++;
            }

            // Check cycles
            color.fill(VisitColor::White);

            hasCycle = false;
            for(id_t i = 0; i < nL; i++) {
                if(color[i] == VisitColor::White) { // DFS
                    std::array<id_t, nL> stack;
                    id_t size = 0;
                    stack[size++] = i;

                    while(size > 0) {
                        size--;
                        id_t node = stack[size];

                        if(color[node] == VisitColor::White) {
                            color[node] = VisitColor::Grey;

                            for(id_t edge = adj.head[node]; edge != invalidID_; edge = adj.next[edge]) {
                                id_t child = adj.child[edge];
                                if(color[child] == VisitColor::White) {
                                    stack[size++] = child;
                                }
                                else if(color[child] == VisitColor::Grey) {
                                    hasCycle = true;
                                    break;
                                }
                            }
                        }
                        else {
                            color[node] = VisitColor::Black;
                        }

                        if(hasCycle) break;
                    }
                
                }
            }
        }

        KinematicPath<nJ> generateLinkPath(id_t linkId) {
            KinematicPath<nJ> path;
            
            std::array<id_t, nL> linkToJoint{};
            linkToJoint.fill(invalidID_);

            for(id_t j = 0; j < nJ; j++) {
                const Joint &joint = joints_[j];
                linkToJoint[joint.getChildId()] = j;
            }

            id_t currLink = linkId;
            while(currLink != invalidID_) {
                id_t parentJointId = linkToJoint[currLink];

                if(parentJointId == invalidID_) {
                    break;
                }
                
                path.joints[path.length++] = parentJointId;
                currLink = joints_[parentJointId].getParentId();
            }

            for(id_t i = 0; i < path.length / 2; i++) {
                std::swap(path.joints[i], path.joints[path.length - 1 - i]);
            }

            return path;
        }

        void computeKinematicPaths() {
            for(id_t i = 0 ; i < nL; i++) { // For links
                linkPaths_[i] = generateLinkPath(i);
            }

            for(id_t i = 0 ; i < nE; i++) { // For frames
                const FrameAttachment &frame = frames_[i];
                framePaths_[i] = linkPaths_[frame.getLinkId()];
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
        explicit RobotModel(const std::string &name = "",
                            const std::array<Link, nL> &links = {},
                            const std::array<Joint, nJ> &joints = {},
                            const std::array<FrameAttachment, nE> &frames = {})
            : name_(name), links_(links), joints_(joints), frames_(frames) {
                Adjacency adj;
                std::array<VisitColor, nL> color;
                id_t rootCount = 0;
                bool hasCycle = false;

                dfs(adj, color, rootCount, hasCycle);

                validate(color);
                type_ = decideType(adj, color, rootCount, hasCycle);
                assert((type_ != RobotType::Invalid) && "[ROBOT MODEL Error] : Robot model topology is invalid.");

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
        const std::string &getName() const { return name_; }
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