#pragma once

#include <string>

#include "cobalt/kinematics/config.hpp"

#include "cobalt/math/geometry/transform/transform.hpp"

namespace cobalt::kinematics {

// --------------------------------------
//         Frame/Tool Attachment    
// --------------------------------------
/**
 *  @brief Tool/Frame attached to a link
 */
struct FrameAttachment {
    private:    
        id_t id_{invalidID_};
        id_t idLink_{invalidID_};
        std::string name_{""};
        cobalt::math::geometry::Transform<> origin_{cobalt::math::geometry::Transform<>::eye()};

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a frame attachment
         *  @param id ID of the frame/attachment
         *  @param idLink ID of the link the frame is attached to
         *  @param name Name of the frame
         *  @param origin Transform of the frame relative to the link frame
         */
        explicit FrameAttachment(id_t id = invalidID_,
                                 id_t idLink = invalidID_,
                                 const std::string &name = "",
                                 const cobalt::math::geometry::Transform<> &origin = cobalt::math::geometry::Transform<>::eye())
            : id_(id), idLink_(idLink), name_(name), origin_(origin) {}

        // ---------------- Getters ----------------
        /**
         *  @brief Get the ID of the frame
         *  @return ID of the frame
         */
        constexpr id_t getId() const { return id_; }
        /**
         *  @brief Get the ID of the link the frame is attached to
         *  @return ID of the link the frame is attached to
         */
        constexpr id_t getLinkId() const { return idLink_; }
        
        /**
         *  @brief Get the name of the frame
         *  @return Name of the frame
         */
        const std::string getName() const { return name_; }

        /**
         *  @brief Get the origin transform of the frame
         *  @return Transform of the frame relative to its parent link's frame
         */
        const cobalt::math::geometry::Transform<> getOrigin() const { return origin_; }
};

} // cobalt::kinematics