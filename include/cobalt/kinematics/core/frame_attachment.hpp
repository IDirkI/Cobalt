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
        id_t idLink_{invalidID_};
        std::string name_{""};
        cobalt::math::geometry::Transform<> origin_{cobalt::math::geometry::Transform<>::eye()};

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a frame attachment
         *  @param idLink ID of the link the frame is attached to
         *  @param name Name of the frame
         *  @param origin Transform of the frame relative to the link frame
         */
        explicit FrameAttachment(id_t idLink = invalidID_,
                                 const std::string &name = "",
                                 const cobalt::math::geometry::Transform<> &origin = cobalt::math::geometry::Transform<>::eye())
            : name_(name), idLink_(idLink), origin_(origin) {}

        // ---------------- Getters ----------------
        constexpr id_t getLinkId() const { return idLink_; }
        
        const std::string getName() const { return name_; }
        const cobalt::math::geometry::Transform<> getOrigin() const { return origin_; }
};

} // cobalt::kinematics