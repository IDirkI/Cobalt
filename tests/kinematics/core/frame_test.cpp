#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "cobalt/kinematics/core/frame_attachment.hpp"

using cobalt::kinematics::FrameAttachment;

TEST_CASE("Frame Attachment, default construction", "[kinematics]") {
    FrameAttachment frame = FrameAttachment();
    
    REQUIRE(true);
} 
