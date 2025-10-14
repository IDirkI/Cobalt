#define _USE_MATH_DEFINES

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/util/meta/cobalt_info.hpp>
#include <cobalt/util/meta/build_info.hpp>
#include <cobalt/util/meta/platform_info.hpp>

using cobalt::util::meta::CobaltInfo;
using cobalt::util::meta::BuildInfo;
using cobalt::util::meta::PlatformInfo;

TEST_CASE("Meta, cobalt info", "[utility]") {
    CAPTURE(CobaltInfo::version());

    REQUIRE(false);
}   

TEST_CASE("Meta, build info", "[utility]") {
    CAPTURE(BuildInfo::compiler());
    CAPTURE(BuildInfo::COMPILE_DATE);
    CAPTURE(BuildInfo::COMPILE_TIME);

    REQUIRE(false);
}   

TEST_CASE("Meta, platform info", "[utility]") {
    CAPTURE(PlatformInfo::os());
    CAPTURE(PlatformInfo::architecture());

    CAPTURE(PlatformInfo::framework());
    CAPTURE(PlatformInfo::framework_version());

    REQUIRE(false);
}   

