#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/control/controller/pid.hpp>

using cobalt::control::PID;

TEST_CASE("PID, default construction & gain set", "[pid]") {
    PID pid;

    REQUIRE(pid.setKp(10.0f));
    REQUIRE(!pid.setKi(-6.0f));
    REQUIRE(pid.setKd(0.5f));

    REQUIRE(pid.getKp() == Catch::Approx(10.0f));
    REQUIRE(pid.getKi() == Catch::Approx(0.0f));
    REQUIRE(pid.getKd() == Catch::Approx(0.5f));

    REQUIRE(pid.setKi(2.3f));
    REQUIRE(pid.getKi() == Catch::Approx(2.3f));
} 