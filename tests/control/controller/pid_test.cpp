#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fstream>
#include <cmath>

#include <cobalt/control/controller/pid.hpp>

using cobalt::control::PID;

TEST_CASE("PID, default construction & gain set", "[pid]") {
    PID pid(10.0f, 2.0f, 0.5, 150.0f, 100.0f, -5.0f, 5.0f);
    pid.setName("Test Controller");

    REQUIRE(pid.getKp() == Catch::Approx(10.0f));
    REQUIRE(pid.getKi() == Catch::Approx(2.0f));
    REQUIRE(pid.getKd() == Catch::Approx(0.5f));
    REQUIRE(pid.getTau() == Catch::Approx(200.0f));
    REQUIRE(pid.getSamplePeriod() == Catch::Approx(100.0f));
    REQUIRE(pid.getOutMin() == Catch::Approx(-5.0f));
    REQUIRE(pid.getOutMax() == Catch::Approx(5.0f));
    REQUIRE(pid.getName() == "Test Controller");

    REQUIRE(pid.setKp(20.0f));
    REQUIRE(!pid.setKi(-8.0f));
    REQUIRE(pid.setKd(1.0f));

    REQUIRE(pid.getKp() == Catch::Approx(20.0f));
    REQUIRE(pid.getKi() == Catch::Approx(2.0f));
    REQUIRE(pid.getKd() == Catch::Approx(1.0f));

    REQUIRE(pid.setKi(2.3f));
    REQUIRE(pid.getKi() == Catch::Approx(2.3f));
} 

TEST_CASE("PID, Windup", "[pid]") {
    std::ofstream log;
    log.open("../../results/control/wind_up.csv");

    const double Ts = 0.001;

    const double simTime = 20; // [s]
    const double stepTime = 1; // [s]

    PID pid(400, 100, 0, Ts*3, Ts, -1000, 1000);

    log << "time,P,I,D,u\n"; 

    pid.setRef(0.0f);
    pid.reset(0.0f);

    // Closed-loop simulation
    for (int step = 0; step < simTime/Ts; step++) {
        if(step == stepTime/Ts) { pid.setRef(1.0f); }

        double u0 = pid.update(0.0);

        log << Ts*step << "," << pid.getPPath() << "," << pid.getIPath() << "," << pid.getDPath() << "," << u0 << "\n";
    }

    log.close();
    
    REQUIRE(pid.getIPath() + pid.getPPath() <= Catch::Approx(1000).margin(1e-6));
} 

TEST_CASE("PID, 2nd Order System", "[pid]") {
    std::ofstream log;
    log.open("../../results/control/pid_test.csv");

    // continuous params
    const double m = 1.0;     // mass
    const double b = 0.5;     // damping
    const double k = 2.0;     // spring constant
    const double Ts = 0.001;  // sample period [s]

    const double simTime = 20; // [s]
    const double stepTime = 1; // [s]

    PID pid(10.4, 6.5, 15.7, Ts*3, Ts, -1000, 1000);

    // Tustin constant
    const double A = 2.0 / Ts;

    // Denominator coefficients after clearing (1 + z^-1)^2
    const double D0 = m*A*A + b*A + k;
    const double D1 = -2.0*m*A*A + 2.0*k;
    const double D2 = m*A*A - b*A + k;

    // Numerator (from (1 + 2 z^-1 + z^-2) * U(z))
    // So difference equation:
    // D0*y[k] + D1*y[k-1] + D2*y[k-2] = 1*u[k] + 2*u[k-1] + 1*u[k-2]

    // Normalized difference-equation coefficients (optional)
    const double a1 = D1/D0;
    const double a2 = D2/D0;
    const double b0 = 1/D0;
    const double b1 = 2/D0;
    const double b2 = 1/D0;

    // state (delays)
    double y  = 0.0;   // current output (y[k])
    double y1 = 0.0;   // y[k-1]
    double y2 = 0.0;   // y[k-2]
    double u1 = 0.0;   // u[k-1]
    double u2 = 0.0;   // u[k-2]

    log << "time,ref,y,P,I,D,u\n"; 

    pid.setRef(0.0f);
    pid.reset(0.0f);

    // Closed-loop simulation
    for (int step = 0; step < simTime/Ts; step++) {
        if(step == stepTime/Ts) { pid.setRef(1.0f); }

        double u0 = pid.update(y);

        // difference equation (Tustin-discretized, causal)
        double y0 = (-a1)*y1 + (-a2)*y2 + b0*u0 + b1*u1 + b2*u2;

        // shift states
        y2 = y1;
        y1 = y0;
        u2 = u1;
        u1 = u0;

        y = y0;

        log << Ts*step << "," << pid.getRef() << "," << y << "," << pid.getPPath() << "," << pid.getIPath() << "," << pid.getDPath() << "," << u0 << "\n";
    }

    log.close();

    REQUIRE(y == Catch::Approx(pid.getRef()).margin(1e-6));
} 