#pragma once

#include <string>
#include <stdint.h>

namespace cobalt::control {

constexpr float PID_DEFAULT_KP = 1.0f;
constexpr float PID_DEFAULT_KI = 0.0f;
constexpr float PID_DEFAULT_KD = 0.0f;

constexpr float PID_DEFAULT_LPF_CUTOFF = 100.0f;
constexpr float PID_DEFAULT_TS = 1.0f;

constexpr float PID_DEFAULT_OUT_MIN = -1.0f;
constexpr float PID_DEFAULT_OUT_MAX = 1.0f;


struct PIDState {
    /* State/Memory */
    float pathP;        // p[n]

    float pathI;        // i[n]
    float prevErr;      // err[n-1]

    float pathD;        // d[n]
    float prevMeasure;  // y[n-1]
};


class PID {
    private:
        // ----- Name -----
        std::string name_;

        // ----- Controller Gains -----
        float Kp_;
        float Ki_;
        float Kd_;

        // ----- State/Memory -----
        PIDState state_;

        // ----- Z-Domain Design -----
        float tau_; // LPF 3dB cutoff frequency   , [ rad/s ]
        float Ts_;  // DT controller sample period, [ s ]

        // ----- Limits -----
        float outMin_;
        float outMax_;

        // ----- Input/Output -----
        float ref_;     // r[n]
        float measure_; // y[n]
        float output_;  // u[n]

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief PID Controller Constrcutor.
         *  
         *  @param Kp Proportional gain.
         *  @param Ki Integral gain.
         *  @param Kd Differential gain.
         *  @param tau 3dB cutoff frequency for the LPF the differentiator path.
         *  @param Ts Discrete time sampling period.
         *  @param outMin Minimum value PID controller can output.
         *  @param outMax Maximum value PID controller can output.
         * 
         */
        PID(float Kp = PID_DEFAULT_KP, float Ki = PID_DEFAULT_KI, float Kd = PID_DEFAULT_KD, float tau = PID_DEFAULT_LPF_CUTOFF, float Ts = PID_DEFAULT_TS,
                      float outMin = PID_DEFAULT_OUT_MIN, float outMax = PID_DEFAULT_OUT_MAX) 
                    : Kp_(Kp), Ki_(Ki), Kd_(Kd), tau_(tau), Ts_(Ts), outMin_(outMin), outMax_(outMax), name_("") {}

        // ---------------- Getters ----------------
        float getKp() const;
        float getKi() const;
        float getKd() const;
        float getCutoff() const;
        float getSamplePeriod() const;
        float getOutMin() const;
        float getOutMax() const;
        
        float getRef() const; 
        float getMeasure() const;
        float getError() const;
        float getOutput() const;
        std::string getName() const;

        // ---------------- Setters ----------------
        bool setKp(float Kp);
        bool setKi(float Ki);
        bool setKd(float Kd);
        bool setGains(float Kp, float Ki, float Kd);
        bool setCutoff(float tau);
        bool setLimits(float min, float max);
        
        bool setRef(float ref);
        bool setName(const std::string &name);

        // ---------------- Functionality  ----------------
        bool reset();
        float update(float measurement);
};

} // cobalt::control