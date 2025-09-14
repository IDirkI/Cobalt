#pragma once

#include <array>

#include "gpio_base.hpp"

#ifdef IDF_VER
#include <driver/gpio.h>
#include <driver/ledc.h>

namespace cobalt::hal {

constexpr float GPIO_ESPIDF_DEFAULT_FREQUENCY = 50.0f;

struct TimerAllocation {
    bool isUsed;
    float frequency;
};

// --------------------------------------
//      ESPIDF GPIO Implementation    
// --------------------------------------

class GPIOESPIDF : public GPIOBase {
    private:
        gpio_num_t pin_;
        float frequency_;

        ledc_channel_t channel_;
        ledc_timer_t timer_;


        static inline std::array<TimerAllocation, LEDC_TIMER_MAX> timerAllocationCounts_{};
        static inline std::array<uint8_t, LEDC_CHANNEL_MAX> channelAllocationCounts_{};

        // ---------------- ESPIDF GPIO Private Helper Functions ----------------
        inline ledc_timer_t allocateTimer();
        inline ledc_channel_t allocateChannel();

    public: 
        // ---------------- Constructors ----------------

        explicit GPIOESPIDF(gpio_num_t pin, float frequency = GPIO_ESPIDF_DEFAULT_FREQUENCY);


        // ---------------- ESPIDF GPIO Getters/Setters ----------------

        uint8_t getChannelNum();
        uint8_t getTimerNum();


        // ---------------- ESPIDF GPIO Member Functions ----------------

        void setMode(GPIOMode mode) override;
        void write(GPIOLevel level) override;
        void pwm(float duty) override;
        [[nodiscard]] GPIOLevel read() const override;
    };
} // cobalt::hal

#endif