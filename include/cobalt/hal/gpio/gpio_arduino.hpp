#pragma once

#include "gpio_base.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>

namespace cobalt::hal {

// --------------------------------------
//      Arduino GPIO Implementation    
// --------------------------------------

class GPIOArduino : public GPIOBase {
    private:
        uint8_t pin_;

    public: 
        // ---------------- Constructors ----------------
        /**
         * @brief Constructor for Arduino GPIO implementation layer
         */
        explicit GPIOArduino(int pin) :  pin_(pin) {}

        // ---------------- Arduino GPIO Member Functions ----------------

        void setMode(GPIOMode mode) override;
        void write(GPIOLevel level) override;
        void pwm(float duty) override;
        [[nodiscard]] GPIOLevel read() const override;
}; 

} // cobalt::hal
#endif