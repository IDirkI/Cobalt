#pragma once

#include "gpio_base.hpp"

#if ARDUINO
#include <Arduino.h>

namespace cobalt::hal {

class GPIOArduino : public GPIOBase {
    private:
        uint8_t pin_;

    public: 
        /**
         * @brief Constructor for Arduino GPIO implementation layer
         */
        explicit GPIOArduino(int pin) :  pin_(pin) {}

        /**
        * @brief Set the GPIO pins mode/type
        * @param mode  `Input`, `Output` or `InnputPullUp`
        */
        void setMode(GPIOMode mode) override {
            switch(mode) {
                case(GPIOMode::Input): { pinMode(pin_, INPUT); break; }
                case(GPIOMode::Output): { pinMode(pin_, OUTPUT); break; }
                case(GPIOMode::InputPullUp): { pinMode(pin_, INPUT_PULLUP); break; }
                default: { break; }
            }
        }

        /**
        * @brief Set the state of the GPIO pin
        * @param level  `High` or `Low`
        * @note Only available when the pin is in `Output` mode
        */
        void write(GPIOLevel level) override {
            digitalWrite(pin_, ((level == GPIOLevel::High) ?HIGH :LOW));
        }

        /**
        * @brief Set the GPIO pin to output a pwm with a +duty cycle of `duty` at 1kHz
        * @param duty +Duty-cycle of the PWM signal, [0.0, 1.0]
        * @note Only works if the hardwares pin support it
        */
        void pwm(float duty) override {
            uint8_t pwmVal = duty*255;
            analogWrite(pin_, pwmVal);
        }

        /**
        * @brief Read the state of the GPIO pin
        * @return `High` or `Low`
        */
        [[nodiscard]] GPIOLevel read() const override {
            return ((digitalRead(pin_) == HIGH) ?GPIOLevel::High :GPIOLevel::Low);
        }
}; 

} // cobalt::hal
#endif