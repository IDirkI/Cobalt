#pragma once

#pragma once

#include "gpio_base.hpp"


#ifdef IDF_VER
#include <driver/gpio.h>

namespace cobalt::hal {

class GPIOESP32 : public GPIOBase {
    private:
        gpio_num_t pin_;

    public: 
        /**
         * @brief Constructor for Arduino GPIO implementation layer
         */
        explicit GPIOESP32(gpio_num_t pin) :  pin_(pin) {}

        /**
        * @brief Set the GPIO pins mode/type
        * @param mode  `Input`, `Output` or `InnputPullUp`
        */
        void setMode(GPIOMode mode) override {
            switch(mode) {
                case(GPIOMode::Input): { gpio_set_direction(pin_, GPIO_MODE_INPUT); break; }
                case(GPIOMode::Output): { gpio_set_direction(pin_, GPIO_MODE_OUTPUT); break; }
                case(GPIOMode::InputPullUp): { gpio_set_direction(pin_, GPIO_MODE_INPUT); break; }
                default: { gpio_set_direction(pin_, GPIO_MODE_DISABLE); break; }
            }
        }

        /**
        * @brief Set the state of the GPIO pin
        * @param level  `High` or `Low`
        * @note Only available when the pin is in `Output` mode
        */
        void write(GPIOLevel level) override {
            gpio_set_level(pin_, static_cast<uint8_t>(level));
        }

        /**
        * @brief Set the GPIO pin to output a pwm with a +duty cycle of `duty` at 1kHz
        * @param duty +Duty-cycle of the PWM signal, [0.0, 1.0]
        * @note Only works if the hardwares pin support it
        */
        void pwm(float duty) override {
            
        }

        /**
        * @brief Read the state of the GPIO pin
        * @return `High` or `Low`
        */
        [[nodiscard]] GPIOLevel read() const override {
            return ((gpio_get_level(pin_)) ?GPIOLevel::High :GPIOLevel::Low);
        }
}; 

} // cobalt::hal
#endif