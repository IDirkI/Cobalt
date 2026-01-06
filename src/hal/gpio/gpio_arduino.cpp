#include "cobalt/hal/gpio/gpio_arduino.hpp"

#if defined(ARDUINO_ARCH_ESP32)

/**
* @brief Set the GPIO pins mode/type
* @param mode  `Input`, `Output` or `InnputPullUp`
*/
void cobalt::hal::GPIOArduino::setMode(GPIOMode mode) {
    switch(mode) {
        case(GPIOMode::Input): { pinMode(pin_, INPUT); break; }
        case(GPIOMode::Output): { pinMode(pin_, OUTPUT); break; }
        case(GPIOMode::InputPullUp): { pinMode(pin_, INPUT_PULLUP); break; }
        case(GPIOMode::PWM): { pinMode(pin_, OUTPUT); break; }
        default: { break; }
    }
}

/**
* @brief Set the state of the GPIO pin
* @param level  `High` or `Low`
* @note Only available when the pin is in `Output` mode
*/
void cobalt::hal::GPIOArduino::write(GPIOLevel level) {
    digitalWrite(pin_, ((level == GPIOLevel::High) ?HIGH :LOW));
}

/**
* @brief Set the GPIO pin to output a pwm with a +duty cycle of `duty` at 1kHz
* @param duty +Duty-cycle of the PWM signal, [0.0, 1.0]
* @note Only works if the hardwares pin support it
*/
void cobalt::hal::GPIOArduino::pwm(float duty) {
    uint8_t pwmVal = duty*255;
    analogWrite(pin_, pwmVal);
}

/**
* @brief Read the state of the GPIO pin
* @return `High` or `Low`
*/
[[nodiscard]] cobalt::hal::GPIOLevel cobalt::hal::GPIOArduino::read() const {
    return ((digitalRead(pin_) == HIGH) ?GPIOLevel::High :GPIOLevel::Low);
}


#endif