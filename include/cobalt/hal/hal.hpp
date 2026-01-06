#pragma once

#include "gpio/gpio_base.hpp"       // Base GPIO virtual class
#include "gpio/gpio_arduino.hpp"    // -- Arudino GPIO implementation class
#include "gpio/gpio_espidf.hpp"     // -- ESPIDF GPIO implementation class

#include "i2c/i2c_base.hpp"       // Base I2C virtual class
#include "i2c/i2c_arduino.hpp"    // -- Arudino I2C implementation class
#include "i2c/i2c_espidf.hpp"     // -- ESPIDF I2C implementation class

#include "timer/timer.hpp"          // Base Timer implementation

namespace cobalt::hal {
    #if defined(ARDUINO_ARCH_ESP32)
        using GPIO = GPIOArduino;
        using I2C = I2CArduino;
    #elif defined(ESP_PLATFORM)
        using GPIO = GPIOESPIDF;
        using I2C = I2CESPIDF;
    #endif
}