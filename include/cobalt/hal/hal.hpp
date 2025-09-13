#pragma once

#include "gpio/gpio_base.hpp"       // Base GPIO virtual class
#include "gpio/gpio_arduino.hpp"    // -- Arudino GPIO implementation class


namespace cobalt::hal {
    #if defined ARDUINO
        using GPIO = GPIOArduino;
    #elif defined IDF_VER
        using GPIO = GPIOESP32;
    #endif
}