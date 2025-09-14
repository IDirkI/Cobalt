#pragma once

#include "gpio/gpio_base.hpp"       // Base GPIO virtual class
#include "gpio/gpio_arduino.hpp"    // -- Arudino GPIO implementation class
#include "gpio/gpio_espidf.hpp"     // -- ESPIDF GPIO implementation class


namespace cobalt::hal {
    #ifdef ARDUINO
        using GPIO = GPIOArduino;
    #elifdef IDF_VER
        using GPIO = GPIOESPIDF;
    #endif
}