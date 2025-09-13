#pragma once

#include "gpio/gpio_base.hpp"       // Base GPIO virtual class
#include "gpio/gpio_arduino.hpp"    // -- Arudino GPIO implementation class


namespace cobalt::hal {
    #if ARDUINO
        using GPIO = GPIOArduino;
    #endif
}