#pragma once

#include <stdint.h>

namespace cobalt::hal {

enum class GPIOMode {
    Input,
    Output,
    InputPullUp,
    PWM,
};

enum class GPIOLevel : uint8_t {
    High = 1,
    Low = 0,
};

// --------------------------------------
//      Base GPIO Implementation    
// --------------------------------------

class GPIOBase {
    protected:

    public:
        virtual ~GPIOBase() = default;

        virtual void setMode(GPIOMode mode) = 0;

        virtual void write(GPIOLevel level) = 0;
        virtual void pwm(float duty) = 0;
        [[nodiscard]] virtual GPIOLevel read() const = 0;
};

} // cobalt::hal