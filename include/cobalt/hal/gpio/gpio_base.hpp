#pragma once

#include <stdint.h>

namespace cobalt::hal {

enum class GPIOMode {
    Input,
    Output,
    InputPullUp,
};

enum class GPIOLevel : bool {
    High = true,
    Low = false,
};

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