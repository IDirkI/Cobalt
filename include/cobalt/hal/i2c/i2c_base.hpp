#pragma once

#include <stdint.h>

namespace cobalt::hal {

enum class I2CMode {
    Master,
    Slave
};

// --------------------------------------
//      Base I2C Implementation    
// --------------------------------------

class I2CBase {
    protected:
    public:
        virtual ~I2CBase() = default;

        virtual bool init(uint8_t address, uint8_t port, I2CMode mode, uint32_t frequency) = 0;
        virtual bool write(uint8_t address, uint8_t *wBuff, size_t len) = 0;
        virtual bool read(uint8_t address, uint8_t *rBuff, size_t len) = 0;
        virtual bool request(uint8_t address, uint8_t *wBuff, uint8_t *rBuff, size_t readLen) = 0;
};

} // cobalt::hal