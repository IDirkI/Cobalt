#pragma once

#include <stddef.h>
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

        virtual bool init(uint8_t bus, I2CMode mode, uint32_t frequency) = 0;
        virtual bool write(uint8_t address, const uint8_t *wBuff, size_t len) = 0;
        virtual bool read(uint8_t address, uint8_t *rBuff, size_t len) = 0;
        virtual bool writeRead(uint8_t address, const uint8_t *wBuff, size_t writeLen, uint8_t *rBuff, size_t readLen) = 0;
        virtual bool writeReg(uint8_t address, uint8_t regAddr, uint8_t wBuff) = 0;
        virtual bool readReg(uint8_t address, uint8_t regAddr, uint8_t *rBuff) = 0;
};

} // cobalt::hal