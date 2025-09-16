#pragma once

#include <stdint.h>

#include "i2c_base.hpp"

namespace cobalt::hal {

constexpr uint8_t I2C_ARDUINO_DEFAULT_PORT = 0; 
constexpr I2CMode I2C_ARDUINO_DEFAULT_MODE = I2CMode::Master;
constexpr uint32_t I2C_ARDUINO_DEFAULT_FREQUENCY = 100000;


// --------------------------------------
//      Arduino I2C Implementation    
// --------------------------------------

class I2CArduino : public I2CBase {
    private:
        uint8_t pinSDA_;
        uint8_t pinSCL_;

    public:
        // ---------------- Constructors ----------------
        /**
         * @brief Constructor for Arduino I2C implementation layer
         * @param sdaPin SDA pin number of the arduino board
         * @param sclPin SCL
         */
        explicit I2CArduino(uint8_t sdaPin, uint8_t sclPin) : pinSDA_(sdaPin), pinSCL_(sclPin) {}

        bool init(uint8_t address, uint8_t port, I2CMode mode, uint32_t frequency) override;
        bool write(uint8_t address, uint8_t *wBuff, size_t len) override;
        bool read(uint8_t address, uint8_t *rBuff, size_t len) override;
        bool request(uint8_t address, uint8_t *wBuff, uint8_t *rBuff, size_t readLen) override;
};

} // cobalt::hal