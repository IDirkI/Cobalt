#pragma once

#include <stdint.h>

#include "i2c_base.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <Wire.h>

namespace cobalt::hal {

constexpr uint8_t I2C_ARDUINO_DEFAULT_SDA = 4; 
constexpr uint8_t I2C_ARDUINO_DEFAULT_SCL = 5; 

constexpr uint8_t I2C_ARDUINO_DEFAULT_BUS = 0; 
constexpr I2CMode I2C_ARDUINO_DEFAULT_MODE = I2CMode::Master;
constexpr uint32_t I2C_ARDUINO_DEFAULT_FREQUENCY = 100000;


// --------------------------------------
//      Arduino I2C Implementation    
// --------------------------------------

class I2CArduino : public I2CBase {
    private:
        uint8_t pinSDA_;
        uint8_t pinSCL_;
        TwoWire *wire_ = nullptr;

    public:
        // ---------------- Constructors ----------------
        /**
         * @brief Constructor for Arduino I2C implementation layer
         * @param sdaPin SDA pin number
         * @param sclPin SCL pin number
         * 
         * @note Leave pin definitions empty if on an Arduino UNO board.
         */
        explicit I2CArduino(uint8_t sdaPin = I2C_ARDUINO_DEFAULT_SDA, uint8_t sclPin = I2C_ARDUINO_DEFAULT_SCL) : pinSDA_(sdaPin), pinSCL_(sclPin) {}

        // ----------------  Arduino I2C Member Functions ----------------

        bool init(uint8_t bus = I2C_ARDUINO_DEFAULT_BUS, I2CMode mode = I2C_ARDUINO_DEFAULT_MODE, uint32_t frequency = I2C_ARDUINO_DEFAULT_FREQUENCY) override;
        bool write(uint8_t address, const uint8_t *wBuff, size_t len = 1) override;
        bool read(uint8_t address, uint8_t *rBuff, size_t len = 1) override;
        bool writeRead(uint8_t address, const uint8_t *wBuff, size_t writeLen, uint8_t *rBuff, size_t readLen) override;
        bool writeReg(uint8_t address, uint8_t regAddr, uint8_t wBuff) override;
        bool readReg(uint8_t address, uint8_t regAddr, uint8_t *rBuff) override;
};

} // cobalt::hal

#endif