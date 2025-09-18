#pragma once

#pragma once

#include <stdint.h>

#include "i2c_base.hpp"

#if defined(IDF_VER)
#include <driver/i2c.h>

namespace cobalt::hal {

constexpr uint8_t I2C_ESPIDF_DEFAULT_BUS = 0; 
constexpr I2CMode I2C_ESPIDF_DEFAULT_MODE = I2CMode::Master;
constexpr uint32_t I2C_ESPIDF_DEFAULT_FREQUENCY = 100000;


// --------------------------------------
//      ESP-IDF I2C Implementation    
// --------------------------------------

class I2CESPIDF : public I2CBase {
    private:
        uint8_t pinSDA_;
        uint8_t pinSCL_;
        i2c_port_t port_;
        bool isInitialized_;


    public:
        // ---------------- Constructors ----------------
        /**
         * @brief Constructor for ESP-IDF I2C implementation layer
         * @param sdaPin SDA pin number
         * @param sclPin SCL pin number
         */
        explicit I2CESPIDF(uint8_t sdaPin, uint8_t sclPin) : pinSDA_(sdaPin), pinSCL_(sclPin), port_(static_cast<i2c_port_t>(I2C_ESPIDF_DEFAULT_BUS)), isInitialized_(false) {}

        ~I2CESPIDF() { if(isInitialized_) { i2c_driver_delete(port_); }}

        // ----------------  ESP-IDF I2C Member Functions ----------------

        bool init(uint8_t bus = I2C_ESPIDF_DEFAULT_BUS, I2CMode mode = I2C_ESPIDF_DEFAULT_MODE, uint32_t frequency = I2C_ESPIDF_DEFAULT_FREQUENCY) override;
        bool write(uint8_t address, const uint8_t *wBuff, size_t len = 1) override;
        bool read(uint8_t address, uint8_t *rBuff, size_t len = 1) override;
        bool writeRead(uint8_t address, const uint8_t *wBuff, size_t writeLen, uint8_t *rBuff, size_t readLen) override;
        bool writeReg(uint8_t address, uint8_t regAddr, uint8_t wBuff) override;
        bool readReg(uint8_t address, uint8_t regAddr, uint8_t *rBuff) override;
};

} // cobalt::hal

#endif