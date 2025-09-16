#include "cobalt/hal/i2c/i2c_arduino.hpp"


#if defined(ARDUINO)

/**
* @brief Initialize to start communication with the I2C Interface
* @param address I2C Address of the device
* @param bus I2C Bus to access. 0 for Arduino UNO boards and 0 or 1 for ESP boards.
* @param mod I2C mode to operate in. Master or Slave
* @param frequency I2C frequency to communicate
* @return `true` if the initialization was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::init(uint8_t address, uint8_t bus = I2C_ARDUINO_DEFAULT_BUS, cobalt::hal::I2CMode mode = I2C_ARDUINO_DEFAULT_MODE, uint32_t frequency = I2C_ARDUINO_DEFAULT_FREQUENCY) {
    #if defined(ESP32)
        switch(bus) {
            case(0): {
                wire_ = &Wire;              // bus-0
                break;
            }
            case(1): {
                static TwoWire Wire1(1);    // bus-1
                wire_ = &Wire1;
                break;
            }
            default:
                return false;               // Invalid Bus
        }

        wire_->begin(pinSDA_, pinSCL_, frequency);

    #elif defined(ARDUINO_AVR_UNO)
        if(bus != 0) { return 0;}

        wire_ = &Wire;
        wire_->begin();

    #endif

    wire_->setClock(frequency);
    return true;
}

/**
* @brief Write the buffer data to the I2C device
* @param address I2C Address of the device
* @param wBuff Data buffer to write from
* @param len Length of `wBuff` to write. (defaults to 1 byte)
* @return `true` if the write was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::write(uint8_t address, const uint8_t *wBuff, size_t len = 1) {
    wire_->beginTransmission(address);
    size_t writtenBytes = wire_->write(wBuff, len);
    uint8_t result = wire_->endTransmission();

    return ((result == 0) && (writtenBytes == len));
}

/**
* @brief Read the I2C data to the buffer
* @param address I2C Address of the device
* @param rBuff Data buffer to read into
* @param len Length of data to read into `rBuff`. (defaults to 1 byte)
* @return `true` if the read was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::read(uint8_t address, uint8_t *rBuff, size_t len = 1) {
    size_t readBytes = wire_->requestFrom(address, len);
    if(readBytes != len) { return false; }

    for(size_t i = 0; i < len; i++) {
        if(wire_->available()) { rBuff[i] = wire_->read(); }
        else { return false; } 
    }

    return true;
}

/**
* @brief Write to and then immediately read from the I2C device
* @param address I2C Address of the device
* @param wBuff Data buffer to read into
* @param writeLen Length of `wBuff` to write.
* @param rBuff Data buffer to read into
* @param readLen Length of data to read into `rBuff`.
* @return `true` if the write-read was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::writeRead(uint8_t address, const uint8_t *wBuff, size_t writeLen, uint8_t *rBuff, size_t readLen) {
    if(!write(address, wBuff, writeLen)) { return false; }
    return read(address, rBuff, readLen);
}

/**
* @brief Write 1 byte to the I2C device's register from the buffer
* @param address I2C Address of the device
* @param regAddr I2C Device's register to read
* @param wBuff Data buffer to write from
* @return `true` if the register write was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::writeReg(uint8_t address, uint8_t regAddr, uint8_t wBuff) {
    uint8_t buff[2] = { regAddr, wBuff };
    return write(address, buff, 2);
}

/**
* @brief Read 1 byte from the I2C device's register to the buffer
* @param address I2C Address of the device
* @param regAddr I2C Device's register to read
* @param rBuff Data buffer to read into
* @return `true` if the register read was successful, `false` otherwise
*/
bool cobalt::hal::I2CArduino::readReg(uint8_t address, uint8_t regAddr, uint8_t *rBuff) {
    uint8_t buff[1] = { regAddr };
    if(!write(address, buff)) { return false; }
    return read(address, rBuff);
}

#endif