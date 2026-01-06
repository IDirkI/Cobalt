#include "cobalt/hal/i2c/i2c_espidf.hpp"

#if (defined(IDF_VER) && !defined(ARDUINO_ARCH_ESP32))

/**
* @brief Initialize to start communication with the I2C Interface
* @param bus I2C Bus to access.
* @param mod I2C mode to operate in. Master or Slave
* @param frequency I2C frequency to communicate
* @return `true` if the initialization was successful, `false` otherwise
*/
bool cobalt::hal::I2CESPIDF::init(uint8_t bus, cobalt::hal::I2CMode mode, uint32_t frequency) {
    switch(bus) {
        case(0) : { port_ = I2C_NUM_0; break; }
        case(1) : { port_ = I2C_NUM_1; break; }
        default : { return false; }
    }

    i2c_config_t conf{
        .mode = (mode == cobalt::hal::I2CMode::Master) ?I2C_MODE_MASTER :I2C_MODE_SLAVE,
        .sda_io_num = pinSDA_,
        .scl_io_num = pinSCL_,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    if(mode == cobalt::hal::I2CMode::Master) {
        conf.master.clk_speed = frequency;
    }

    i2c_param_config(port_, &conf);

    i2c_driver_install(port_, ((mode == cobalt::hal::I2CMode::Master) ?I2C_MODE_MASTER :I2C_MODE_SLAVE), 0, 0, 0);
    isInitialized_ = true;

    return true;
}

/**
* @brief Write the buffer data to the I2C device
* @param address I2C Address of the device
* @param wBuff Data buffer to write from
* @param len Length of `wBuff` to write. (defaults to 1 byte)
* @return `true` if the write was successful, `false` otherwise
*/
bool cobalt::hal::I2CESPIDF::write(uint8_t address, const uint8_t *wBuff, size_t len) {
    if(!isInitialized_) { return false; }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

    if(wBuff && len > 0) {
        i2c_master_write(cmd, wBuff, len, true);
    }

    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (result == ESP_OK);
}

/**
* @brief Read the I2C data to the buffer
* @param address I2C Address of the device
* @param rBuff Data buffer to read into
* @param len Length of data to read into `rBuff`. (defaults to 1 byte)
* @return `true` if the read was successful, `false` otherwise
*/
bool cobalt::hal::I2CESPIDF::read(uint8_t address, uint8_t *rBuff, size_t len) {
    if(!isInitialized_ || !rBuff) { return false; }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);

    if(len > 1) {
        i2c_master_read(cmd, rBuff, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rBuff + len-1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (result == ESP_OK);
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
bool cobalt::hal::I2CESPIDF::writeRead(uint8_t address, const uint8_t *wBuff, size_t writeLen, uint8_t *rBuff, size_t readLen) {
    if(!isInitialized_) { return false; }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

    if(wBuff && writeLen > 0) {
        i2c_master_write(cmd, wBuff, writeLen, true);
    }

    i2c_master_start(cmd);  // Mid-start

    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if(readLen > 1) {
        i2c_master_read(cmd, rBuff, readLen-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rBuff + readLen-1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (result == ESP_OK);
}

/**
* @brief Write 1 byte to the I2C device's register from the buffer
* @param address I2C Address of the device
* @param regAddr I2C Device's register to read
* @param wBuff Data buffer to write from
* @return `true` if the register write was successful, `false` otherwise
*/
bool cobalt::hal::I2CESPIDF::writeReg(uint8_t address, uint8_t regAddr, uint8_t wBuff) {
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
bool cobalt::hal::I2CESPIDF::readReg(uint8_t address, uint8_t regAddr, uint8_t *rBuff) {
    uint8_t buff[1] = { regAddr };
    if(!write(address, buff)) { return false; }
    return read(address, rBuff);
}
#endif