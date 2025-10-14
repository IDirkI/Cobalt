#include "cobalt/hal/gpio/gpio_espidf.hpp"

#if (defined(IDF_VER) && !defined(ARDUINO_ARCH_ESP32))

// ---------------- ESPIDF GPIO Private Helper Functions ----------------
/**
 * @brief Help pick an unused or the least used channel for a PWM pin
 */
inline ledc_channel_t cobalt::hal::GPIOESPIDF::allocateChannel() {
    uint8_t minAllocation = 0;
    uint8_t minAllocationIndex = 0;
    for(uint8_t i = 0; i < LEDC_CHANNEL_MAX; i++) {
        if(channelAllocationCounts_[i] == 0) { 
            channelAllocationCounts_[i]++;
            return static_cast<ledc_channel_t>(i); 
        } 

        if(channelAllocationCounts_[i] < minAllocation) { 
            minAllocation = channelAllocationCounts_[i];
            minAllocationIndex = i;
        }
    }
    channelAllocationCounts_[minAllocationIndex]++;
    return (static_cast<ledc_channel_t>(minAllocationIndex));
}

inline ledc_timer_t cobalt::hal::GPIOESPIDF::allocateTimer() {
    for(uint8_t i = 0; i < LEDC_TIMER_MAX; i++) {
        if((timerAllocationCounts_[i].isUsed) && (timerAllocationCounts_[i].frequency == frequency_)) {
            return static_cast<ledc_timer_t>(i);
        }
    }

    for(uint8_t i = 0; i < LEDC_TIMER_MAX; i++) {
        if(!timerAllocationCounts_[i].isUsed) { 
            timerAllocationCounts_[i].isUsed = true;
            timerAllocationCounts_[i].frequency = frequency_;
            return static_cast<ledc_timer_t>(i); 
        } 
    }

    return static_cast<ledc_timer_t>(0); // No free timers, (?) handle error
}

// ---------------- Constructors ----------------
/**
 * @brief Constructor for ESPIDF GPIO implementation layer
 */
cobalt::hal::GPIOESPIDF::GPIOESPIDF(gpio_num_t pin, float frequency): pin_(pin), frequency_(frequency) {
    channel_ = allocateChannel();
    timer_ = allocateTimer();
}

// ---------------- ESPIDF GPIO Getters/Setters ----------------
/**
* @brief Return the CHANNEL number of the pin used for PWM outputs
*/        
uint8_t cobalt::hal::GPIOESPIDF::getChannelNum() { return static_cast<uint8_t>(channel_); }

/**
* @brief Return the TIMER number of the pin used for PWM outputs
*/  
uint8_t cobalt::hal::GPIOESPIDF::getTimerNum() { return static_cast<uint8_t>(timer_); }

// ---------------- ESPIDF GPIO Member Functions ----------------
/**
* @brief Set the GPIO pins mode/type
* @param mode  `Input`, `Output` or `InnputPullUp`
*/
void cobalt::hal::GPIOESPIDF::setMode(GPIOMode mode) {
    switch(mode) {
        case(GPIOMode::Input): { gpio_set_direction(pin_, GPIO_MODE_INPUT); break; }
        case(GPIOMode::Output): { gpio_set_direction(pin_, GPIO_MODE_OUTPUT); break; }
        case(GPIOMode::InputPullUp): { gpio_set_direction(pin_, GPIO_MODE_INPUT); break; }
        case(GPIOMode::PWM): {
            ledc_timer_config_t timerConfig = {
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = timer_,
                .freq_hz = static_cast<uint32_t>(frequency_),
                .clk_cfg = LEDC_AUTO_CLK
            };

            ledc_timer_config(&timerConfig);

            ledc_channel_config_t channel_config = {
                .gpio_num = pin_,
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .channel = channel_,
                .timer_sel = timer_,
                .duty = 0,
                .hpoint = 0
            };

            ledc_channel_config(&channel_config);
            
            break;
        }
        default: { gpio_set_direction(pin_, GPIO_MODE_DISABLE); break; }
    }
}

/**
* @brief Set the state of the GPIO pin
* @param level  `High` or `Low`
* @note Only available when the pin is in `Output` mode
*/
void cobalt::hal::GPIOESPIDF::write(GPIOLevel level) {
    gpio_set_level(pin_, static_cast<uint8_t>(level));
}

/**
* @brief Set the GPIO pin to output a pwm with a +duty cycle of `duty`
* @param duty +Duty-cycle of the PWM signal, [0.0, 1.0]
* @note Only works if the hardwares pin support it
*/
void cobalt::hal::GPIOESPIDF::pwm(float duty) {
    uint32_t pwmVal = duty*1023;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_, pwmVal);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_);
}

/**
* @brief Read the state of the GPIO pin
* @return `High` or `Low`
*/
[[nodiscard]] cobalt::hal::GPIOLevel cobalt::hal::GPIOESPIDF::read() const {
    return ((gpio_get_level(pin_)) ?GPIOLevel::High :GPIOLevel::Low);
}

#endif