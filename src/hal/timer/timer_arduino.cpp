#include "cobalt/hal/timer/timer.hpp"

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

void cobalt::hal::wait(uint32_t millis) {
    delay(millis);
}

unsigned long cobalt::hal::elapsed() {
    return millis();
}

unsigned long cobalt::hal::elapsedUs() {
    return micros();
}

void cobalt::hal::setStamp() {
    cobalt::hal::timeStamp = elapsedUs();
}
unsigned long cobalt::hal::checkStamp() {
    return (elapsedUs() - cobalt::hal::timeStamp);
}

#endif