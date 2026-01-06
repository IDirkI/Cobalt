#include "cobalt/hal/timer/timer.hpp"

#if (defined(IDF_VER) && !defined(ARDUINO_ARCH_ESP32))

#include <esp_timer.h>

void cobalt::hal::wait(uint32_t millis) {
    vTaskDelay(millis / portTICK_RATE_MS);
}

unsigned long cobalt::hal::elapsed() {
    return esp_timer_get_time() / 1000;
}

unsigned long cobalt::hal::elapsedUs() {
    return esp_timer_get_time();
}

void cobalt::hal::setStamp() {
    cobalt::hal::timeStamp = elapsedUs();
}
unsigned long cobalt::hal::checkStamp() {
    return (elapsedUs() - cobalt::hal::timeStamp);
}

#endif