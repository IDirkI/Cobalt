#pragma once

#include <stdint.h>

namespace cobalt::hal {
    // --------------------------------------
    //      Base Timer Implementation    
    // --------------------------------------

    static unsigned long timeStamp = 0;

    // ---------------- Timer Functions ----------------

    /**
     * @brief Sleep the program for `millis` ms
     * @param millis Amount of time in ms to sleep
     */
    void wait(uint32_t millis);

    /**
     * @brief Get current time (ms)
     * @return Amount of time elapsed since the start of execution in ms
     */
    unsigned long elapsed();
    /**
     * @brief Get current time (μs)
     * @return Amount of time elapsed since the start of execution in μs
     */
    unsigned long elapsedUs();

    /**
     * @brief Set the timerStamp to the current elapsed time. 
     * 
     * Use `checkStamp()` later to calculate amount of time passed
     */
    void setStamp();
    /**
     * @brief Check the amount of time passed since the timeStamp placement
     * @return Amount of time elapsed since the `timeStamp` was placed in μs
     */
    unsigned long checkStamp();
    
} //cobalt::hal