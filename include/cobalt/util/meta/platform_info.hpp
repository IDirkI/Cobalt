#pragma once

#include <stdio.h>
#include <string>

namespace cobalt::util::meta {

#if defined(ARDUINO) || defined(STM32) || defined(ESP_PLATFROM) || defined(__AVR__)
    #define COBALT_PLATFORM_EMBEDDED
#else 
    #define COBALT_PLATFORM_DESKTOP
#endif 

struct PlatformInfo {

    /**
     *  @brief Check if the current platform is an embedded platform (e.g. microcontroller) or a desktop platform (e.g. PC)
     */
    static constexpr bool isEmbedded() {
        #if defined(COBALT_PLATFORM_EMBEDDED)
            return true;
        #else  
            return false;
        #endif
    }

    /**
     *  @brief Get the current CPU architecture used to run the code
     */
    static const char* architecture() {
        #if defined(__x86_64__) || defined(_M_X64)
            return "x86_64";
        #elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
            return "x86_32";
        #elif defined(__aarch64__) || defined(_M_ARM64)
            return "ARM64";
        #elif defined(__ARM_ARCH_7M__)
            return "ARM Cortex-M";
        #elif defined(__ARM_ARCH_7A__)
            return "ARM Cortex-A";
        #elif defined(__ARM_ARCH_7__)
            return "ARM7";
        #elif defined(__ARM_ARCH_6__)
            return "ARM6";
        #elif defined(__AVR__)
            return "AVR";
        #elif defined(mips) || defined(__mips__) || defined(__mips)
            return "MIPS";
        #elif defined(__powerpc__)
            return "POWERPC";
        #elif defined(__powerpc64__)
            return "POWERPC64";
        #elif defined(__sparc__)
            return "SPARC";
        #else
            return "UNKNOWN";
        #endif
    }

    /**
     *  @brief Get the current OS/RTOS the code is running in
     */
    static const char* os() {
        #if defined(_WIN64)
            return "Windows 64-Bit";
        #elif defined(_WIN32)
            return "Windows 32-Bit";
        #elif defined(__linux__)
            return "Linux";
        #elif defined(__APPLE__) || defined(__MACH__)
            return "macOS";
        #elif defined(ESP_PLATFORM)
            return "ESP-IDF";
        #elif defined(ARDUINO)
            return "Arduino";
        #elif defined(STM32)
            return "STM32 HAL";
        #elif defined(__AVR__)
            return "AVR libc";
        #else
            return "UNKNOWN";
        #endif 
    }
};
    
}   // cobalt::util::meta