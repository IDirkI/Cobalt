#pragma once

#include <string>

namespace cobalt::util::meta {

struct PlatformInfo {

    /**
     *  @brief Get the current framework used to run the code
     */
    static const char* framework() {
        #if defined(ARDUINO)
            return "ARDUINO";
        #elif defined(ESP_PLATFORM)
            return "ESP-IDF";
        #else
            return "UNKNWON";
        #endif 
    }

    /**
     *  @brief Get the current framework used to run the code
     */
    static const char* framework_version() {
        static char buff[16];

        #if defined(ARDUINO)
            snprintf(buff, sizeof(buff), "%d", ARDUINO);
        #elif defined(ESP_PLATFORM)
            snprintf(buff, sizeof(buff), "%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR);
        #else
            return "UNKNOWN";
        #endif 

        return buff;
    }

    /**
     *  @brief Get the current CPU architecture used to run the code
     */
    static const char* architecture() {
        #if defined(__x86_64__) || defined(_M_X64)
            return "x86_64";
        #elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
            return "x86_32";
        #elif defined(__ARM_ARCH_2__)
            return "ARM2";
        #elif defined(__ARM_ARCH_3__) || defined(__ARM_ARCH_3M__)
            return "ARM3";
        #elif defined(__ARM_ARCH_4T__) || defined(__TARGET_ARM_4T)
            return "ARM4T";
        #elif defined(__ARM_ARCH_5_) || defined(__ARM_ARCH_5E_)
            return "ARM5"
        #elif defined(__ARM_ARCH_6T2_) || defined(__ARM_ARCH_6T2_)
            return "ARM6T2";
        #elif defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6J__) || defined(__ARM_ARCH_6K__) || defined(__ARM_ARCH_6Z__) || defined(__ARM_ARCH_6ZK__)
            return "ARM6";
        #elif defined(__ARM_ARCH_7__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
            return "ARM7";
        #elif defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
            return "ARM7A";
        #elif defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
            return "ARM7R";
        #elif defined(__ARM_ARCH_7M__)
            return "ARM7M";
        #elif defined(__ARM_ARCH_7S__)
            return "ARM7S";
        #elif defined(__aarch64__) || defined(_M_ARM64)
            return "ARM64";
        #elif defined(mips) || defined(__mips__) || defined(__mips)
            return "MIPS";
        #elif defined(__sh__)
            return "SUPERH";
        #elif defined(__powerpc) || defined(__powerpc__) || defined(__powerpc64__) || defined(__POWERPC__) || defined(__ppc__) || defined(__PPC__) || defined(_ARCH_PPC)
            return "POWERPC";
        #elif defined(__PPC64__) || defined(__ppc64__) || defined(_ARCH_PPC64)
            return "POWERPC64";
        #elif defined(__sparc__) || defined(__sparc)
            return "SPARC";
        #elif defined(__m68k__)
            return "M68K";
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
        #elif defined(__AVR__)
            return "AVR";
        #else
            return "UNKNOWN";
        #endif 
    }
};
    
}   // cobalt::util::meta