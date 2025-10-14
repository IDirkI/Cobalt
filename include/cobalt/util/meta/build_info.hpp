#pragma once

#include <stdio.h>

namespace cobalt::util::meta {

struct BuildInfo {
    /**
     *  @brief Date of latest compilation
     */
    static constexpr char* COMPILE_DATE = __DATE__;

    /**
     *  @brief Time of latest compilation
     */
    static constexpr char* COMPILE_TIME = __TIME__;

    /**
     *  @brief C++ standard of the library
     */
    static constexpr char* CPP_STANDARD = "C++17";

    /**
     *  @brief Get the compiler used to compile the current code with version info
     */
    static const char* compiler() {
        static char buff[32];

        #if defined(__MINGW64__)
            snprintf(buff, sizeof(buff), "MINGW64 %d.%d", __MINGW64_VERSION_MAJOR, __MINGW64_VERSION_MINOR);
        #elif defined(__MINGW32__)
            snprintf(buff, sizeof(buff), "MINGW32 %d.%d", __MINGW32_MAJOR_VERSION, __MINGW32_MINOR_VERSION);
        #elif defined(__GNUC__)
            snprintf(buff, sizeof(buff), "GCC %d.%d", __GNUC__, __GNUC_MINOR__);
        #elif defined(__clang__)
            snprintf(buff, sizeof(buff), "CLANG %d.%d", __clang_major__, __clang_minor__);
        #elif defined(_MSVC_VER)
            snprintf(buff, sizeof(buff), "MSVC %d",_MSVC_VER);
        #else
            return "UNKNWON";
        #endif 

        return buff;
    }
};
    
}   // cobalt::util::meta