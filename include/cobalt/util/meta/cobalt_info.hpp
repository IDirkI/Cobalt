#pragma once

#include <string>

namespace cobalt::util::meta {

struct CobaltInfo {
    static constexpr int VERSION_MAJOR = 2;
    static constexpr int VERSION_MINOR = 1;
    static constexpr int VERSION_PATCH = 0;

    /**
     * @brief Get the Cobalt version being used
     */
    static const char* version() {
        static char buff[16];
        snprintf(buff, sizeof(buff), "%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

        return buff;
    }
};
    
}   // cobalt::util::meta