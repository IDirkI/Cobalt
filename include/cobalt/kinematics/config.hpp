#pragma once

#include <cstdint>
#include <limits>

namespace cobalt::kinematics {
    // ---------------- Types ----------------
    /**
     *  @brief Default alias for ids
     */
    using id_t  = std::uint16_t;

    /**
     *  @brief Default alias for iterations
     */
    using iter_t  = std::uint16_t;
    
    // ---------------- Constnats ----------------
    /**
     *  @brief Default invalid index value
     */
    inline constexpr id_t invalidID_ = std::numeric_limits<id_t>::max();

} // cobalt::kinematics