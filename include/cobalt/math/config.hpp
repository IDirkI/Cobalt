#pragma once

#include <cstddef>
#include <limits>
#include <type_traits>

namespace cobalt::math {
    // ---------------- Types ----------------
    /**
     *  @brief Default alias for indexing
     */
    using index_t  = std::size_t;

    /**
     *  @brief Default scalar value
     */
    using def_scalar = float;

    // ---------------- Contstants ----------------
    /**
     *  @brief Default universal epsilon for equality checking
     */
    template<typename T = def_scalar>
        inline constexpr T epsilon = (std::numeric_limits<T>::epsilon() * static_cast<T>(100));

    // ---------------- Template type checking ----------------
    template<typename T>
        constexpr bool Scalar = std::is_arithmetic_v<T>;

} // cobalt::math