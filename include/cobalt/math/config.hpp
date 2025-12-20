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

    // ---------------- Template type checking ----------------
    template<typename T>
        constexpr bool Scalar = std::is_arithmetic_v<T>;

    template<typename T>
        constexpr bool Floating = std::is_floating_point_v<T>;
    

    // ---------------- Contstants ----------------
    /**
     *  @brief Default universal epsilon for equality checking
     */
    template<typename T = def_scalar>
        inline constexpr T epsilon_ = (std::numeric_limits<T>::epsilon() * static_cast<T>(100));

    template<typename T = def_scalar, typename = std::enable_if_t<Floating<T>>>
        inline constexpr T pi_ = 3.14159265358979323846;

    template<typename T = def_scalar, typename = std::enable_if_t<Floating<T>>>
        inline constexpr T e_ = 2.71828182845904523536;

} // cobalt::math