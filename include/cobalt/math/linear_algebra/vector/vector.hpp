#pragma once

#include <stdint.h>
#include <cmath>
#include <array>
#include <string>

namespace cobalt::math::linear_algebra {

constexpr uint8_t VECTOR_MAX_SIZE = 12;

constexpr float   VECTOR_EQUAL_THRESHOLD = 1e-5;
constexpr float   VECTOR_ZERO_THRESHOLD = 1e-12;

constexpr uint8_t   VECTOR_DEFAULT_PRECISION = 3;

// --------------------------------------
//          N-Dimentional Vector    
// --------------------------------------

/**
 *  @brief Fixed-size vector.
 *  @tparam N Dimention/size of the vector.
 *  @tparam T Element type (default float).
 */
template<uint8_t N, typename T = float>
struct Vector{
    static_assert(N > 0                  , "[VECTOR Error] : Size must be positive.");
    static_assert(N <= VECTOR_MAX_SIZE   , "[VECTOR Error] : Size exceeds maximum size.");

    private:
        std::array<T, N> data_{};
    
    public:
        // ---------------- Constructors ----------------

        /**
         *  @brief Construct a zero-initialized vector.
         */ 
        constexpr Vector() noexcept : data_{} {}

        /**
         *  @brief Construct a vector from an initializer list.
         *  @param list Initializer list to copy values from.
         */ 
        Vector(std::initializer_list<T> list) {
            uint8_t i = 0;

            for(T val : list) {
                if(i < N) data_[i] = val;
                i++;
            }

            for(; i < N; i++) { data_[i] = static_cast<T>(0); }
        }

        /**
         *  @brief Construct a vector from an list of elements.
         *  @param args Elements of the vector (matches the length N).
         */
        template<typename... Args, typename = std::enable_if_t<sizeof...(Args) == N>>
            constexpr Vector(Args... args) noexcept : data_{{ static_cast<T>(args)... }} {}

        // ---------------- Static Factories ----------------

        /**
         *  @brief Construct a zero vector.
         */
        static constexpr Vector zero() noexcept { return Vector(); }

        /**
         *  @brief Construct a unit vector in the +x direction.
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>>
            static constexpr Vector unitX() noexcept {
                if constexpr (M == 2)   { return Vector{static_cast<T>(1), static_cast<T>(0)}; }
                else                    { return Vector{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0)}; }
            }
        
        /**
         *  @brief Construct a unit vector in the +y direction.
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>>
            static constexpr Vector unitY() noexcept {
                if constexpr (M == 2)   { return Vector{static_cast<T>(0), static_cast<T>(1)}; }
                else                    { return Vector{static_cast<T>(0), static_cast<T>(1), static_cast<T>(0)}; }
            }
        
        /**
         *  @brief Construct a unit vector in the +z direction.
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>>
            static constexpr Vector unitZ() noexcept { return Vector{static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)}; }
        

        // ---------------- Getters ----------------
        /**
         *  @brief Return the size of the vector.
         */
        constexpr uint8_t size() { return N; }


        // ---------------- Special Accessors ----------------
        /**
         *  @brief Access the x-component (for 2D/3D-vectors).
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            constexpr T &x() { return data_[0]; }
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            const T &x() const { return data_[0]; }
        
        /**
         *  @brief Access the y-component (for 2D/3D-vectors).
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            constexpr T &y() { return data_[1]; }
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            const T &y() const { return data_[1]; }

        /**
         *  @brief Access the z-component (for 3D-vectors).
         */
        template<uint8_t M = N, typename = std::enable_if_t<(M == 3)>> 
            constexpr T &z() { return data_[2]; }
        template<uint8_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            const T &z() const { return data_[2]; }


        // ---------------- Element Accessors ----------------
        /**
         *  @brief Access element at the given index.
         *  @param n Index of the accessed element.
         *  @return Reference to element.
         */
        constexpr T &operator[](uint8_t n) { if(n >= N) n = N-1; return data_[n]; }

        /**
         *  @brief Const access to element at the given index.
         *  @param n Index of the accessed element.
         *  @return Const reference to element.
         */
        const T &operator[](uint8_t n) const { if(n >= N) n = N-1; return data_[n]; }
        
        // ---------------- Arithmetic Overloads ----------------
        /**
         *  @brief Add another vector to this vector.
         */
        constexpr Vector &operator+=(const Vector &rhs) {
            for(uint8_t i = 0; i < N; i++) { data_[i] += rhs.data_[i]; }
            return *this;
        }

        /**
         *  @brief Subtarct another vector from this vector.
         */
        constexpr Vector &operator-=(const Vector &rhs) {
            for(uint8_t i = 0; i < N; i++) { data_[i] -= rhs.data_[i]; }
            return *this;
        }

        /**
         *  @brief Scalar multiply the vector.
         */
        constexpr Vector &operator*=(T c) {
            for(T &e : data_) { e *= c; }
            return *this;
        }

        /**
         *  @brief Scalar divide the vector.
         */
        constexpr Vector &operator/=(T c) {
            for(T &e : data_) { e /= c; }
            return *this;
        }

        /**
         *  @brief Flip the vector. Element wise negation.
         */
        constexpr Vector &operator-() {
            *this *= -1;
            return *this;
        }

        // ---------------- Utility  ----------------
        
        std::string toString(uint8_t percision = VECTOR_DEFAULT_PRECISION) const;
};

} // cobalt::math::linear_algebra