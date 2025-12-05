#pragma once

#include <stdint.h>
#include <cassert>
#include <array>

#include "../../types.hpp"

namespace cobalt::math::linear_algebra {

constexpr types::index_t VECTOR_MAX_SIZE = 12;

template<typename T = float>
    constexpr T VECTOR_EPSILON = static_cast<T>(1e-6);

// --------------------------------------
//          N-Dimentional Vector    
// --------------------------------------

/**
 *  @brief Fixed-size vector
 *  @tparam N Dimention/size of the vector
 *  @tparam T Element type (default float)
 */
template<types::index_t N, typename T = float>
struct Vector{
    static_assert(N > 0                  , "[VECTOR Error] : Size must be positive.");
    static_assert(N <= VECTOR_MAX_SIZE   , "[VECTOR Error] : Size exceeds maximum size.");

    private:
        std::array<T, N> data_{};
    
    public:
        // ---------------- Constructors ----------------

        /**
         *  @brief Default constructor (zero-initializes elements)
         */ 
        constexpr Vector() noexcept : data_{} {}

        /**
         *  @brief Construct a vector from an initializer list
         *  @param list Initializer list of elements
         *  @note If the list has fewer than N elements, remaining elements are zero-initialized
         */ 
        Vector(std::initializer_list<T> list) {
            types::index_t i = 0;

            for(T val : list) {
                if(i < N) data_[i] = val;
                i++;
            }

            for(; i < N; i++) { data_[i] = static_cast<T>(0); }
        }

        /**
         *  @brief Construct a vector from variadic arguments
         *  @param args Variadic list of elements
         *  @warning Compile-time error if number of arguments is not N
         */
        template<typename... Args, typename = std::enable_if_t<sizeof...(Args) == N>>
            constexpr Vector(Args... args) noexcept : data_{{ static_cast<T>(args)... }} {}

        // ---------------- Static Factories ----------------

        /**
         *  @brief Create a zero-vector
         *  @return Zero vector of size N
         */
        static constexpr Vector zero() noexcept { return Vector(); }

        /**
         *  @brief Construct a unit vector in the +x direction
         *  @return Unit vector of size N in +x direction
         *  @note Only available for 2D and 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>>
            static constexpr Vector unitX() noexcept {
                if constexpr (M == 2)   { return Vector{static_cast<T>(1), static_cast<T>(0)}; }
                else                    { return Vector{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0)}; }
            }
        
        /**
         *  @brief Construct a unit vector in the +y direction
         *  @return Unit vector of size N in +y direction
         *  @note Only available for 2D and 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>>
            static constexpr Vector unitY() noexcept {
                if constexpr (M == 2)   { return Vector{static_cast<T>(0), static_cast<T>(1)}; }
                else                    { return Vector{static_cast<T>(0), static_cast<T>(1), static_cast<T>(0)}; }
            }
        
        /**
         *  @brief Construct a unit vector in the +z direction
         *  @return Unit vector of size N in +z direction
         *  @note Only available for 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 3)>>
            static constexpr Vector unitZ() noexcept { return Vector{static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)}; }
        

        /**
         *  @brief Construct a vector from an array
         *  @param arr Array to convert
         *  @return Vector constructed from the array
         */
        static constexpr Vector fromArray(const std::array<T, N> &arr) noexcept {
            Vector<N, T> v;
            for(types::index_t i = 0; i < N; i++) {
                v[i] = arr[i];
            }

            return v;
        }

        // ---------------- Getters ----------------
        /**
         *  @brief Get the size of the vector
         */
        static constexpr types::index_t size() noexcept { return N; }


        // ---------------- Special Accessors ----------------
        /**
         *  @brief Access the x-component (for 2D/3D-vectors)
         *  @return Reference to x-component
         *  @note Only available for 2D and 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            constexpr T &x() noexcept { return data_[0]; }
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            const T &x() const noexcept { return data_[0]; }
        
        /**
         *  @brief Access the y-component (for 2D/3D-vectors)
         *  @return Reference to y-component
         *  @note Only available for 2D and 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            constexpr T &y() noexcept { return data_[1]; }
        template<types::index_t M = N, typename = std::enable_if_t<(M == 2) || (M == 3)>> 
            const T &y() const noexcept { return data_[1]; }

        /**
         *  @brief Access the z-component (for 3D-vectors)
         *  @return Reference to z-component
         *  @note Only available for 3D vectors
         */
        template<types::index_t M = N, typename = std::enable_if_t<(M == 3)>> 
            constexpr T &z() noexcept { return data_[2]; }
        template<types::index_t M = N, typename = std::enable_if_t<(M == 3)>> 
            const T &z() const noexcept { return data_[2]; }


        // ---------------- Element Accessors ----------------
        /**
         *  @brief Access element at the given index
         *  @param n Index of the accessed element
         *  @warning No bounds checking. Use at() for safe access
         *  @note In debug mode, asserts if `n` >= `N`
         *  @return Reference to element
         */
        constexpr T &operator[](types::index_t n) noexcept { 
            assert(n < N && "[VECTOR Error] : Accessed index must be within vector size.");
            return data_[n]; 
        }

        /**
         *  @brief Const access element at the given index
         *  @param n Index of the accessed element
         *  @warning No bounds checking. Use at() for safe access
         *  @note In debug mode, asserts if `n` >= `N`
         *  @return Const reference to element
         */
        constexpr const T &operator[](types::index_t n) const noexcept { 
            assert(n < N && "[VECTOR Error] : Accessed index must be within vector size.");
            return data_[n];
        }

        /**
         *  @brief Safe access to element at the given index
         *  @param n Index of the accessed element
         *  @note Clamps the output to the last element if the asked index is out of bounds
         *  @return Reference to element
         */
        constexpr T &at(types::index_t n) noexcept { if(n >= N) n = N-1; return data_[n]; }

        /**
         *  @brief Const safe access to element at the given index
         *  @param n Index of the accessed element
         *  @note Clamps the output to the last element if the asked index is out of bounds
         *  @return Const reference to element
         */
        constexpr const T &at(types::index_t n) const noexcept { if(n >= N) n = N-1; return data_[n]; }

        /**
         *  @brief Access to raw data of the vector
         *  @return Raw data array
         */
        constexpr T* data() noexcept { return data_.data(); }

        /**
         *  @brief Const access to raw data of the vector
         *  @return Const raw data array
         */
        constexpr const T* data() const noexcept { return data_.data(); }

        /**
         *  @brief Access to the start of the data in memory
         *  @return Iterator to the start of the raw data array
         */
        constexpr auto begin() const noexcept { return data_.begin(); }

        /**
         *  @brief Access to the end of the data in memory
         *  @return Iterator to the end of the raw data array
         */
        constexpr auto end() const noexcept { return data_.end(); }
        
        // ---------------- Arithmetic Overloads ----------------
        /**
         *  @brief Add another vector to this vector
         */
        constexpr Vector &operator+=(const Vector &rhs) noexcept {
            for(types::index_t i = 0; i < N; i++) { data_[i] += rhs.data_[i]; }
            return *this;
        }

        /**
         *  @brief Subtarct another vector from this vector
         */
        constexpr Vector &operator-=(const Vector &rhs) noexcept {
            for(types::index_t i = 0; i < N; i++) { data_[i] -= rhs.data_[i]; }
            return *this;
        }

        /**
         *  @brief Scalar multiply the vector
         */
        constexpr Vector &operator*=(T c) {
            for(T &e : data_) { e *= c; }
            return *this;
        }

        /**
         *  @brief Scalar divide the vector
         */
        constexpr Vector &operator/=(T c) {
            for(T &e : data_) { e /= c; }
            return *this;
        }
};

} // cobalt::math::linear_algebra