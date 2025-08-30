cmake_minimum_required(VERSION 3.15)
project(Cobalt LANGUAGES C CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add libarary
add_library( cobalt INTERFACE)
target_include_directories( cobalt INTERFACE include)

# Debug Files
add_executable(cobalt_sanity_check src/debug.cpp)
target_include_directories(cobalt_sanity_check
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Tests
add_subdirectory(extern/Catch2)
add_subdirectory(tests)



