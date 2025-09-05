cmake_minimum_required(VERSION 3.15)
project(Cobalt LANGUAGES C CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add sources
file(GLOB_RECURSE COBALT_SOURCES
    "src/*.cpp"
)

# Add libarary
add_library(cobalt STATIC src/control/controller/pid.cpp) # TODO: Fix for later use, auto include src
target_include_directories( cobalt 
    PUBLIC include 
)

# Debug files
add_executable(cobalt_sanity_check src/debug.cpp)
target_include_directories(cobalt_sanity_check
    PRIVATE include
)

# Tests
add_subdirectory(extern/Catch2)
add_subdirectory(tests)