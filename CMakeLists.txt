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
add_library(cobalt STATIC ${COBALT_SOURCES}) # TODO: Fix for later use, auto include src
target_include_directories( cobalt 
    PUBLIC include 
)

# Tests
add_subdirectory(extern/Catch2)
add_subdirectory(tests)