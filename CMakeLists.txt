cmake_minimum_required(VERSION 3.15)
project(Cobalt VERSION 2.0.0 LANGUAGES C CXX)

set(COBALT_VERSION_MAJOR 2)
set(COBALT_VERSION_MINOR 0)
set(COBALT_VERSION_PATCH 0)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add sources
file(GLOB_RECURSE COBALT_SOURCES
    "src/*.cpp"
)

# Add libarary
add_library(cobalt STATIC ${COBALT_SOURCES})
target_include_directories( cobalt 
    PUBLIC include 
)

# Tests
if(EXISTS "../extern/Catch2/CMakeLists.txt")
    add_subdirectory(extern/Catch2)
endif()

if(EXISTS "../tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()