cmake_minimum_required(VERSION 3.15)
project(Cobalt LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library Targets
##add_library( cobalt STATIC)

# Headers
##target_include_directories( cobalt 
##    PUBLIC
##        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
##        $<INSTALL_INTERFACE: include>
##)

# Library Targets
add_library( cobalt INTERFACE)

# Headers
target_include_directories( cobalt 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Checks
add_executable(cobalt_sanity_check src/debug.cpp)
target_include_directories(cobalt_sanity_check
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Examples
## add_executable("EXAMPLE" examples/"EXAMPLE".cpp)
## target_link_libraries("EXAMPLE" PRIVATE cobalt)

# Tests
enable_testing()
add_subdirectory(tests)



