cmake_minimum_required(VERSION 3.15)
project(Cobalt 
    VERSION 2.2.0 
    LANGUAGES C CXX
)

## ===== LIB VERSION ======
set(COBALT_VERSION_MAJOR 2)
set(COBALT_VERSION_MINOR 2)
set(COBALT_VERSION_PATCH 0)

## ===== C++ STANDARD =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## ===== SOURCES =====
file(GLOB_RECURSE COBALT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
if (NOT COBALT_SOURCES)
    message(WARNING "No sources found for Cobalt in src/")
endif()

## ===== ROBOT PARSER FILES =====
set(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools/rob_parser")
set(PARSER_FILES
    "${PARSER_DIR}/parser.py"
    "${PARSER_DIR}/defaults.py"
)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

## ===== PARSER OPTIONS =====
option(ROB_PARSER_VERBOSE "Enable verbose logging for robot parser" OFF)
option(ROB_PARSER_DEBUG "Enable detailed debugger logs robot parser" OFF)
option(ROB_PARSER_WARNING "Enable warnings from robot parser" ON)

## ===== GENERATE ROBOT HEADERS WHEN BUILDING COBALT =====
if(PROJECT_NAME STREQUAL "Cobalt")
    # Output directory inside cobalt repo
    set(ROBOT_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
    file(MAKE_DIRECTORY "${ROBOT_OUTPUT_DIR}")

    # Find .rob files
    file(GLOB LOCAL_ROBOTS "${CMAKE_CURRENT_SOURCE_DIR}/robots/*.rob")

    if(NOT LOCAL_ROBOTS)
        message(STATUS "No .rob files found in robots/")
        return()
    endif()

    # Compute expected generated headers
    set(ROBOT_GENERATED_HEADERS "")
    foreach(ROB_FILE ${LOCAL_ROBOTS})
        get_filename_component(ROB_NAME ${ROB_FILE} NAME_WE)
        list(APPEND ROBOT_GENERATED_HEADERS
            "${ROBOT_OUTPUT_DIR}/${ROB_NAME}.hpp"
        )
    endforeach()

    add_custom_command(
        OUTPUT ${ROBOT_GENERATED_HEADERS}
        COMMAND ${Python3_EXECUTABLE}
            ${PARSER_DIR}/parser.py
            $<$<BOOL:${ROB_PARSER_VERBOSE}>:-v>
            $<$<BOOL:${ROB_PARSER_DEBUG}>:-d>
            $<$<BOOL:${ROB_PARSER_WARNING}>:-w>
            ${LOCAL_ROBOTS}
            ${ROBOT_OUTPUT_DIR}
        DEPENDS ${PARSER_FILES} ${LOCAL_ROBOTS}
        COMMENT "Parsing .rob files --> generated/*.hpp"
        VERBATIM
    )

    add_custom_target(generate_robots ALL
        DEPENDS ${ROBOT_GENERATED_HEADERS}
    )
endif()

## ===== LIBRARY TARGET =====
add_library(cobalt STATIC)
target_sources(cobalt PRIVATE ${COBALT_SOURCES})

# Include only the public headers; BUILD_INTERFACE for building Cobalt, INSTALL_INTERFACE for consumers
target_include_directories(cobalt
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)
target_compile_features(cobalt PUBLIC cxx_std_17)

# Make cobalt depend on robot generation only when building Cobalt
if(TARGET generate_robots)
    add_dependencies(cobalt generate_robots)
endif()

## ===== INSTALL =====
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    if(WIN32)
        set(CMAKE_INSTALL_PREFIX "C:/dev/Cobalt" CACHE PATH "Install path prefix")
    else()
        set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Install path prefix")
    endif()
endif()

include(GNUInstallDirs)

# Library
install(TARGETS cobalt
    EXPORT CobaltTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Public headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Base tools folder in the source tree
set(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools")
file(GLOB CHILD_DIRS RELATIVE "${PARSER_DIR}" "${PARSER_DIR}/*")
foreach(DIR ${CHILD_DIRS})
    if(IS_DIRECTORY "${PARSER_DIR}/${DIR}")
        message(STATUS "Installing parser tools from ${DIR}...")

        install(DIRECTORY "${PARSER_DIR}/${DIR}/"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/tools/${DIR}"
            FILES_MATCHING PATTERN "*.py"
            PATTERN "pio_*.py" EXCLUDE
        )
    endif()
endforeach()

install(EXPORT CobaltTargets
    FILE CobaltTargets.cmake
    NAMESPACE Cobalt::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Cobalt
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CobaltConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CobaltConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/CobaltConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CobaltConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CobaltConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Cobalt"
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CobaltFunctions.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Cobalt"
)

## ===== TESTS =====
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extern/Catch2/CMakeLists.txt")
    add_subdirectory(extern/Catch2)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
        add_subdirectory(tests)
    else()
        message(WARNING "Tests are found but no CMake file available.")
    endif()
else()
    message(STATUS "Compiling without tests")
endif()
