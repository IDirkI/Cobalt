cmake_minimum_required(VERSION 3.15)
project(Cobalt 
    VERSION 2.0.0 
    LANGUAGES C CXX
)

## ===== LIB VERSION ======
set(COBALT_VERSION_MAJOR 2)
set(COBALT_VERSION_MINOR 0)
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

## ===== PARSERS =====
    # ----- ROB -----
    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    file(GLOB ROBOT_FILES "${CMAKE_SOURCE_DIR}/tools/kinematics/robots/*.rob")
    set(ROBOT_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/include/cobalt/kinematics/robots")

    set(ROB_PARSER "${CMAKE_SOURCE_DIR}/tools/kinematics/rob_parser.py")
    if(NOT EXISTS "${ROB_PARSER}")
        message(FATAL_ERROR "rob_parser.py not found at ${ROB_PARSER}")
    endif()

    file(MAKE_DIRECTORY ${ROBOT_OUTPUT_DIR})

    set(ROBOT_GENERATED_HEADERS "")
    foreach(ROBOT_FILE ${ROBOT_FILES})
        get_filename_component(ROB_FILE_NAME ${ROBOT_FILE} NAME_WE)
        set(ROBOT_OUT_FILE "${ROBOT_OUTPUT_DIR}/${ROB_FILE_NAME}.hpp")
        
        add_custom_command(
            OUTPUT  ${ROBOT_OUT_FILE}
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/kinematics/rob_parser.py ${ROB_FILE_NAME}.rob -o
            DEPENDS ${ROBOT_FILE} ${ROB_PARSER}
            COMMENT "Parsing ${ROB_FILE_NAME}.rob ==> ${ROB_FILE_NAME}.hpp (${ROBOT_OUT_FILE})"
            VERBATIM
        )
        
        list(APPEND ROBOT_GENERATED_HEADERS ${ROBOT_OUT_FILE})
    endforeach()

    add_custom_target(generate_robots ALL DEPENDS ${ROBOT_GENERATED_HEADERS})

## ===== LIBRARY TARGETS =====
add_library(cobalt STATIC)

target_sources(cobalt PRIVATE ${COBALT_SOURCES})

target_include_directories(cobalt 
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)
target_compile_features(cobalt PUBLIC cxx_std_17)

add_dependencies(cobalt 
    generate_robots # .rob files
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

## ===== INSTALL =====
include(GNUInstallDirs)
install(TARGETS cobalt
    EXPORT CobaltTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION include)