set(_COBALT_PARSER_FLAGS "")

if(COBALT_ROB_PARSER_VERBOSE)
    list(APPEND _COBALT_PARSER_FLAGS -v)
endif()
if(COBALT_ROB_PARSER_DEBUG)
    list(APPEND _COBALT_PARSER_FLAGS -d)
endif()
if(COBALT_ROB_PARSER_WARNING)
    list(APPEND _COBALT_PARSER_FLAGS -w)
endif()

function(cobalt_generate_robot_headers)
    cmake_parse_arguments(ARG "" "TARGET" "" ${ARGN})

    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    # Default input/output directories
    set(ROBOT_INPUT_DIR "${PROJECT_SOURCE_DIR}/robots")
    set(ROBOT_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/generated")
    file(MAKE_DIRECTORY ${ROBOT_OUTPUT_DIR})

    # Locate parser.py from installed Cobalt
    find_file(ROB_PARSER
        NAMES parser.py
        PATHS "${Cobalt_DIR}/../tools/rob_parser"
        NO_DEFAULT_PATH
    )
    if(NOT ROB_PARSER)
        message(FATAL_ERROR "Could not find parser.py in Cobalt tools directory")
    endif()

    # Collect all .rob files
    file(GLOB ROBOT_FILES "${ROBOT_INPUT_DIR}/*.rob")
    if(NOT ROBOT_FILES)
        message(WARNING "No .rob files found in ${ROBOT_INPUT_DIR}")
    endif()

    # Create a custom command for each .rob file
    set(ROBOT_GENERATED_HEADERS "")

    add_custom_command(
        OUTPUT ${ROBOT_OUTPUT_DIR}
        COMMAND ${Python3_EXECUTABLE} 
                ${ROB_PARSER} 
                ${_COBALT_PARSER_FLAGS} 
                ${ROBOT_FILES} 
                ${ROBOT_OUTPUT_DIR}
        DEPENDS ${ROB_PARSER} ${ROBOT_FILE}
        COMMENT "Parsing ${ROBOT_INPUT_DIR}/*.rob --> ${ROBOT_OUTPUT_DIR}/*.hpp"
        VERBATIM
    )

    list(APPEND ROBOT_GENERATED_HEADERS ${ROBOT_OUTPUT_DIR})

    # Create parser target
    add_custom_target(generate_robots
        DEPENDS ${ROBOT_GENERATED_HEADERS}
    )

    # Automatically attach to executable if TARGET argument given
    if(ARG_TARGET)
        add_dependencies(${ARG_TARGET} generate_robots)
        target_include_directories(${ARG_TARGET} PRIVATE "${ROBOT_OUTPUT_DIR}")
    endif()
endfunction()
