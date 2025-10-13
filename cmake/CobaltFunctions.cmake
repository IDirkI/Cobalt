# ROB Parse function
    function(cobalt_generate_robot_headers)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        file(GLOB ROBOT_FILES "${CMAKE_SOURCE_DIR}/robots/*.rob")
        set(ROBOT_OUTPUT_DIR "${CMAKE_BINARY_DIR}/robots")
        set(ROB_PARSER "${Cobalt_DIR}/../tools/rob_parser.py")

        file(MAKE_DIRECTORY ${ROBOT_OUTPUT_DIR})

        set(ROBOT_GENERATED_HEADERS "")
        foreach(ROBOT_FILE ${ROBOT_FILES})
            get_filename_component(ROB_FILE_NAME ${ROBOT_FILE} NAME_WE)
            set(ROBOT_OUT_FILE "${ROBOT_OUTPUT_DIR}/${ROB_FILE_NAME}.hpp")
            
            add_custom_command(
                OUTPUT  ${ROBOT_OUT_FILE}
                COMMAND ${Python3_EXECUTABLE} ${ROB_PARSER} ${ROBOT_FILE} ${ROBOT_OUTPUT_DIR}
                DEPENDS ${ROB_PARSER} ${ROBOT_FILE}
                COMMENT "Parsing ${ROB_FILE_NAME}.rob ==> ${ROB_FILE_NAME}.hpp (${ROBOT_OUTPUT_DIR})"
                VERBATIM
            )
            
            list(APPEND ROBOT_GENERATED_HEADERS ${ROBOT_OUT_FILE})
        endforeach()

        add_custom_target(generate_robots ALL DEPENDS ${ROBOT_GENERATED_HEADERS})
    endfunction()