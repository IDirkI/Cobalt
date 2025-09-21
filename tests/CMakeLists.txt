# Build tests
add_executable(cobalt_tests
    sanity_test.cpp

    # ===== Math =====
    #./math/linear_algebra/vector_test.cpp
    #./math/linear_algebra/matrix_test.cpp
    #./math/linear_algebra/complex_vector_test.cpp

    #./math/algebra/complex_test.cpp

    #./math/geometry/quaternion_test.cpp
    #./math/geometry/transform_test.cpp

    # ===== Control =====
    #./control/controller/pid_test.cpp

    # ===== Kinematics =====
    ./kinematics/kinematics_test.cpp
)

# Include paths
target_include_directories(cobalt_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include      # Cobalt Headers
)

# Link Libraries
target_link_libraries(cobalt_tests PRIVATE cobalt Catch2::Catch2WithMain)

# Register the tests
enable_testing()
include(CTEST)
include(CATCH)
catch_discover_tests(cobalt_tests)