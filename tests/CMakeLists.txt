# Build tests
add_executable(cobalt_tests
    sanity_test.cpp

    # ===== Math =====
        # ----- Vector -----
        ./math/linear_algebra/vector/vector_core_test.cpp
        ./math/linear_algebra/vector/vector_ops_test.cpp
        ./math/linear_algebra/vector/vector_util_test.cpp

        # ----- Matrix -----
        ./math/linear_algebra/matrix/matrix_core_test.cpp
        ./math/linear_algebra/matrix/matrix_ops_test.cpp

        # ----- Complex -----
        #./math/algebra/complex_test.cpp
        #./math/linear_algebra/complex_vector_test.cpp

        # ----- Quaternion -----
        #./math/geometry/quaternion_test.cpp

        # ----- Transform -----
        #./math/geometry/transform_test.cpp

    # ===== Control =====
    #./control/controller/pid_test.cpp

    # ===== Kinematics =====
    #./kinematics/kinematics_test.cpp
    #./kinematics/ik_graphic_test.cpp

    # ===== Utility =====
    #./util/meta_test.cpp
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