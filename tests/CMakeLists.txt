cmake_minimum_required(VERSION 3.16)
message("Building tests...")

# TODO(Andrew): fix the CI to use gurobi
option(IGNORE_GUROBI "Ignores the gurobi unit tests" OFF)

# Collect headers/sources/data files
add_subdirectory(include)
add_subdirectory(src)

enable_testing()
find_package(GTest 1.11 REQUIRED)
include(GoogleTest)

# Create unit test executable
add_executable(unittests)
target_sources(unittests
        PRIVATE
        ${TEST_HEADERS} ${TEST_SOURCES})
target_compile_features(unittests
        PUBLIC
        cxx_std_20)
target_link_libraries(unittests
        PRIVATE
        ${GTEST_BOTH_LIBRARIES} gmock Threads::Threads
        _${PROJECT_NAME})
target_compile_options(unittests
        PRIVATE
        ${_opts})
target_include_directories(unittests
        PRIVATE
        include
        ${GTEST_INCLUDE_DIRS})
if (IGNORE_GUROBI)
    message("Ignoring Gurobi")
    target_compile_definitions(unittests PUBLIC NO_MILP)
endif (IGNORE_GUROBI)
if (BUILD_COVERAGE)
    target_code_coverage(unittests)
endif ()
gtest_discover_tests(unittests)

# Copy test data to outdir/test
add_custom_command(TARGET unittests PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/data $<TARGET_FILE_DIR:unittests>/data)