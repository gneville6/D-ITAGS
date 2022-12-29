cmake_minimum_required(VERSION 3.16)
project(grstapse
        VERSION 0.1
        DESCRIPTION "Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution"
        LANGUAGES CXX)
message("Building ${PROJECT_NAME}...")

# Store the language
get_property(LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)

set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

include(GNUInstallDirs)
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# Add external cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Determine if this project is built as a subproject (using add_subdirectory)
# or if it is the master project.
set(MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
endif (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)

option(BUILD_DOCUMENTATION "Build Documentation" ${MASTER_PROJECT})
option(BUILD_UNITTESTS "Build Unit Tests" ${MASTER_PROJECT})
option(BUILD_COVERAGE "Build Coverage" OFF)
option(BUILD_EXECUTABLE "Build Executable" OFF)

# include cmake modules
include(CodeCoverage)

# Add external libraries
add_subdirectory(extern)

# Add pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Collect header and source files
add_subdirectory(include)
add_subdirectory(src)

# Create library
add_library(_${PROJECT_NAME} STATIC)
target_sources(_${PROJECT_NAME}
        PUBLIC
        ${HEADERS}
        PRIVATE
        ${SOURCES})
target_precompile_headers(_${PROJECT_NAME}
        PUBLIC
        "${PROJECT_BINARY_DIR}/include/grstapse/config.hpp")
target_compile_features(_${PROJECT_NAME}
        PRIVATE
        cxx_std_20)
target_link_libraries(_${PROJECT_NAME}
        PUBLIC
        fmt
        magic_enum
        ompl
        robin_hood
        spdlog
        ${YAMLCPP_LIBRARY}
        ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY}
        ${GEOS_CXX_LIBRARY} ${GEOS_LIBRARY}
        )
target_include_directories(_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${YAMLCPP_INCLUDE_DIRS}
        ${GUROBI_INCLUDE_DIRS}
        ${GEOS_INCLUDE_DIRS})
target_compile_options(_${PROJECT_NAME}
        PRIVATE
        "$<$<CONFIG:RELEASE>:-Ofast>")
target_compile_definitions(_${PROJECT_NAME}
        PUBLIC
        "$<$<CONFIG:DEBUG>:DEBUG>")
if (BUILD_COVERAGE)
    target_code_coverage(_${PROJECT_NAME})
endif (BUILD_COVERAGE)

# Build documentation
if (BUILD_DOCUMENTATION)
    add_subdirectory(docs)
endif (BUILD_DOCUMENTATION)

# Build unit tests
if (BUILD_UNITTESTS)
    add_subdirectory(tests)
endif (BUILD_UNITTESTS)