cmake_minimum_required(VERSION 3.16)

## Third Party Libraries

# Use: Formatting of strings (Will be the implementation for string formatting in C++20)
if (NOT TARGET fmt)
    find_package(fmt REQUIRED)
endif (NOT TARGET fmt)

# Use:Logger
if (NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif (NOT TARGET spdlog)

if (NOT TARGET nlohmann_json)
    find_package(nlohmann_json 3.10.2 REQUIRED)
endif (NOT TARGET nlohmann_json)

if (NOT TARGET YAMLCPP)
    find_package(YAMLCPP REQUIRED)

    set(YAMLCPP_INCLUDE_DIR "${YAMLCPP_INCLUDE_DIR}" PARENT_SCOPE)
    set(YAMLCPP_LIBRARY "${YAMLCPP_LIBRARY}" PARENT_SCOPE)
endif (NOT TARGET YAMLCPP)

# Use: Mixed-Integer Linear Programming Solver
if (NOT TARGET GUROBI)
    find_package(GUROBI REQUIRED)

    set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIRS}" PARENT_SCOPE)
    set(GUROBI_CXX_LIBRARY "${GUROBI_CXX_LIBRARY}" PARENT_SCOPE)
    set(GUROBI_LIBRARY "${GUROBI_LIBRARY}" PARENT_SCOPE)
endif (NOT TARGET GUROBI)

# Use: Matrix algebra library
if (NOT TARGET Eigen3)
    # Install via apt
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif (NOT TARGET Eigen3)

# Use: Motion Planning library
if (NOT TARGET ompl)
    # Build and install
    find_package(ompl 1.5.2 REQUIRED)
endif (NOT TARGET ompl)

# Use: Geometry library
if (NOT TARGET GEOS)
    # Install via apt
    find_package(GEOS REQUIRED)

    set(GEOS_INCLUDE_DIRS "${GEOS_INCLUDE_DIRS}" PARENT_SCOPE)
    set(GEOS_CXX_LIBRARY "${GEOS_CXX_LIBRARY}" PARENT_SCOPE)
    set(GEOS_LIBRARY "${GEOS_LIBRARY}" PARENT_SCOPE)
endif (NOT TARGET GEOS)

# Use: Machine Learning library
#if(NOT TARGET Torch)
#    # Install via pip
#    find_package(Torch REQUIRED)
#endif(NOT TARGET Torch)

# Use: fast maps and sets
if (NOT TARGET robin_hood)
    add_subdirectory(robin_hood)
endif (NOT TARGET robin_hood)

# Use: enum reflection
if (NOT TARGET magic_enum)
    add_subdirectory(magic_enum)
endif (NOT TARGET magic_enum)

# Use: comma line arguments
if (NOT TARGET cli11)
    add_subdirectory(cli11)
endif (NOT TARGET cli11)

list(APPEND LCOV_REMOVE_PATTERNS "'${CMAKE_CURRENT_SOURCE_DIR}/*'")