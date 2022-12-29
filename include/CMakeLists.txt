cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE HEADERS **.hpp)
set(HEADERS ${HEADERS} PARENT_SCOPE)

# Configure config file
configure_file(
        "${PROJECT_SOURCE_DIR}/include/grstapse/config.hpp.in"
        "${PROJECT_BINARY_DIR}/include/grstapse/config.hpp"
        @ONLY
)

# Create core file (Is this bad for any reason? Compile Time?)
set(CORE_INCLUDES "")
foreach (INCLUDE IN ITEMS ${HEADERS})
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" INCLUDE "${INCLUDE}")
    string(APPEND CORE_INCLUDES "#include \"${INCLUDE}\"\n")
endforeach ()

configure_file(
        "${PROJECT_SOURCE_DIR}/include/grstapse/core.hpp.in"
        "${PROJECT_BINARY_DIR}/include/grstapse/core.hpp"
        @ONLY
)