cmake_minimum_required(VERSION 3.16)
project(robin_hood CXX)
message("Building ${PROJECT_NAME}")

add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME}
        INTERFACE
        include/${PROJECT_NAME}/robin_hood.hpp)
target_compile_features(${PROJECT_NAME}
        INTERFACE
        cxx_std_20)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)