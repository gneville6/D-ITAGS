cmake_minimum_required(VERSION 3.12...3.18)
project(cli11 CXX)
message("Building ${PROJECT_NAME}")

add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME}
        INTERFACE
        include/${PROJECT_NAME}/cli11.hpp)
target_compile_features(${PROJECT_NAME}
        INTERFACE
        cxx_std_20)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)