cmake_minimum_required(VERSION 3.27)
project(kalman_filter_four_wheel_model)

set(CMAKE_CXX_STANDARD 23)

include_directories(include)

add_subdirectory(submodules)

set(SRC_Files
    src/kf_bicycle_model.cpp)

set(HEADER_Files
        src/kf_bicycle_model.h)

add_library(kalman_filter_four_wheel_model
        ${SRC_FILES}
        ${HEADER_FILES}
)

target_link_libraries(kalman_filter_four_wheel_model PUBLIC simplified_four_wheel_model)

target_include_directories(kalman_filter_four_wheel_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(kalman_filter_four_wheel_model_executable
        main.cpp
)

target_link_libraries(kalman_filter_four_wheel_model_executable PUBLIC kalman_filter_four_wheel_model)