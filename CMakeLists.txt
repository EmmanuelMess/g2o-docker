cmake_minimum_required(VERSION 3.22)
project(g2otest)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")

set(G2O_LIBRARY_DIR "/root/g2o/lib")
set(G2O_INCLUDE_DIR "/root/g2o")

link_directories(${G2O_LIBRARY_DIR})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

add_subdirectory(slam2d)
add_subdirectory(gaussian_estimation)
add_subdirectory(moving_gaussian_estimation)