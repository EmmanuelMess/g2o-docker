cmake_minimum_required(VERSION 3.22)
project(g2otest)

set(CMAKE_CXX_STANDARD 17)

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")

set(G2O_LIBRARY_DIR "/root/g2o/lib")
set(G2O_INCLUDE_DIR "/root/g2o")

link_directories(${G2O_LIBRARY_DIR})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    tutorial_slam2d.cpp
)

target_link_libraries(${PROJECT_NAME}
    glog
    ${EIGEN3_LIBS}
    g2o_core
    g2o_solver_eigen
    g2o_solver_dense
    g2o_types_sba
    g2o_types_sim3
    g2o_types_slam2d
    g2o_types_slam2d_addons
    g2o_types_slam3d
    g2o_types_slam3d_addons
    g2o_stuff
    g2o_solver_cholmod
)