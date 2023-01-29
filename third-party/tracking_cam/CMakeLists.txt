cmake_minimum_required(VERSION 3.1.0)
##project(t_cam)
set(DEPENDENCIES realsense2)
add_library(tracking_cam SHARED
    TCam.cpp)
add_executable(Tcam_test test/TCam_test.cpp)
set_property(TARGET Tcam_test PROPERTY CXX_STANDARD 11)
target_link_libraries(Tcam_test ${DEPENDENCIES})
target_link_libraries(Tcam_test tracking_cam)