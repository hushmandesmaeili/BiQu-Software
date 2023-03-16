cmake_minimum_required(VERSION 3.0)
project( personTracking )
find_package( OpenCV REQUIRED )
find_package(realsense2 2.29.0)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_library( personTracking SHARED personTracking.cpp)
target_link_libraries( personTracking ${OpenCV_LIBS} )
target_link_libraries(personTracking ${DEPENDENCIES} ${realsense2_LIBRARY})

