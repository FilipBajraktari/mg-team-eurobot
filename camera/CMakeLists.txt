cmake_minimum_required(VERSION 3.2)
project( Camera )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( camera camera.cpp )

target_link_libraries( camera ${OpenCV_LIBS} )
target_include_directories( camera PUBLIC "${PROJECT_SOURCE_DIR}/include/aruco" )
