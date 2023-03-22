cmake_minimum_required(VERSION 3.0)

project(Lidar)

# Set the directory where the library file is located
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set the name of the library file
set(LIBRARY_NAME rplidar_driver)

# Add the library directory to the list of directories to search for include files
include_directories(${LIBRARY_DIR})

# Add the library file to the project
add_library(${LIBRARY_NAME} STATIC IMPORTED)
set_target_properties(${LIBRARY_NAME} PROPERTIES IMPORTED_LOCATION ${LIBRARY_DIR}/rplidar_driver.lib)

# Add the source files of your project
add_executable(lidar lidar.cpp)

# Link the library to your executable
target_link_libraries(lidar ${LIBRARY_NAME})
