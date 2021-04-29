cmake_minimum_required(VERSION 3.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(LIDAR_OBSTACLE_DETECTION_PROJECT)

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (obstacleDetection src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/helper.cpp src/kdtreecluster.cpp)
target_link_libraries (obstacleDetection ${PCL_LIBRARIES})








