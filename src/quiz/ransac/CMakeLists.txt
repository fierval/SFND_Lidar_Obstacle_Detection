cmake_minimum_required(VERSION 3.2)

project(playback LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.9.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
add_compile_definitions(POINT_CLOUD_FILE="C:/git/udacity/LidarDetection/src/sensors/data/pcd/simpleHighway.pcd")
target_link_libraries (quizRansac ${PCL_LIBRARIES})