cmake_minimum_required(VERSION 3.2)

project(playback LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

message(STATUS "PCL INCLUDES: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL LIBRARIES: ${PCL_LIBRARIES}")

set(SOURCE_FILES 
      src/environment.cpp 
      src/render/render.cpp 
      src/processPointClouds.cpp)

add_executable (environment ${SOURCE_FILES})
target_link_libraries (environment ${PCL_LIBRARIES})




