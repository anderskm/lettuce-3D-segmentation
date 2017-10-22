cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-label-cloud)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_LABEL_CLOUD_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(pcl-label-cloud ${PCL_LABEL_CLOUD_SRC})

target_link_libraries (pcl-label-cloud ${PCL_LIBRARIES})
