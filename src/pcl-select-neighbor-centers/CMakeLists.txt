cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-select-neighbor-centers)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_SELECT_NEIGHBOR_CENTERS_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(pcl-select-neighbor-centers ${PCL_SELECT_NEIGHBOR_CENTERS_SRC})

target_link_libraries (pcl-select-neighbor-centers ${PCL_LIBRARIES})
