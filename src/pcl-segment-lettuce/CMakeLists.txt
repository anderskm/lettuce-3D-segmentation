cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-segment-lettuce)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_SEGMENT_LETTUCE_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(pcl-segment-lettuce ${PCL_SEGMENT_LETTUCE_SRC})

target_link_libraries (pcl-segment-lettuce ${PCL_LIBRARIES})
target_link_libraries (pcl-segment-lettuce ${Boost_LIBRARIES})

