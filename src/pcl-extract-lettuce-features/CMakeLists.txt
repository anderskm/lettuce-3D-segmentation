cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-extract-lettuce-features)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_EXTRACT_LETTUCE_FEATURES_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(pcl-extract-lettuce-features ${PCL_EXTRACT_LETTUCE_FEATURES_SRC})

target_link_libraries (pcl-extract-lettuce-features ${PCL_LIBRARIES})
