cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_mc)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( pcl_mc pcl_mc.cpp )
target_link_libraries( pcl_mc ${PCL_LIBRARIES} )
