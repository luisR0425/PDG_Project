cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(algoritmo)

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (algoritmo algoritmo.cpp)
target_link_libraries (algoritmo ${PCL_LIBRARIES})
