cmake_minimum_required(VERSION 3.5)
project(Coursera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES graph.hpp graph_algorithms.cpp)
add_executable(Coursera ${SOURCE_FILES})