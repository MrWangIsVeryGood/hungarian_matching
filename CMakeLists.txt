cmake_minimum_required(VERSION 3.24)
project(hun)

set(CMAKE_CXX_STANDARD 14)
include_directories(/usr/include/eigen3)

add_executable(hun main.cpp hm_optimizer.h)
