cmake_minimum_required(VERSION 3.24)
project(hungarian_matching)

set(CMAKE_CXX_STANDARD 14)
include_directories(/usr/include/eigen3)

add_executable(hungarian_matching main.cpp hm_optimizer.h)
