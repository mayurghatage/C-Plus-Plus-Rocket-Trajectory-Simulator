cmake_minimum_required(VERSION 3.15)
project(RocketSimulator)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(RocketSimulator
    src/main.cpp
    src/Atmosphere.cpp
)