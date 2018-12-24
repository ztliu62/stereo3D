cmake_minimum_required(VERSION 3.13)
project(stereo)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stereo main.cpp StereoMatch.cpp StereoMatch.h)
target_link_libraries(stereo ${OpenCV_LIBS})
