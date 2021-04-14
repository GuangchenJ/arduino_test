cmake_minimum_required(VERSION 3.19)
project(arduino_test)

set(CMAKE_CXX_STANDARD 20)

add_executable(arduino_test main.cpp SerialPort.cpp SerialPort.h)
#add_executable(arduino_test main.cpp)