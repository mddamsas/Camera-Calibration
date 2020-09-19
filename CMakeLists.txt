# Define the minimum CMake version which can execute this file
cmake_minimum_required(VERSION 2.8.12)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Give a name to your project
PROJECT(CameraCalibration)

# Find header folder and library folder for OpenCV
find_package( OpenCV REQUIRED )

# Include headers’ folders for OpenCV
include_directories( ${OpenCV_INCLUDE_DIRS})

ADD_LIBRARY(Calibration CameraCalibration.cpp CameraCalibration.h)

TARGET_LINK_LIBRARIES(Calibration ${OpenCV_LIBS})

ADD_EXECUTABLE(CameraCalibration main.cpp)

TARGET_LINK_LIBRARIES(CameraCalibration Calibration ${OpenCV_LIBS})