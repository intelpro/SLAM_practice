cmake_minimum_required(VERSION 3.5)
project(MY_SLAM)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
SET(SOURCE_FILES
    src/main.cc
    src/SystemHandler.cc
    src/Tracker.cc
    src/Frame.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/KeyFrame.cc)

find_package(OpenCV REQUIRED)
add_executable(MY_SLAM ${SOURCE_FILES})
target_link_libraries(MY_SLAM ${OpenCV_LIBS})
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
