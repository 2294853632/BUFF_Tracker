cmake_minimum_required(VERSION 3.5)

project(buff LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include/Eigen)


add_executable(buff main.cpp iou.cpp run.cpp buff.cpp circlefitting.cpp kalman.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

