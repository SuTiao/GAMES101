cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "C:/mingw64/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/mingw64/bin/g++.exe")
project(Rasterizer)
set(OpenCV_DIR  D:/opencv/build/x64/vc16/lib/)
set(Eigen3_DIR  D:/eigen-3.4.0/Einstall/share/eigen3/cmake/)
include_directories("D:/eigen-3.4.0/Einstall/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer Eigen3::Eigen ${OpenCV_LIBRARIES})