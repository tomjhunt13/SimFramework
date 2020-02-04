cmake_minimum_required(VERSION 3.14)
project(SimInterface)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/Eigen)
include_directories(libs/Eigen)

# SimInterface
add_library(SimInterface src/library.cpp src/library.h src/StateSpace.cpp src/StateSpace.h)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
#target_include_directories(Sandbox src)
target_link_libraries(Sandbox SimInterface)