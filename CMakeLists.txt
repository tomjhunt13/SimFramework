cmake_minimum_required(VERSION 3.14)
project(SimInterface)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/Eigen)
include_directories(libs/Eigen)

# SimInterface
add_library(SimInterface src/StateSpace.cpp src/StateSpace.h src/RK4.h src/RK4.cpp src/DynamicSystem.h src/Signal.h src/Block.h src/DynamicSystem.cpp src/Sink.cpp src/Sink.h src/TimeManager.cpp src/TimeManager.h src/Block.cpp)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
target_link_libraries(Sandbox SimInterface)