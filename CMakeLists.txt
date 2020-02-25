cmake_minimum_required(VERSION 3.14)
project(Framework)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/eigen)
include_directories(libs/eigen)

# json
add_subdirectory(libs/json)
include_directories(libs/json/single_include)

# Framework
add_library(Framework STATIC Framework/Framework.h Framework/Framework.cpp Framework/Components.h Framework/Components.cpp Framework/Interpolation.h)
include_directories(Framework)

# Vehicle
add_library(Vehicle STATIC Vehicle/SpringDamper1D.cpp Vehicle/SpringDamper1D.h Vehicle/Mass1D.cpp Vehicle/Mass1D.h Vehicle/OutputBlock.h Vehicle/OutputBlock.cpp Vehicle/Engine.cpp Vehicle/Engine.h Vehicle/Clutch.cpp Vehicle/Clutch.h Vehicle/Vehicle.cpp Vehicle/Vehicle.h)
target_link_libraries(Vehicle Framework)
include_directories(Vehicle)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
target_link_libraries(Sandbox Framework Vehicle)
