cmake_minimum_required(VERSION 3.14)
project(SimFramework)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/Eigen)
include_directories(libs/Eigen)

# SimFramework
add_library(SimFramework STATIC src/Framework.h src/Framework.cpp)
include_directories(src)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
target_link_libraries(Sandbox SimFramework)
