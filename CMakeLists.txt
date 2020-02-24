cmake_minimum_required(VERSION 3.14)
project(SimFramework)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/Eigen)
include_directories(libs/Eigen)

# json
add_subdirectory(libs/json)
include_directories(libs/json/single_include)

# SimFramework
add_library(SimFramework STATIC src/Framework.h src/Framework.cpp src/Components.h src/Components.cpp src/Interpolation.cpp src/Interpolation.h)
include_directories(src)

# Sandbox
add_executable(Sandbox sandbox/main.cpp sandbox/SpringDamper1D.cpp sandbox/SpringDamper1D.h sandbox/Mass1D.cpp sandbox/Mass1D.h sandbox/OutputBlock.h sandbox/OutputBlock.cpp sandbox/Engine.cpp sandbox/Engine.h)
target_link_libraries(Sandbox SimFramework)
