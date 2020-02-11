cmake_minimum_required(VERSION 3.14)
project(SimInterface)

set(CMAKE_CXX_STANDARD 14)

# Eigen
add_subdirectory(libs/Eigen)
include_directories(libs/Eigen)

# SimInterface
add_library(SimInterface STATIC src/DynamicSystem.h src/Signal.h src/Block.h src/Sink.cpp src/Sink.h src/Block.cpp src/SystemManager.cpp src/SystemManager.h src/Utilities.h src/Utilities.cpp src/ForwardEuler.cpp src/ForwardEuler.h src/ConstantBlock.h)
include_directories(src)

# Sandbox
add_executable(Sandbox sandbox/main.cpp sandbox/Mass1D.cpp sandbox/Mass1D.h sandbox/SpringDamper1D.cpp sandbox/SpringDamper1D.h sandbox/OutputBlock.cpp sandbox/OutputBlock.h)
target_link_libraries(Sandbox SimInterface)
#include_directories(include)

# TestUE4Link
add_library(TestUE4Link sandbox/TestUE4Link.cpp sandbox/TestUE4Link.h)