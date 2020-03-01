cmake_minimum_required(VERSION 3.14)
project(Framework)

set(CMAKE_CXX_STANDARD 14)

# eigen
add_subdirectory(ThirdParty/eigen)
include_directories(ThirdParty/eigen)

# json
add_subdirectory(ThirdParty/json)
include_directories(ThirdParty/json/single_include)

# googletest
add_subdirectory(ThirdParty/googletest)
include_directories(ThirdParty/googletest/googletest/include)

# Framework
add_library(Framework STATIC Framework/include/SimFramework/Framework.h Framework/Framework.cpp Framework/include/SimFramework/Components.h Framework/Components.cpp Framework/include/SimFramework/Interpolation.h Framework/Interpolation.cpp Framework/include/SimFramework/Utilities.h Framework/Utilities.cpp)
include_directories(Framework/include)

# Vehicle
add_library(Vehicle STATIC Vehicle/SpringDamper1D.cpp Vehicle/include/Vehicle/SpringDamper1D.h Vehicle/include/Vehicle/OutputBlock.h Vehicle/OutputBlock.cpp Vehicle/Clutch.cpp Vehicle/include/Vehicle/Clutch.h Vehicle/Vehicle.cpp Vehicle/include/Vehicle/Vehicle.h)
target_link_libraries(Vehicle Framework)
include_directories(Vehicle/include)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
target_link_libraries(Sandbox Framework Vehicle)

# Tests
add_executable(Tests Test/test_Interpolation.cpp Test/test_Integration.cpp)
target_link_libraries(Tests gtest gtest_main Framework Vehicle)