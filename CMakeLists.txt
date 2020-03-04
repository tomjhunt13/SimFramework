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

# Models
add_library(Models STATIC Models/Clutch.cpp Models/include/Models/Clutch.h Models/Vehicle.cpp Models/include/Models/Vehicle.h Models/MassSpringDamper1D.cpp Models/include/Models/MassSpringDamper1D.h)
target_link_libraries(Models Framework)
include_directories(Models/include)

# Sandbox
add_executable(Sandbox sandbox/main.cpp)
target_link_libraries(Sandbox Framework Models)

# Tests
add_executable(Tests Test/test_Interpolation.cpp Test/test_Integration.cpp)
target_link_libraries(Tests gtest gtest_main Framework Models)