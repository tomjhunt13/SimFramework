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

# SimModels
add_library(SimModels STATIC SimModels/MassSpringDamper1D.cpp SimModels/include/SimModels/MassSpringDamper1D.h SimModels/EngineStandalone.cpp SimModels/include/SimModels/EngineStandalone.h SimModels/TransmissionStandalone.cpp SimModels/include/SimModels/TransmissionStandalone.h SimModels/Vehicle.cpp SimModels/include/SimModels/Vehicle.h SimModels/VehicleComponents.cpp SimModels/include/SimModels/VehicleComponents.h)
target_link_libraries(SimModels Framework)
include_directories(SimModels/include)

# Sandbox
add_executable(Sandbox Sandbox/main.cpp Sandbox/SandboxTransmission.h Sandbox/SandboxMassSpringDamper.h Sandbox/SandboxEngine.h)
target_link_libraries(Sandbox Framework SimModels)

# Tests
add_executable(Tests Test/test_Interpolation.cpp Test/test_Integration.cpp Test/test_Components.cpp Test/test_Framework.cpp Test/test_Vehicle.cpp)
target_link_libraries(Tests gtest gtest_main Framework SimModels)