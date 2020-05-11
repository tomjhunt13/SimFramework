cmake_minimum_required(VERSION 3.14)
project(Framework)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)


message(${CMAKE_HOST_SYSTEM_PROCESSOR})

# eigen
message("Configuring Eigen")
add_subdirectory(ThirdParty/eigen)
include_directories(ThirdParty/eigen)

# json
message("Configuring JSON")
add_subdirectory(ThirdParty/json)
include_directories(ThirdParty/json/single_include)

# googletest
message("Configuring Google Test")
add_subdirectory(ThirdParty/googletest)
include_directories(ThirdParty/googletest/googletest/include)

# Framework
add_library(Framework STATIC Framework/include/SimFramework/Framework.h Framework/Framework.cpp Framework/include/SimFramework/Components.h Framework/Components.cpp Framework/include/SimFramework/Interpolation.h Framework/Interpolation.cpp Framework/include/SimFramework/Utilities.h Framework/Utilities.cpp)
include_directories(Framework/include)


# SimModels
add_library(SimModels STATIC SimModels/MassSpringDamper1D.cpp SimModels/include/SimModels/MassSpringDamper1D.h SimModels/Vehicle.cpp SimModels/include/SimModels/Vehicle.h SimModels/VehicleComponents.cpp SimModels/include/SimModels/VehicleComponents.h SimModels/Road.cpp SimModels/include/SimModels/Road.h SimModels/Transmission.cpp SimModels/include/SimModels/Transmission.h SimModels/VehicleController.cpp SimModels/include/SimModels/VehicleController.h SimModels/VehicleDynamics.cpp SimModels/include/SimModels/VehicleDynamics.h SimModels/Engine.cpp SimModels/include/SimModels/Engine.h SimModels/Powertrain.cpp SimModels/include/SimModels/Powertrain.h)
target_link_libraries(SimModels Framework)
include_directories(SimModels/include)


# Sandbox
add_executable(Sandbox Sandbox/main.cpp Sandbox/SandboxMassSpringDamper.h Sandbox/SandboxFullModel.h Sandbox/SandboxLockupClutch.h Sandbox/ReportResults_Mass.h Sandbox/ReportResults_Aero.h Sandbox/ReportResults_TransmissionAndClutch.h Sandbox/ReportResults_VehicleDynamics.h)
target_link_libraries(Sandbox Framework SimModels)


# Tests
add_executable(Tests Test/test_Interpolation.cpp Test/test_Integration.cpp Test/test_Components.cpp Test/test_Framework.cpp Test/test_Vehicle.cpp)
target_link_libraries(Tests gtest gtest_main Framework SimModels)


# Copy lib files
add_custom_command(TARGET Framework POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy
        "$<TARGET_FILE:Framework>"
        "${CMAKE_SOURCE_DIR}/Binaries/${CMAKE_HOST_SYSTEM_NAME}/${CMAKE_HOST_SYSTEM_PROCESSOR}/$<TARGET_FILE_NAME:Framework>"
        COMMAND ${CMAKE_COMMAND} -E echo
        "Copying $<TARGET_FILE:Framework> to ${CMAKE_SOURCE_DIR}/Binaries/${CMAKE_HOST_SYSTEM_NAME}/${CMAKE_HOST_SYSTEM_PROCESSOR}/$<TARGET_FILE_NAME:Framework>")

add_custom_command(TARGET SimModels POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy
        "$<TARGET_FILE:SimModels>"
        "${CMAKE_SOURCE_DIR}/Binaries/${CMAKE_HOST_SYSTEM_NAME}/${CMAKE_HOST_SYSTEM_PROCESSOR}/$<TARGET_FILE_NAME:SimModels>"
        COMMAND ${CMAKE_COMMAND} -E echo
        "Copying $<TARGET_FILE:SimModels> to ${CMAKE_SOURCE_DIR}/Binaries/${CMAKE_HOST_SYSTEM_NAME}/${CMAKE_HOST_SYSTEM_PROCESSOR}/$<TARGET_FILE_NAME:SimModels>")