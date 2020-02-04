# Install script for directory: /Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/AdolcForward"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/AlignedVector3"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/ArpackSupport"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/AutoDiff"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/BVH"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/EulerAngles"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/FFT"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/IterativeSolvers"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/KroneckerProduct"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/LevenbergMarquardt"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/MatrixFunctions"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/MoreVectorization"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/MPRealSupport"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/NonLinearOptimization"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/NumericalDiff"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/OpenGLSupport"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/Polynomials"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/Skyline"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/SparseExtra"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/SpecialFunctions"
    "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

