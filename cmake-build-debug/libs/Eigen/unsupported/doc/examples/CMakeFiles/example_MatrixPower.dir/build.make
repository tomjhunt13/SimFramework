# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = "/Users/tom/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/192.6262.62/CLion.app/Contents/bin/cmake/mac/bin/cmake"

# The command to remove a file.
RM = "/Users/tom/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/192.6262.62/CLion.app/Contents/bin/cmake/mac/bin/cmake" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/tom/Documents/University/Y4_S2/SimInterface

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug

# Include any dependencies generated for this target.
include libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/depend.make

# Include the progress variables for this target.
include libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/progress.make

# Include the compile flags for this target's objects.
include libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/flags.make

libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o: libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/flags.make
libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o: ../libs/Eigen/unsupported/doc/examples/MatrixPower.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o"
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o -c /Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/doc/examples/MatrixPower.cpp

libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.i"
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/doc/examples/MatrixPower.cpp > CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.i

libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.s"
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/doc/examples/MatrixPower.cpp -o CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.s

# Object files for target example_MatrixPower
example_MatrixPower_OBJECTS = \
"CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o"

# External object files for target example_MatrixPower
example_MatrixPower_EXTERNAL_OBJECTS =

libs/Eigen/unsupported/doc/examples/example_MatrixPower: libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/MatrixPower.cpp.o
libs/Eigen/unsupported/doc/examples/example_MatrixPower: libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/build.make
libs/Eigen/unsupported/doc/examples/example_MatrixPower: libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_MatrixPower"
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_MatrixPower.dir/link.txt --verbose=$(VERBOSE)
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && ./example_MatrixPower >/Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples/MatrixPower.out

# Rule to build all files generated by this target.
libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/build: libs/Eigen/unsupported/doc/examples/example_MatrixPower

.PHONY : libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/build

libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/clean:
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples && $(CMAKE_COMMAND) -P CMakeFiles/example_MatrixPower.dir/cmake_clean.cmake
.PHONY : libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/clean

libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/depend:
	cd /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/tom/Documents/University/Y4_S2/SimInterface /Users/tom/Documents/University/Y4_S2/SimInterface/libs/Eigen/unsupported/doc/examples /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples /Users/tom/Documents/University/Y4_S2/SimInterface/cmake-build-debug/libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/Eigen/unsupported/doc/examples/CMakeFiles/example_MatrixPower.dir/depend

