# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /snap/clion/135/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/135/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/griv/CLionProjects/crazyflie_NatNet

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug

# Include any dependencies generated for this target.
include Filter/CMakeFiles/Filter_test.dir/depend.make

# Include the progress variables for this target.
include Filter/CMakeFiles/Filter_test.dir/progress.make

# Include the compile flags for this target's objects.
include Filter/CMakeFiles/Filter_test.dir/flags.make

Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o: Filter/CMakeFiles/Filter_test.dir/flags.make
Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o: ../Filter/test/Filter_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o -c /home/griv/CLionProjects/crazyflie_NatNet/Filter/test/Filter_test.cpp

Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Filter_test.dir/test/Filter_test.cpp.i"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/griv/CLionProjects/crazyflie_NatNet/Filter/test/Filter_test.cpp > CMakeFiles/Filter_test.dir/test/Filter_test.cpp.i

Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Filter_test.dir/test/Filter_test.cpp.s"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/griv/CLionProjects/crazyflie_NatNet/Filter/test/Filter_test.cpp -o CMakeFiles/Filter_test.dir/test/Filter_test.cpp.s

# Object files for target Filter_test
Filter_test_OBJECTS = \
"CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o"

# External object files for target Filter_test
Filter_test_EXTERNAL_OBJECTS =

Filter/Filter_test: Filter/CMakeFiles/Filter_test.dir/test/Filter_test.cpp.o
Filter/Filter_test: Filter/CMakeFiles/Filter_test.dir/build.make
Filter/Filter_test: Filter/libFIR_Filter.so
Filter/Filter_test: Filter/CMakeFiles/Filter_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Filter_test"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Filter_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Filter/CMakeFiles/Filter_test.dir/build: Filter/Filter_test

.PHONY : Filter/CMakeFiles/Filter_test.dir/build

Filter/CMakeFiles/Filter_test.dir/clean:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter && $(CMAKE_COMMAND) -P CMakeFiles/Filter_test.dir/cmake_clean.cmake
.PHONY : Filter/CMakeFiles/Filter_test.dir/clean

Filter/CMakeFiles/Filter_test.dir/depend:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet/Filter /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Filter/CMakeFiles/Filter_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Filter/CMakeFiles/Filter_test.dir/depend

