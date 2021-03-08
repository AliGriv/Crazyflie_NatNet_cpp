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
include CMakeFiles/scrap.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/scrap.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scrap.dir/flags.make

CMakeFiles/scrap.dir/src/scrap.cpp.o: CMakeFiles/scrap.dir/flags.make
CMakeFiles/scrap.dir/src/scrap.cpp.o: ../src/scrap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/scrap.dir/src/scrap.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scrap.dir/src/scrap.cpp.o -c /home/griv/CLionProjects/crazyflie_NatNet/src/scrap.cpp

CMakeFiles/scrap.dir/src/scrap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scrap.dir/src/scrap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/griv/CLionProjects/crazyflie_NatNet/src/scrap.cpp > CMakeFiles/scrap.dir/src/scrap.cpp.i

CMakeFiles/scrap.dir/src/scrap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scrap.dir/src/scrap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/griv/CLionProjects/crazyflie_NatNet/src/scrap.cpp -o CMakeFiles/scrap.dir/src/scrap.cpp.s

# Object files for target scrap
scrap_OBJECTS = \
"CMakeFiles/scrap.dir/src/scrap.cpp.o"

# External object files for target scrap
scrap_EXTERNAL_OBJECTS =

scrap: CMakeFiles/scrap.dir/src/scrap.cpp.o
scrap: CMakeFiles/scrap.dir/build.make
scrap: Crazyflie/crazyflie_cpp/libcrazyflie_cpp.a
scrap: libVelocity_Filter.so
scrap: libPB_Control.so
scrap: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
scrap: Filter/libFIR_Filter.so
scrap: CMakeFiles/scrap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scrap"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scrap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scrap.dir/build: scrap

.PHONY : CMakeFiles/scrap.dir/build

CMakeFiles/scrap.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scrap.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scrap.dir/clean

CMakeFiles/scrap.dir/depend:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles/scrap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scrap.dir/depend
