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
include Crazyflie/CMakeFiles/listLogVariables.dir/depend.make

# Include the progress variables for this target.
include Crazyflie/CMakeFiles/listLogVariables.dir/progress.make

# Include the compile flags for this target's objects.
include Crazyflie/CMakeFiles/listLogVariables.dir/flags.make

Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o: Crazyflie/CMakeFiles/listLogVariables.dir/flags.make
Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o: ../Crazyflie/src/listLogVariables.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o -c /home/griv/CLionProjects/crazyflie_NatNet/Crazyflie/src/listLogVariables.cpp

Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.i"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/griv/CLionProjects/crazyflie_NatNet/Crazyflie/src/listLogVariables.cpp > CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.i

Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.s"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/griv/CLionProjects/crazyflie_NatNet/Crazyflie/src/listLogVariables.cpp -o CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.s

# Object files for target listLogVariables
listLogVariables_OBJECTS = \
"CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o"

# External object files for target listLogVariables
listLogVariables_EXTERNAL_OBJECTS =

Crazyflie/listLogVariables: Crazyflie/CMakeFiles/listLogVariables.dir/src/listLogVariables.cpp.o
Crazyflie/listLogVariables: Crazyflie/CMakeFiles/listLogVariables.dir/build.make
Crazyflie/listLogVariables: Crazyflie/crazyflie_cpp/libcrazyflie_cpp.a
Crazyflie/listLogVariables: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
Crazyflie/listLogVariables: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
Crazyflie/listLogVariables: Crazyflie/CMakeFiles/listLogVariables.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable listLogVariables"
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/listLogVariables.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Crazyflie/CMakeFiles/listLogVariables.dir/build: Crazyflie/listLogVariables

.PHONY : Crazyflie/CMakeFiles/listLogVariables.dir/build

Crazyflie/CMakeFiles/listLogVariables.dir/clean:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie && $(CMAKE_COMMAND) -P CMakeFiles/listLogVariables.dir/cmake_clean.cmake
.PHONY : Crazyflie/CMakeFiles/listLogVariables.dir/clean

Crazyflie/CMakeFiles/listLogVariables.dir/depend:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet/Crazyflie /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/Crazyflie/CMakeFiles/listLogVariables.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Crazyflie/CMakeFiles/listLogVariables.dir/depend

