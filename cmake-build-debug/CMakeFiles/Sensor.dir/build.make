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
include CMakeFiles/Sensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Sensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Sensor.dir/flags.make

# Object files for target Sensor
Sensor_OBJECTS =

# External object files for target Sensor
Sensor_EXTERNAL_OBJECTS =

libSensor.so: CMakeFiles/Sensor.dir/build.make
libSensor.so: libVelocity_Filter.so
libSensor.so: Filter/libFIR_Filter.so
libSensor.so: CMakeFiles/Sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking CXX shared library libSensor.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Sensor.dir/build: libSensor.so

.PHONY : CMakeFiles/Sensor.dir/build

CMakeFiles/Sensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Sensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Sensor.dir/clean

CMakeFiles/Sensor.dir/depend:
	cd /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug /home/griv/CLionProjects/crazyflie_NatNet/cmake-build-debug/CMakeFiles/Sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Sensor.dir/depend

