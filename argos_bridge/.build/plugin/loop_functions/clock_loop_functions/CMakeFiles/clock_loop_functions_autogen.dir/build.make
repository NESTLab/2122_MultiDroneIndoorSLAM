# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/src/argos_bridge/argos_bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/src/argos_bridge/argos_bridge/.build

# Utility rule file for clock_loop_functions_autogen.

# Include the progress variables for this target.
include plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/progress.make

plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/peter/src/argos_bridge/argos_bridge/.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target clock_loop_functions"
	cd /home/peter/src/argos_bridge/argos_bridge/.build/plugin/loop_functions/clock_loop_functions && /usr/bin/cmake -E cmake_autogen /home/peter/src/argos_bridge/argos_bridge/.build/plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/AutogenInfo.json ""

clock_loop_functions_autogen: plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen
clock_loop_functions_autogen: plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/build.make

.PHONY : clock_loop_functions_autogen

# Rule to build all files generated by this target.
plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/build: clock_loop_functions_autogen

.PHONY : plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/build

plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/clean:
	cd /home/peter/src/argos_bridge/argos_bridge/.build/plugin/loop_functions/clock_loop_functions && $(CMAKE_COMMAND) -P CMakeFiles/clock_loop_functions_autogen.dir/cmake_clean.cmake
.PHONY : plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/clean

plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/depend:
	cd /home/peter/src/argos_bridge/argos_bridge/.build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/src/argos_bridge/argos_bridge /home/peter/src/argos_bridge/argos_bridge/plugin/loop_functions/clock_loop_functions /home/peter/src/argos_bridge/argos_bridge/.build /home/peter/src/argos_bridge/argos_bridge/.build/plugin/loop_functions/clock_loop_functions /home/peter/src/argos_bridge/argos_bridge/.build/plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugin/loop_functions/clock_loop_functions/CMakeFiles/clock_loop_functions_autogen.dir/depend
