# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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

# Produce verbose output by default.
VERBOSE = 1

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build

# Utility rule file for Food_autogen.

# Include any custom commands dependencies for this target.
include source/Base/CMakeFiles/Food_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include source/Base/CMakeFiles/Food_autogen.dir/progress.make

source/Base/CMakeFiles/Food_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target Food"
	cd /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/source/Base && /usr/bin/cmake -E cmake_autogen /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/source/Base/CMakeFiles/Food_autogen.dir/AutogenInfo.json ""

Food_autogen: source/Base/CMakeFiles/Food_autogen
Food_autogen: source/Base/CMakeFiles/Food_autogen.dir/build.make
.PHONY : Food_autogen

# Rule to build all files generated by this target.
source/Base/CMakeFiles/Food_autogen.dir/build: Food_autogen
.PHONY : source/Base/CMakeFiles/Food_autogen.dir/build

source/Base/CMakeFiles/Food_autogen.dir/clean:
	cd /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/source/Base && $(CMAKE_COMMAND) -P CMakeFiles/Food_autogen.dir/cmake_clean.cmake
.PHONY : source/Base/CMakeFiles/Food_autogen.dir/clean

source/Base/CMakeFiles/Food_autogen.dir/depend:
	cd /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/source/Base /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/source/Base /home/qilu/Documents/DDSA-Distributed-Deterministic-Spiral-Search-Algorithm/build/source/Base/CMakeFiles/Food_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/Base/CMakeFiles/Food_autogen.dir/depend

