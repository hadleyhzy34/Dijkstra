# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/hadley/Developments/Dijkstra

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/hadley/Developments/Dijkstra/build

# Include any dependencies generated for this target.
include CMakeFiles/plan.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/plan.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/plan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plan.dir/flags.make

CMakeFiles/plan.dir/src/Dijsktra.cpp.o: CMakeFiles/plan.dir/flags.make
CMakeFiles/plan.dir/src/Dijsktra.cpp.o: /Users/hadley/Developments/Dijkstra/src/Dijsktra.cpp
CMakeFiles/plan.dir/src/Dijsktra.cpp.o: CMakeFiles/plan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/hadley/Developments/Dijkstra/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/plan.dir/src/Dijsktra.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/plan.dir/src/Dijsktra.cpp.o -MF CMakeFiles/plan.dir/src/Dijsktra.cpp.o.d -o CMakeFiles/plan.dir/src/Dijsktra.cpp.o -c /Users/hadley/Developments/Dijkstra/src/Dijsktra.cpp

CMakeFiles/plan.dir/src/Dijsktra.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/plan.dir/src/Dijsktra.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/hadley/Developments/Dijkstra/src/Dijsktra.cpp > CMakeFiles/plan.dir/src/Dijsktra.cpp.i

CMakeFiles/plan.dir/src/Dijsktra.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/plan.dir/src/Dijsktra.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/hadley/Developments/Dijkstra/src/Dijsktra.cpp -o CMakeFiles/plan.dir/src/Dijsktra.cpp.s

# Object files for target plan
plan_OBJECTS = \
"CMakeFiles/plan.dir/src/Dijsktra.cpp.o"

# External object files for target plan
plan_EXTERNAL_OBJECTS =

libplan.a: CMakeFiles/plan.dir/src/Dijsktra.cpp.o
libplan.a: CMakeFiles/plan.dir/build.make
libplan.a: CMakeFiles/plan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/hadley/Developments/Dijkstra/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libplan.a"
	$(CMAKE_COMMAND) -P CMakeFiles/plan.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plan.dir/build: libplan.a
.PHONY : CMakeFiles/plan.dir/build

CMakeFiles/plan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plan.dir/clean

CMakeFiles/plan.dir/depend:
	cd /Users/hadley/Developments/Dijkstra/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/hadley/Developments/Dijkstra /Users/hadley/Developments/Dijkstra /Users/hadley/Developments/Dijkstra/build /Users/hadley/Developments/Dijkstra/build /Users/hadley/Developments/Dijkstra/build/CMakeFiles/plan.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/plan.dir/depend

