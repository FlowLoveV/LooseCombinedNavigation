# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.26.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.26.4/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build

# Include any dependencies generated for this target.
include CMakeFiles/Matrix.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Matrix.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Matrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Matrix.dir/flags.make

CMakeFiles/Matrix.dir/Matrix.cpp.o: CMakeFiles/Matrix.dir/flags.make
CMakeFiles/Matrix.dir/Matrix.cpp.o: /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/Matrix.cpp
CMakeFiles/Matrix.dir/Matrix.cpp.o: CMakeFiles/Matrix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Matrix.dir/Matrix.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Matrix.dir/Matrix.cpp.o -MF CMakeFiles/Matrix.dir/Matrix.cpp.o.d -o CMakeFiles/Matrix.dir/Matrix.cpp.o -c /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/Matrix.cpp

CMakeFiles/Matrix.dir/Matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Matrix.dir/Matrix.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/Matrix.cpp > CMakeFiles/Matrix.dir/Matrix.cpp.i

CMakeFiles/Matrix.dir/Matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Matrix.dir/Matrix.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/Matrix.cpp -o CMakeFiles/Matrix.dir/Matrix.cpp.s

# Object files for target Matrix
Matrix_OBJECTS = \
"CMakeFiles/Matrix.dir/Matrix.cpp.o"

# External object files for target Matrix
Matrix_EXTERNAL_OBJECTS =

libMatrix.dylib: CMakeFiles/Matrix.dir/Matrix.cpp.o
libMatrix.dylib: CMakeFiles/Matrix.dir/build.make
libMatrix.dylib: CMakeFiles/Matrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libMatrix.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Matrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Matrix.dir/build: libMatrix.dylib
.PHONY : CMakeFiles/Matrix.dir/build

CMakeFiles/Matrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Matrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Matrix.dir/clean

CMakeFiles/Matrix.dir/depend:
	cd /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/Matrix/build/CMakeFiles/Matrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Matrix.dir/depend

