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
CMAKE_SOURCE_DIR = /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build

# Include any dependencies generated for this target.
include CMakeFiles/TIME_SYSTEM_CPP.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/TIME_SYSTEM_CPP.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/TIME_SYSTEM_CPP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TIME_SYSTEM_CPP.dir/flags.make

CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o: CMakeFiles/TIME_SYSTEM_CPP.dir/flags.make
CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o: /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/TimeSys.cpp
CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o: CMakeFiles/TIME_SYSTEM_CPP.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o -MF CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o.d -o CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o -c /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/TimeSys.cpp

CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/TimeSys.cpp > CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.i

CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/TimeSys.cpp -o CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.s

# Object files for target TIME_SYSTEM_CPP
TIME_SYSTEM_CPP_OBJECTS = \
"CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o"

# External object files for target TIME_SYSTEM_CPP
TIME_SYSTEM_CPP_EXTERNAL_OBJECTS =

libTIME_SYSTEM_CPP.dylib: CMakeFiles/TIME_SYSTEM_CPP.dir/TimeSys.cpp.o
libTIME_SYSTEM_CPP.dylib: CMakeFiles/TIME_SYSTEM_CPP.dir/build.make
libTIME_SYSTEM_CPP.dylib: CMakeFiles/TIME_SYSTEM_CPP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libTIME_SYSTEM_CPP.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TIME_SYSTEM_CPP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TIME_SYSTEM_CPP.dir/build: libTIME_SYSTEM_CPP.dylib
.PHONY : CMakeFiles/TIME_SYSTEM_CPP.dir/build

CMakeFiles/TIME_SYSTEM_CPP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TIME_SYSTEM_CPP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TIME_SYSTEM_CPP.dir/clean

CMakeFiles/TIME_SYSTEM_CPP.dir/depend:
	cd /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build /Users/0-0mashuo/Desktop/Clion/CombinedNavigation/lib/TimeSys/build/CMakeFiles/TIME_SYSTEM_CPP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TIME_SYSTEM_CPP.dir/depend
