# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/minieye/桌面/ICP/ICPv5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/minieye/桌面/ICP/ICPv5/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/Test/test.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/Test/test.cpp.o: ../Test/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/minieye/桌面/ICP/ICPv5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/Test/test.cpp.o"
	/usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Test/test.cpp.o -c /home/minieye/桌面/ICP/ICPv5/Test/test.cpp

CMakeFiles/test.dir/Test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Test/test.cpp.i"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/minieye/桌面/ICP/ICPv5/Test/test.cpp > CMakeFiles/test.dir/Test/test.cpp.i

CMakeFiles/test.dir/Test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Test/test.cpp.s"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/minieye/桌面/ICP/ICPv5/Test/test.cpp -o CMakeFiles/test.dir/Test/test.cpp.s

CMakeFiles/test.dir/Test/test.cpp.o.requires:

.PHONY : CMakeFiles/test.dir/Test/test.cpp.o.requires

CMakeFiles/test.dir/Test/test.cpp.o.provides: CMakeFiles/test.dir/Test/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/Test/test.cpp.o.provides.build
.PHONY : CMakeFiles/test.dir/Test/test.cpp.o.provides

CMakeFiles/test.dir/Test/test.cpp.o.provides.build: CMakeFiles/test.dir/Test/test.cpp.o


# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/Test/test.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/Test/test.cpp.o
test: CMakeFiles/test.dir/build.make
test: libICP.a
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/minieye/桌面/ICP/ICPv5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/requires: CMakeFiles/test.dir/Test/test.cpp.o.requires

.PHONY : CMakeFiles/test.dir/requires

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/minieye/桌面/ICP/ICPv5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/minieye/桌面/ICP/ICPv5 /home/minieye/桌面/ICP/ICPv5 /home/minieye/桌面/ICP/ICPv5/build /home/minieye/桌面/ICP/ICPv5/build /home/minieye/桌面/ICP/ICPv5/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

