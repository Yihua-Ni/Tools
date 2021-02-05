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
CMAKE_SOURCE_DIR = /home/minieye/Lidar_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/minieye/Lidar_camera/build

# Include any dependencies generated for this target.
include CMakeFiles/Dbscan.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Dbscan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Dbscan.dir/flags.make

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o: CMakeFiles/Dbscan.dir/flags.make
CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o: ../dbscan/kdtree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/minieye/Lidar_camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o -c /home/minieye/Lidar_camera/dbscan/kdtree.cpp

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/minieye/Lidar_camera/dbscan/kdtree.cpp > CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.i

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/minieye/Lidar_camera/dbscan/kdtree.cpp -o CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.s

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.requires:

.PHONY : CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.requires

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.provides: CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.requires
	$(MAKE) -f CMakeFiles/Dbscan.dir/build.make CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.provides.build
.PHONY : CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.provides

CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.provides.build: CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o


# Object files for target Dbscan
Dbscan_OBJECTS = \
"CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o"

# External object files for target Dbscan
Dbscan_EXTERNAL_OBJECTS =

libDbscan.so: CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o
libDbscan.so: CMakeFiles/Dbscan.dir/build.make
libDbscan.so: CMakeFiles/Dbscan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/minieye/Lidar_camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libDbscan.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Dbscan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Dbscan.dir/build: libDbscan.so

.PHONY : CMakeFiles/Dbscan.dir/build

CMakeFiles/Dbscan.dir/requires: CMakeFiles/Dbscan.dir/dbscan/kdtree.cpp.o.requires

.PHONY : CMakeFiles/Dbscan.dir/requires

CMakeFiles/Dbscan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Dbscan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Dbscan.dir/clean

CMakeFiles/Dbscan.dir/depend:
	cd /home/minieye/Lidar_camera/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/minieye/Lidar_camera /home/minieye/Lidar_camera /home/minieye/Lidar_camera/build /home/minieye/Lidar_camera/build /home/minieye/Lidar_camera/build/CMakeFiles/Dbscan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Dbscan.dir/depend

