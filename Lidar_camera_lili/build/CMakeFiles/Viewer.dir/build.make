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
include CMakeFiles/Viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Viewer.dir/flags.make

CMakeFiles/Viewer.dir/viewer/viewer.cpp.o: CMakeFiles/Viewer.dir/flags.make
CMakeFiles/Viewer.dir/viewer/viewer.cpp.o: ../viewer/viewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/minieye/Lidar_camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Viewer.dir/viewer/viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Viewer.dir/viewer/viewer.cpp.o -c /home/minieye/Lidar_camera/viewer/viewer.cpp

CMakeFiles/Viewer.dir/viewer/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Viewer.dir/viewer/viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/minieye/Lidar_camera/viewer/viewer.cpp > CMakeFiles/Viewer.dir/viewer/viewer.cpp.i

CMakeFiles/Viewer.dir/viewer/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Viewer.dir/viewer/viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/minieye/Lidar_camera/viewer/viewer.cpp -o CMakeFiles/Viewer.dir/viewer/viewer.cpp.s

CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.requires:

.PHONY : CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.requires

CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.provides: CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Viewer.dir/build.make CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.provides.build
.PHONY : CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.provides

CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.provides.build: CMakeFiles/Viewer.dir/viewer/viewer.cpp.o


# Object files for target Viewer
Viewer_OBJECTS = \
"CMakeFiles/Viewer.dir/viewer/viewer.cpp.o"

# External object files for target Viewer
Viewer_EXTERNAL_OBJECTS =

libViewer.so: CMakeFiles/Viewer.dir/viewer/viewer.cpp.o
libViewer.so: CMakeFiles/Viewer.dir/build.make
libViewer.so: CMakeFiles/Viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/minieye/Lidar_camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libViewer.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Viewer.dir/build: libViewer.so

.PHONY : CMakeFiles/Viewer.dir/build

CMakeFiles/Viewer.dir/requires: CMakeFiles/Viewer.dir/viewer/viewer.cpp.o.requires

.PHONY : CMakeFiles/Viewer.dir/requires

CMakeFiles/Viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Viewer.dir/clean

CMakeFiles/Viewer.dir/depend:
	cd /home/minieye/Lidar_camera/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/minieye/Lidar_camera /home/minieye/Lidar_camera /home/minieye/Lidar_camera/build /home/minieye/Lidar_camera/build /home/minieye/Lidar_camera/build/CMakeFiles/Viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Viewer.dir/depend
