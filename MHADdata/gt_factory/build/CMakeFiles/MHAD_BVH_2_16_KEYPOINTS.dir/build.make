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
CMAKE_SOURCE_DIR = /home/baard/master/MHADdata/gt_factory

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/baard/master/MHADdata/gt_factory/build

# Include any dependencies generated for this target.
include CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/flags.make

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/flags.make
CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o: ../src/bvh_read.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/baard/master/MHADdata/gt_factory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o -c /home/baard/master/MHADdata/gt_factory/src/bvh_read.cpp

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/baard/master/MHADdata/gt_factory/src/bvh_read.cpp > CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.i

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/baard/master/MHADdata/gt_factory/src/bvh_read.cpp -o CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.s

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.requires:

.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.requires

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.provides: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.requires
	$(MAKE) -f CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/build.make CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.provides.build
.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.provides

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.provides.build: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o


# Object files for target MHAD_BVH_2_16_KEYPOINTS
MHAD_BVH_2_16_KEYPOINTS_OBJECTS = \
"CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o"

# External object files for target MHAD_BVH_2_16_KEYPOINTS
MHAD_BVH_2_16_KEYPOINTS_EXTERNAL_OBJECTS =

MHAD_BVH_2_16_KEYPOINTS: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o
MHAD_BVH_2_16_KEYPOINTS: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/build.make
MHAD_BVH_2_16_KEYPOINTS: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/baard/master/MHADdata/gt_factory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MHAD_BVH_2_16_KEYPOINTS"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/build: MHAD_BVH_2_16_KEYPOINTS

.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/build

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/requires: CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/src/bvh_read.cpp.o.requires

.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/requires

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/clean

CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/depend:
	cd /home/baard/master/MHADdata/gt_factory/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/baard/master/MHADdata/gt_factory /home/baard/master/MHADdata/gt_factory /home/baard/master/MHADdata/gt_factory/build /home/baard/master/MHADdata/gt_factory/build /home/baard/master/MHADdata/gt_factory/build/CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MHAD_BVH_2_16_KEYPOINTS.dir/depend

