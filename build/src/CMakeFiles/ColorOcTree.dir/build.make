# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/huo/Documents/mapping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huo/Documents/mapping/build

# Include any dependencies generated for this target.
include src/CMakeFiles/ColorOcTree.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ColorOcTree.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/ColorOcTree.dir/flags.make

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o: src/CMakeFiles/ColorOcTree.dir/flags.make
src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o: ../src/ColorOcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/huo/Documents/mapping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o"
	cd /home/huo/Documents/mapping/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o -c /home/huo/Documents/mapping/src/ColorOcTree.cpp

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.i"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/huo/Documents/mapping/src/ColorOcTree.cpp > CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.i

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.s"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/huo/Documents/mapping/src/ColorOcTree.cpp -o CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.s

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.requires:
.PHONY : src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.requires

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.provides: src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/ColorOcTree.dir/build.make src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.provides.build
.PHONY : src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.provides

src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.provides.build: src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o

# Object files for target ColorOcTree
ColorOcTree_OBJECTS = \
"CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o"

# External object files for target ColorOcTree
ColorOcTree_EXTERNAL_OBJECTS =

../bin/ColorOcTree: src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o
../bin/ColorOcTree: src/CMakeFiles/ColorOcTree.dir/build.make
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/ColorOcTree: /usr/lib/libpcl_common.so
../bin/ColorOcTree: /usr/lib/libpcl_octree.so
../bin/ColorOcTree: /usr/lib/libOpenNI.so
../bin/ColorOcTree: /usr/lib/libOpenNI2.so
../bin/ColorOcTree: /usr/lib/libvtkCommon.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkImaging.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkIO.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkParallel.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGeovis.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkViews.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkCharts.so.5.8.0
../bin/ColorOcTree: /usr/lib/libpcl_io.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/ColorOcTree: /usr/lib/libpcl_kdtree.so
../bin/ColorOcTree: /usr/lib/libpcl_search.so
../bin/ColorOcTree: /usr/lib/libpcl_sample_consensus.so
../bin/ColorOcTree: /usr/lib/libpcl_filters.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/ColorOcTree: /usr/lib/libOpenNI.so
../bin/ColorOcTree: /usr/lib/libOpenNI2.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/ColorOcTree: /usr/lib/libvtkCommon.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkImaging.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkIO.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkParallel.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGeovis.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkViews.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkCharts.so.5.8.0
../bin/ColorOcTree: /usr/local/lib/liboctomap.so
../bin/ColorOcTree: /usr/local/lib/liboctomath.so
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/ColorOcTree: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/ColorOcTree: /usr/lib/libpcl_common.so
../bin/ColorOcTree: /usr/lib/libpcl_octree.so
../bin/ColorOcTree: /usr/lib/libpcl_io.so
../bin/ColorOcTree: /usr/lib/libpcl_kdtree.so
../bin/ColorOcTree: /usr/lib/libpcl_search.so
../bin/ColorOcTree: /usr/lib/libpcl_sample_consensus.so
../bin/ColorOcTree: /usr/lib/libpcl_filters.so
../bin/ColorOcTree: /usr/local/lib/liboctomap.so
../bin/ColorOcTree: /usr/local/lib/liboctomath.so
../bin/ColorOcTree: /usr/lib/libvtkViews.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkParallel.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkRendering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkImaging.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkIO.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtkCommon.so.5.8.0
../bin/ColorOcTree: /usr/lib/libvtksys.so.5.8.0
../bin/ColorOcTree: src/CMakeFiles/ColorOcTree.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/ColorOcTree"
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ColorOcTree.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/ColorOcTree.dir/build: ../bin/ColorOcTree
.PHONY : src/CMakeFiles/ColorOcTree.dir/build

src/CMakeFiles/ColorOcTree.dir/requires: src/CMakeFiles/ColorOcTree.dir/ColorOcTree.cpp.o.requires
.PHONY : src/CMakeFiles/ColorOcTree.dir/requires

src/CMakeFiles/ColorOcTree.dir/clean:
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -P CMakeFiles/ColorOcTree.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ColorOcTree.dir/clean

src/CMakeFiles/ColorOcTree.dir/depend:
	cd /home/huo/Documents/mapping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huo/Documents/mapping /home/huo/Documents/mapping/src /home/huo/Documents/mapping/build /home/huo/Documents/mapping/build/src /home/huo/Documents/mapping/build/src/CMakeFiles/ColorOcTree.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ColorOcTree.dir/depend

