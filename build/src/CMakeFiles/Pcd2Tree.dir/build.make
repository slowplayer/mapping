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
include src/CMakeFiles/Pcd2Tree.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/Pcd2Tree.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/Pcd2Tree.dir/flags.make

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o: src/CMakeFiles/Pcd2Tree.dir/flags.make
src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o: ../src/pcd2tree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/huo/Documents/mapping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o"
	cd /home/huo/Documents/mapping/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o -c /home/huo/Documents/mapping/src/pcd2tree.cpp

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.i"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/huo/Documents/mapping/src/pcd2tree.cpp > CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.i

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.s"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/huo/Documents/mapping/src/pcd2tree.cpp -o CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.s

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.requires:
.PHONY : src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.requires

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.provides: src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/Pcd2Tree.dir/build.make src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.provides.build
.PHONY : src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.provides

src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.provides.build: src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o

# Object files for target Pcd2Tree
Pcd2Tree_OBJECTS = \
"CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o"

# External object files for target Pcd2Tree
Pcd2Tree_EXTERNAL_OBJECTS =

../bin/Pcd2Tree: src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o
../bin/Pcd2Tree: src/CMakeFiles/Pcd2Tree.dir/build.make
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/Pcd2Tree: /usr/lib/libpcl_common.so
../bin/Pcd2Tree: /usr/lib/libpcl_octree.so
../bin/Pcd2Tree: /usr/lib/libOpenNI.so
../bin/Pcd2Tree: /usr/lib/libOpenNI2.so
../bin/Pcd2Tree: /usr/lib/libvtkCommon.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkImaging.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkIO.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkParallel.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGeovis.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkViews.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkCharts.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libpcl_io.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/Pcd2Tree: /usr/lib/libpcl_kdtree.so
../bin/Pcd2Tree: /usr/lib/libpcl_search.so
../bin/Pcd2Tree: /usr/lib/libpcl_sample_consensus.so
../bin/Pcd2Tree: /usr/lib/libpcl_filters.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/Pcd2Tree: /usr/lib/libOpenNI.so
../bin/Pcd2Tree: /usr/lib/libOpenNI2.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/Pcd2Tree: /usr/lib/libvtkCommon.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkImaging.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkIO.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkParallel.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGeovis.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkViews.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkCharts.so.5.8.0
../bin/Pcd2Tree: /usr/local/lib/liboctomap.so
../bin/Pcd2Tree: /usr/local/lib/liboctomath.so
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/Pcd2Tree: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/Pcd2Tree: /usr/lib/libpcl_common.so
../bin/Pcd2Tree: /usr/lib/libpcl_octree.so
../bin/Pcd2Tree: /usr/lib/libpcl_io.so
../bin/Pcd2Tree: /usr/lib/libpcl_kdtree.so
../bin/Pcd2Tree: /usr/lib/libpcl_search.so
../bin/Pcd2Tree: /usr/lib/libpcl_sample_consensus.so
../bin/Pcd2Tree: /usr/lib/libpcl_filters.so
../bin/Pcd2Tree: /usr/local/lib/liboctomap.so
../bin/Pcd2Tree: /usr/local/lib/liboctomath.so
../bin/Pcd2Tree: /usr/lib/libvtkViews.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkInfovis.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkWidgets.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkHybrid.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkParallel.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkRendering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkImaging.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkGraphics.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkIO.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkFiltering.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtkCommon.so.5.8.0
../bin/Pcd2Tree: /usr/lib/libvtksys.so.5.8.0
../bin/Pcd2Tree: src/CMakeFiles/Pcd2Tree.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/Pcd2Tree"
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Pcd2Tree.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/Pcd2Tree.dir/build: ../bin/Pcd2Tree
.PHONY : src/CMakeFiles/Pcd2Tree.dir/build

src/CMakeFiles/Pcd2Tree.dir/requires: src/CMakeFiles/Pcd2Tree.dir/pcd2tree.cpp.o.requires
.PHONY : src/CMakeFiles/Pcd2Tree.dir/requires

src/CMakeFiles/Pcd2Tree.dir/clean:
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -P CMakeFiles/Pcd2Tree.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/Pcd2Tree.dir/clean

src/CMakeFiles/Pcd2Tree.dir/depend:
	cd /home/huo/Documents/mapping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huo/Documents/mapping /home/huo/Documents/mapping/src /home/huo/Documents/mapping/build /home/huo/Documents/mapping/build/src /home/huo/Documents/mapping/build/src/CMakeFiles/Pcd2Tree.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/Pcd2Tree.dir/depend
