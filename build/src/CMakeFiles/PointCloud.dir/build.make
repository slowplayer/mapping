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
include src/CMakeFiles/PointCloud.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/PointCloud.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/PointCloud.dir/flags.make

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o: src/CMakeFiles/PointCloud.dir/flags.make
src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o: ../src/PointCloud.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/huo/Documents/mapping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o"
	cd /home/huo/Documents/mapping/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/PointCloud.dir/PointCloud.cpp.o -c /home/huo/Documents/mapping/src/PointCloud.cpp

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PointCloud.dir/PointCloud.cpp.i"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/huo/Documents/mapping/src/PointCloud.cpp > CMakeFiles/PointCloud.dir/PointCloud.cpp.i

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PointCloud.dir/PointCloud.cpp.s"
	cd /home/huo/Documents/mapping/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/huo/Documents/mapping/src/PointCloud.cpp -o CMakeFiles/PointCloud.dir/PointCloud.cpp.s

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.requires:
.PHONY : src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.requires

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.provides: src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/PointCloud.dir/build.make src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.provides.build
.PHONY : src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.provides

src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.provides.build: src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o

# Object files for target PointCloud
PointCloud_OBJECTS = \
"CMakeFiles/PointCloud.dir/PointCloud.cpp.o"

# External object files for target PointCloud
PointCloud_EXTERNAL_OBJECTS =

../bin/PointCloud: src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o
../bin/PointCloud: src/CMakeFiles/PointCloud.dir/build.make
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/PointCloud: /usr/lib/libpcl_common.so
../bin/PointCloud: /usr/lib/libpcl_octree.so
../bin/PointCloud: /usr/lib/libOpenNI.so
../bin/PointCloud: /usr/lib/libOpenNI2.so
../bin/PointCloud: /usr/lib/libvtkCommon.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkFiltering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkImaging.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGraphics.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkIO.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkHybrid.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkWidgets.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkParallel.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkInfovis.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGeovis.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkViews.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkCharts.so.5.8.0
../bin/PointCloud: /usr/lib/libpcl_io.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/PointCloud: /usr/lib/libpcl_kdtree.so
../bin/PointCloud: /usr/lib/libpcl_search.so
../bin/PointCloud: /usr/lib/libpcl_sample_consensus.so
../bin/PointCloud: /usr/lib/libpcl_filters.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/PointCloud: /usr/lib/libOpenNI.so
../bin/PointCloud: /usr/lib/libOpenNI2.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/PointCloud: /usr/lib/libvtkCommon.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkFiltering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkImaging.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGraphics.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkIO.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkHybrid.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkWidgets.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkParallel.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkInfovis.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGeovis.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkViews.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkCharts.so.5.8.0
../bin/PointCloud: /usr/local/lib/liboctomap.so
../bin/PointCloud: /usr/local/lib/liboctomath.so
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/PointCloud: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/PointCloud: /usr/lib/libpcl_common.so
../bin/PointCloud: /usr/lib/libpcl_octree.so
../bin/PointCloud: /usr/lib/libpcl_io.so
../bin/PointCloud: /usr/lib/libpcl_kdtree.so
../bin/PointCloud: /usr/lib/libpcl_search.so
../bin/PointCloud: /usr/lib/libpcl_sample_consensus.so
../bin/PointCloud: /usr/lib/libpcl_filters.so
../bin/PointCloud: /usr/local/lib/liboctomap.so
../bin/PointCloud: /usr/local/lib/liboctomath.so
../bin/PointCloud: /usr/lib/libvtkViews.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkInfovis.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkWidgets.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkHybrid.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkParallel.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkRendering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkImaging.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkGraphics.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkIO.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkFiltering.so.5.8.0
../bin/PointCloud: /usr/lib/libvtkCommon.so.5.8.0
../bin/PointCloud: /usr/lib/libvtksys.so.5.8.0
../bin/PointCloud: src/CMakeFiles/PointCloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/PointCloud"
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PointCloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/PointCloud.dir/build: ../bin/PointCloud
.PHONY : src/CMakeFiles/PointCloud.dir/build

src/CMakeFiles/PointCloud.dir/requires: src/CMakeFiles/PointCloud.dir/PointCloud.cpp.o.requires
.PHONY : src/CMakeFiles/PointCloud.dir/requires

src/CMakeFiles/PointCloud.dir/clean:
	cd /home/huo/Documents/mapping/build/src && $(CMAKE_COMMAND) -P CMakeFiles/PointCloud.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/PointCloud.dir/clean

src/CMakeFiles/PointCloud.dir/depend:
	cd /home/huo/Documents/mapping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huo/Documents/mapping /home/huo/Documents/mapping/src /home/huo/Documents/mapping/build /home/huo/Documents/mapping/build/src /home/huo/Documents/mapping/build/src/CMakeFiles/PointCloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/PointCloud.dir/depend

