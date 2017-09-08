cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# octomap 
find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )



add_executable( OcTree OcTree.cpp )
target_link_libraries( OcTree ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable( PointCloud PointCloud.cpp )
target_link_libraries( PointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable( ColorOcTree ColorOcTree.cpp )
target_link_libraries( ColorOcTree ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(Pcd2Tree pcd2tree.cpp)
target_link_libraries( Pcd2Tree ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )
