cmake_minimum_required( VERSION 2.8 )
project( csv2pcd )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# pcl 
#find_package( PCL REQUIRED COMPONENT common io )
find_package( PCL 1.7 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") #use this in ubuntu 16.04
include_directories( ${PCL_INCLUDE_DIRS} )
#add_definitions( ${PCL_DEFINITIONS} )

add_executable( csv2pcd csv2pcd.cpp )
target_link_libraries( csv2pcd  ${PCL_LIBRARIES} )
