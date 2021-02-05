cmake_minimum_required(VERSION 3.2)
project(lane)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -lpthread  -fopenmp")

find_package(OpenCV 3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(PCL 1.7 REQUIRED)

include_directories(
        ThirdParty/eigen3
        ThirdParty/sophus
		/usr/local/include
		/usr/include
		./
		${Pangolin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
#		${PCL_INCLUDE_DIRS}
)

aux_source_directory(tools TOOL_SRC)
add_library(Tools SHARED ${TOOL_SRC})

aux_source_directory(pcap RAW_SRC)
add_library(RAW SHARED ${RAW_SRC})

aux_source_directory(viewer VIEW_SRC)
add_library(Viewer SHARED ${VIEW_SRC})

#aux_source_directory(dbscan DBSCAN_SRC)
#add_library(Dbscan SHARED ${DBSCAN_SRC})

aux_source_directory(icp ICP_SRC)
add_library(ICP SHARED ${ICP_SRC})

add_executable(t test/main.cpp)
target_link_libraries(t
					  ${OpenCV_LIBRARIES}
					  ${Pangolin_LIBRARIES}
					  ${OPENGL_gl_LIBRARY}
#					  ${PCL_LIBRARIES}
		                          RAW
					  Viewer
					  Tools
#					  Dbscan
					  ICP
					  pcap
					  )



