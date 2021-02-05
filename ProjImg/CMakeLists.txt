#指定CMake最小版本
cmake_minimum_required(VERSION 2.8.3)

add_definitions(-std=c++11)

#设置项目名称
project(ProjImg)

#设置包含库目录
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


#查找指定的库
find_package(OpenCV 3 REQUIRED)


#设置包含的目录//指定头文件路径
include_directories(
	/usr/include/eigen3
	./
	/usr/include/
	/usr/local/include
	${OpenCV_INCLUDE_DIRS}
)

#生成可执行文件
add_executable(ProjImg ProjImg.cpp)

#指定target需要链接的库
target_link_libraries(ProjImg 
						${OpenCV_LIBRARIES}
						)
