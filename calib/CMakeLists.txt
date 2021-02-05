#指定CMake最小版本
cmake_minimum_required(VERSION 2.8.3)

#设置项目名称
project(LC_Calib)

#add_definitions(-std=c++11)

#设置包含库目录
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -o3 -lpthread")


#查找指定的库
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)

#输出相关信息
message(STATUS " PCL library status: ${OpenCV_LIBS}")


#设置包含的目录//指定头文件路径
include_directories(
	/usr/include/eigen3
	./
	/usr/include/
	/usr/local/include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(calib calib_SRC)
add_library(CALIB STATIC ${calib_SRC})

aux_source_directory(InitParam init_param_SRC)
add_library(INIT_PARAM STATIC ${init_param_SRC})


#生成可执行文件
add_executable(t main/main.cpp)

#指定target需要链接的库
target_link_libraries(t 
						CALIB 
						INIT_PARAM
						${OpenCV_LIBRARIES}
						${PCL_LIBRARIES}
						)
