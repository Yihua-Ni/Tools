#1 声明要求的cmake最低版本
cmake_minimum_required( VERSION 2.8 )
#2 声明一个cmake工程
project( ICP )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -lpthread")

#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-O3" )

#3 添加Eigen头文件
include_directories( "/usr/include/eigen3"
		/usr/local/include
		/usr/include
		./)


aux_source_directory(ICP ICP_SRC)
aux_source_directory(Tools Tools_SRC)

#4 添加库文件
add_library(ICP STATIC ${ICP_SRC})
add_library(Tools STATIC ${Tools_SRC})

#5 添加可执行程序
#只有带有main函数的文件才会生成可执行文件
#语法: add_executable(程序名  源代码文件)
add_executable( test Test/test.cpp )

#6 将库文件链接到可执行程序上
target_link_libraries(test ICP Tools)


