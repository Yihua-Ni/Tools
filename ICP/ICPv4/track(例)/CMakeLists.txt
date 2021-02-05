cmake_minimum_required(VERSION 3.2)
project(tck)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -lpthread")

#find_package(OpenCV 3 REQUIRED)

include_directories(
        ThirdParty/eigen3
		/usr/local/include
		/usr/include
		./
#		${OpenCV_INCLUDE_DIRS}
)


aux_source_directory(Filter Filter_SRC)
add_library(ESKF STATIC ${Filter_SRC})

add_executable(t test/main.cpp)
target_link_libraries(t
					  ${OpenCV_LIBS}
                      ESKF
					  )
