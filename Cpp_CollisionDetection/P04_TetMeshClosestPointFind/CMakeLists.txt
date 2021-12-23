cmake_minimum_required(VERSION 2.8)

find_package(embree 3.0 REQUIRED)

if(NOT DEFINED ENV{MESHFRAME_DIRECTORY})
    message(FATAL_ERROR "not defined environment variable:MESHFRAME_DIRECTORY")  
else()
	message("Defined environment variable:MESHFRAME_DIRECTORY:")
	message( $ENV{MESHFRAME_DIRECTORY})

endif() 

project(TetMeshClosestPointFind)

include_directories(${EMBREE_INCLUDE_DIRS})
include_directories($ENV{MESHFRAME_DIRECTORY})
include_directories("../Modules")

file(GLOB SRC
    "*.h"
    "*.cpp"
    "../Modules/common/math/constants.cpp"
	"../Modules/PathFinder/**.cpp"
	"../Modules/PathFinder/**.h"
	"../Modules/PointQuery/**.h"
	"../Modules/PointQuery/**.cpp"
	
)
add_executable(TetMeshClosestPointFind ${SRC})
target_link_libraries(TetMeshClosestPointFind ${EMBREE_LIBRARY})