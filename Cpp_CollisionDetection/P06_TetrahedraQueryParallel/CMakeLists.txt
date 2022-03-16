cmake_minimum_required(VERSION 2.8)

find_package(embree 3.0 REQUIRED)

if(NOT DEFINED ENV{MESHFRAME_DIRECTORY})
    message(FATAL_ERROR "not defined environment variable:MESHFRAME_DIRECTORY")  
else()
	message("Defined environment variable:MESHFRAME_DIRECTORY:")
	message( $ENV{MESHFRAME_DIRECTORY})

endif() 

project(P06_TetrahedraQueryParallel)

include_directories(${EMBREE_INCLUDE_DIRS})
include_directories($ENV{MESHFRAME_DIRECTORY})
include_directories("../Modules")
include_directories("../3rdParty/oneTBB/include")

file(GLOB SRC
    "*.h"
    "*.cpp"
    "../Modules/common/math/constants.cpp"
	"../Modules/PathFinder/**.cpp"
	"../Modules/PathFinder/**.h"
	"../Modules/PointQuery/**.h"
	"../Modules/PointQuery/**.cpp"
	
)
add_executable(TetrahedraQueryParallel ${SRC})
target_link_libraries(TetrahedraQueryParallel ${EMBREE_LIBRARY})