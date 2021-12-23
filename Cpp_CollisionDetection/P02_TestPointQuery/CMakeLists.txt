cmake_minimum_required(VERSION 2.8)

find_package(embree 3.0 REQUIRED)

if(NOT DEFINED ENV{MESHFRAME_DIRECTORY})
    message(FATAL_ERROR "not defined environment variable:MESHFRAME_DIRECTORY")  
else()
	message("Defined environment variable:MESHFRAME_DIRECTORY:")
	message( $ENV{MESHFRAME_DIRECTORY})

endif() 

project(EmbreePointQuery)

include_directories(${EMBREE_INCLUDE_DIRS})
include_directories($ENV{MESHFRAME_DIRECTORY})

file(GLOB SRC
    "*.h"
    "*.cpp"
    "common/math/constants.cpp"
)
add_executable(EmbreePointQuery ${SRC})
target_link_libraries(EmbreePointQuery ${EMBREE_LIBRARY})