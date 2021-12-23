find_package(embree 3.0 REQUIRED)

project(TestEmbree)

include_directories(${EMBREE_INCLUDE_DIRS})


file(GLOB SRC
    "*.h"
    "*.cpp"
)
add_executable(TestEmbree ${SRC})
target_link_libraries(TestEmbree ${EMBREE_LIBRARY})