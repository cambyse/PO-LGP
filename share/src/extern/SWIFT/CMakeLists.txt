get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(SOURCES fileio.cpp lut.cpp mesh.cpp mesh_utils.cpp object.cpp pair.cpp
        pqueue.cpp scene.cpp)

add_library(extern_${NAME} STATIC ${SOURCES})
target_link_libraries(extern_${NAME} qhull)
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_definitions(-DNO_CYCLE_DETECTION -DSWIFT_DECOMP)

