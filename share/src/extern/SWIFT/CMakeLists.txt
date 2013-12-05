get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)

add_library(extern_${NAME} STATIC ${SOURCES})
target_link_libraries(extern_${NAME} qhull)
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_definitions(-DNO_CYCLE_DETECTION -DSWIFT_DECOMP)

