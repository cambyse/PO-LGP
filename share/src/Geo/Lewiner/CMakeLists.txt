get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ply)
add_library(extern_${NAME} STATIC ${SOURCES})
set_target_properties(extern_${NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

