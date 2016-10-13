get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(SOURCES gjk.c)
add_library(extern_${NAME} STATIC ${SOURCES})
set_source_files_properties(${SOURCES} PROPERTIES POSITION_INDEPENDENT_CODE ON)

