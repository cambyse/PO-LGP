get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_library(extern_${NAME} STATIC gjk.c)
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

