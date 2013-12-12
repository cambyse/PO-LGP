get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)

add_library(extern_${NAME} STATIC ${SOURCES})
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

