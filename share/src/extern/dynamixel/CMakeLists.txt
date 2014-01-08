get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)

add_library(extern_${NAME} STATIC ${SOURCES})
set_target_properties(extern_${NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

