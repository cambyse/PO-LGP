
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR} disabled because of compile error")

#get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
#include(.sources.cmake)
#add_executable(${NAME} ${SOURCES})
#target_link_libraries(${NAME} Core Algo )
#add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
