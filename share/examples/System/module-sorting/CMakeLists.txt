get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(${NAME} ${SOURCES})

target_link_libraries(${NAME} LINK_PRIVATE Core Gui System )

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

