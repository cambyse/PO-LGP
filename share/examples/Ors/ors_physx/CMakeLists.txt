get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Ors Core Gui)

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
file(COPY pin1.off DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
