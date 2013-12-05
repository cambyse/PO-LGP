get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(${NAME} ${SOURCES})

target_link_libraries(${NAME} Ors Core Algo Gui extern_SWIFT Optim)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

