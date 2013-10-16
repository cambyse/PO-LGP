get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_executable(${NAME} main.cpp)

target_link_libraries(${NAME} Ors Core Gui)
add_definitions(-DMT_GL -DMT_Core -DMT_Gui -DMT_Ors)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

