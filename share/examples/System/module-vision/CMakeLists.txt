get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_executable(${NAME} main.cpp)

target_link_libraries(${NAME} LINK_PRIVATE Core Gui System Perception)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

