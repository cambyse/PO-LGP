get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_executable(${NAME} main.cpp mobject.cpp)

target_link_libraries(${NAME} Motion Optim Gui Ors)
add_definitions(-DMT_GL -DMT_Motion -DMT_Core -DMT_Ors -DMT_Gui -DMT_Optim)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

