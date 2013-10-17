get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_executable(${NAME} main.cpp mobject.cpp)

include(FindPkgConfig)

pkg_check_modules(MLRGUI REQUIRED mlr_gui)

target_link_libraries(${NAME} Motion Optim Gui Ors)
add_definitions(${MLRGUI_CFLAGS})

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

