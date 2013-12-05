get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
set(SOURCES ${SOURCES} ../splines/spline.cpp)
add_executable(${NAME} ${SOURCES})

include(FindPkgConfig)

target_link_libraries(${NAME} Motion Optim Gui Ors Algo)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

