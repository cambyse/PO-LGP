get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
set(SOURCES ${SOURCES} ../splines/spline.cpp ../pfc/mobject.cpp ../pfc/pfc.cpp ../mpc/mpc.cpp ../dmp/dmp.cpp)
add_executable(x_${NAME} ${SOURCES})

include(FindPkgConfig)

target_link_libraries(x_${NAME} Motion Optim Gui Ors Algo)

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

