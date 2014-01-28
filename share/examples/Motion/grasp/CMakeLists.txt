get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Motion Optim Gui)

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
file(COPY grasp_mesh.tri MT.cfg schunk_3d schunk.ors schunk.ors.qlin DESTINATION .)
