get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Motion Optim Gui)

mlr_example_test(${NAME})
file(COPY grasp_mesh.tri MT.cfg schunk.ors schunk.ors.qlin DESTINATION .)
