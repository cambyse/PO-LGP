get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} LINK_PRIVATE Core Gui System )

mlr_example_test(${NAME})

