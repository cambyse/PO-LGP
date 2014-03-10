get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Ors Core Algo Gui extern_SWIFT Optim)

mlr_example_test(${NAME})

