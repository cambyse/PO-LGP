get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
set(SOURCES ${SOURCES} )
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Motion Optim Gui Ors Algo devTools)

mlr_example_test(${NAME})

