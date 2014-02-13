get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

#target_link_libraries(x_${NAME} Core Algo)

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
