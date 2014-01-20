get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${NAME})

include_directories(${CMAKE_SOURCE_DIR}/src)

if(ROSBUILD)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})
target_link_libraries(x_${NAME} Ors Core Algo Gui extern_SWIFT Optim)
add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

endif(ROSBUILD)
