get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
add_executable(${NAME} ${SOURCES})

target_link_libraries(${NAME} Core Gui Optim)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

