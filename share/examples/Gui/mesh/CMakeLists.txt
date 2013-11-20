get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
add_executable(${NAME} main.cpp)

target_link_libraries(${NAME} Core Gui ${GL_LIBRARIES} qhull)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

