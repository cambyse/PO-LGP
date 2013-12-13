get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

set(SOURCES main.cpp)

link_directories(${OPENCV_LIBRARY_DIRS})
add_executable(x_${NAME} ${SOURCES})

include_directories(${OPENCV_INCLUDE_DIRS})
target_link_libraries(x_${NAME} Core ${OPENCV_LIBRARIES})

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
