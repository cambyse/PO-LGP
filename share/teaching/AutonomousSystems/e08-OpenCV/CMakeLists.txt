get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

add_executable(${NAME} main.cpp)

target_link_libraries(${NAME} opencv_core opencv_imgproc opencv_highgui opencv_features2d)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
