get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(OPENCV_FOUND)

  add_executable(${NAME} main.cpp)

  link_directories(${OPENCV_LIBRARY_DIRS})
  target_link_libraries(${NAME} opencv_highgui opencv_imgproc opencv_video opencv_features2d opencv_core )

  add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

endif(OPENCV_FOUND)
