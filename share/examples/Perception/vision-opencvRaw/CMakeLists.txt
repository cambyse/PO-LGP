get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(OPENCV_FOUND)
include(.sources.cmake)
link_directories(${OPENCV_LIBRARY_DIRS})
add_executable(x_${NAME} ${SOURCES})


target_link_libraries(x_${NAME} Ors Gui Core Perception ${OPENCV_LIBRARIES})

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
endif(OPENCV_FOUND)
