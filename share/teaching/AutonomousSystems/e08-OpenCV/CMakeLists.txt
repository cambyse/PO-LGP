get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(OPENCV_FOUND)
set(SOURCES main.cpp)

link_directories(${OPENCV_LIBRARY_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

check_include_file_cxx(opencv2/nonfree/features2d.hpp HAVE_OPENCV_NONFREE)
if(HAVE_OPENCV_NONFREE)
add_definitions(-DHAVE_OPENCV_NONFREE)
endif(HAVE_OPENCV_NONFREE)

add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Core ${OPENCV_LIBRARIES})




add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

endif(OPENCV_FOUND)

