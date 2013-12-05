get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
add_executable(${NAME} ${SOURCES})

link_directories(${OPENCV_LIBRARY_DIRS})
target_link_libraries(${NAME} Ors Core Gui extern_SWIFT ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
