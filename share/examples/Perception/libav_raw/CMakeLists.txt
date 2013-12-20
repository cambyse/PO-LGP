get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

set(SOURCES main.cpp)
find_package(LIBAV COMPONENTS avcodec swscale)

if(LIBAV_FOUND)
add_definitions(-DHAVE_LIBAV)
include_directories(${LIBAV_INCLUDE_DIRS})

add_executable(x_${NAME} ${SOURCES})
target_link_libraries(x_${NAME} Gui Core ${LIBAV_LIBRARIES})
add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

endif(LIBAV_FOUND)


