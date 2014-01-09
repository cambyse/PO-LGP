get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

set(SOURCES main.cpp)
find_package(LIBAV COMPONENTS avcodec swscale)

if(LIBAV_FOUND)

find_file(CHANNEL_LAYOUT libavutil/channel_layout.h)

if(CHANNEL_LAYOUT_FOUND)

add_definitions(-DHAVE_LIBAV)
include_directories(${LIBAV_INCLUDE_DIRS})

add_executable(x_${NAME} ${SOURCES})
target_link_libraries(x_${NAME} Gui Core ${LIBAV_LIBRARIES})
add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)
else(CHANNEL_LAYOUT_FOUND)
message(STATUS "Not building libav_raw, because avutil/channel_layout.h not found")
endif(CHANNEL_LAYOUT_FOUND)

endif(LIBAV_FOUND)


