get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(FindPkgConfig)
pkg_check_modules(LIBFREENECT libfreenect)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(LIBFREENECT_FOUND)
include(.sources.cmake)

set(TARGET Hardware_${NAME})
add_library(${TARGET} SHARED ${SOURCES})
include_directories(${LIBFREENECT_INCLUDE_DIRS})
add_definitions(${LIBFREENECT_CFLAGS})
target_link_libraries(${TARGET} 
        LINK_PRIVATE Core ${LIBFREENECT_LIBRARY_DIR} ${LIBFREENECT_LIBRARIES})

else(LIBFREENECT_FOUND)
message(STATUS "Kinect support not built because libfreenect not found")
endif(LIBFREENECT_FOUND)

