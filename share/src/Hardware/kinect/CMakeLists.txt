get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(FindPkgConfig)
pkg_check_modules(DEPLIB libfreenect)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(DEPLIB_FOUND)
set(SOURCES     ${SOURCES}
                kinect.cpp
)

set(TARGET libHardware_${NAME})
add_library(${TARGET} SHARED ${SOURCES})
include_directories(${DEPLIB_INCLUDE_DIRS})
add_definitions(${DEPLIB_CFLAGS})
target_link_libraries(${TARGET} 
        LINK_PRIVATE Core ${DEPLIB_LIBRARY_DIR} ${DEPLIB_LIBRARIES})

else(DEPLIB_FOUND)
message(STATUS "Kinect support not built because libfreenect not found")
endif(DEPLIB_FOUND)

