get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

option(WITH_FREENECT "Use libfreenect for Kinect" ON)
if(WITH_FREENECT)
    include(FindPkgConfig)
    pkg_check_modules(LIBFREENECT libfreenect)

    set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

    if(LIBFREENECT_FOUND)
        include(.sources.cmake)

        set(TARGET Hardware_${NAME})

        link_directories(${LIBFREENECT_LIBRARY_DIRS})
        include_directories(${LIBFREENECT_INCLUDE_DIRS})
        add_definitions(${LIBFREENECT_CFLAGS_OTHER})

        add_library(${TARGET} SHARED ${SOURCES})
        target_link_libraries(${TARGET}
                LINK_PRIVATE Core ${LIBFREENECT_LIBRARIES})


    else(LIBFREENECT_FOUND)
        message(STATUS "Kinect support not built because libfreenect not found")
    endif(LIBFREENECT_FOUND)
else(WITH_FREENECT)
    message(STATUS "Kinect support disabled")
endif(WITH_FREENECT)

