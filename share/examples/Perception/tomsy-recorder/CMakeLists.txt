get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

if(LIBFREENECT_FOUND)

    include(.sources.cmake)
    add_executable(x_${NAME} ${SOURCES})

    target_link_libraries(x_${NAME} Core Ors Perception Hardware_kinect Hardware_ueyecamera System)

endif(LIBFREENECT_FOUND)
