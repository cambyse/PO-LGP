get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(LIBFREENECT_FOUND AND FLYCAPTURE_LIBRARY)

    add_executable(x_${NAME} main.cpp)

    target_link_libraries(x_${NAME} Core Perception Hardware_kinect Hardware_flycap)
	file(COPY MT.cfg view.launch image_rect.launch DESTINATION .)

    add_executable(fps fps_measure.cpp)

    target_link_libraries(fps Core Perception Gui Hardware_kinect Hardware_flycap)
	file(COPY MT.cfg DESTINATION .)

	add_executable(3rdhand_jitter jitter.cpp)
	add_executable(3rdhand_jitter_mt jitter_mt.cpp)
	target_link_libraries(3rdhand_jitter_mt Core) 
endif()
