get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_FREENECT)
if(LIBFREENECT_FOUND)

	include(.sources.cmake)
	add_executable(x_${NAME} ${SOURCES})
	
	target_link_libraries(x_${NAME} Hardware_kinect Gui Core)
	
	mlr_example_test(${NAME})

endif()	
endif(WITH_FREENECT)
