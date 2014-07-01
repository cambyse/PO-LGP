get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)


set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_FREENECT)
if(LIBFREENECT_FOUND)
	include(.sources.cmake)

	set(TARGET Hardware_${NAME})

	link_directories(${LIBFREENECT_LIBRARY_DIRS})
	include_directories(${LIBFREENECT_INCLUDE_DIRS})
	add_definitions(${LIBFREENECT_CFLAGS_OTHER})

	add_library(${TARGET} SHARED ${SOURCES})
	target_link_libraries(${TARGET}
		LINK_PRIVATE Core ${LIBFREENECT_LIBRARIES} -lfreenect_sync)


else()
	message(STATUS "Kinect support not built because libfreenect not found")
endif()
endif(WITH_FREENECT) 
