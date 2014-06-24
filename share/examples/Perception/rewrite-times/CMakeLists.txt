get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_LIBAV)
	find_package(LIBAV COMPONENTS avformat avcodec avutil)
	if(LIBAV_FOUND)
		include(.sources.cmake)
		add_definitions(-DHAVE_LIBAV)
        include_directories(${LIBAV_INCLUDE_DIRS})
		link_directories(${LIBAV_LIBRARY_DIRS})
		add_executable(${NAME} ${SOURCES})
		
		target_link_libraries(${NAME} 
			${LIBAV_AVFORMAT_LIBRARY} 
			${LIBAV_AVCODEC_LIBRARY} 
			${LIBAV_AVUTIL_LIBRARY})
	endif()
endif()
