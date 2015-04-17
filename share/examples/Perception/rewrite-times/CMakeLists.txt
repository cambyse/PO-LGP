get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_LIBAV)
	find_package(LIBAV COMPONENTS avformat avcodec avutil)
	if(LIBAV_FOUND)
		add_definitions(-DHAVE_LIBAV)
        include_directories(${LIBAV_INCLUDE_DIRS})
		link_directories(${LIBAV_LIBRARY_DIRS})
		add_executable(${NAME} main.cpp av_out.cpp)
		add_executable(av_info avinfo.cpp av_out.cpp)
		
		target_link_libraries(${NAME} 
			${LIBAV_AVFORMAT_LIBRARY} 
			${LIBAV_AVCODEC_LIBRARY} 
			${LIBAV_AVUTIL_LIBRARY})
		target_link_libraries(av_info 
			${LIBAV_AVFORMAT_LIBRARY} 
			${LIBAV_AVCODEC_LIBRARY} 
			${LIBAV_AVUTIL_LIBRARY})
	endif()
endif()
