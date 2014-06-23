get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_LIBAV)
	find_package(LIBAV COMPONENTS avformat avcodec avutil)
	if(LIBAV_FOUND)
		include(.sources.cmake)
		add_definitions(-DHAVE_LIBAV)
        include_directories(${LIBAV_INCLUDE_DIRS})
		add_executable(${NAME} ${SOURCES})
		
		target_link_libraries(${NAME} Core Perception ${LIBAV_AVFORMAT_LIBRARIES} ${LIBAV_AVCODEC_LIBRARIES} ${LIBAV_AVUTIL_LIBRARIES})
	endif()
endif()
