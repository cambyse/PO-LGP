get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_G4)
  if(G4TRACK)
    include(.sources.cmake)

	# optionally put in some ROS deps, if available and enabled
	if(ROSBUILD)
		pkg_check_modules(ROS_G4 roscpp tf)
		if(ROS_G4_FOUND)
			message(STATUS "Building ROS transport for G4 transform data")
			add_definitions(-DHAVE_ROS_G4 ${ROS_G4_CFLAGS_OTHER})
			include_directories(${ROS_G4_INCLUDE_DIRS})
			link_directories(${ROS_G4_LIBRARY_DIRS})
		else()
			unset(ROS_G4_LIBRARIES CACHE) # remove possible left-overs
		endif()
	else()
		message(STATUS "ROSBUILD not found, not building ROS transport for G4")
	endif(ROSBUILD)

    set(TARGET Hardware_${NAME})
    add_library(${TARGET} SHARED ${SOURCES})
    include_directories(${G4ROOT}/include ${G4ROOT}/Inc)
    add_definitions(-DG4_INSTALLED)
    target_link_libraries(${TARGET} 
        LINK_PRIVATE Core Gui Ors ${ROS_G4_LIBRARIES}
        LINK_PUBLIC ${G4TRACK}
    )

  else(G4TRACK)
    message(STATUS "G4 tracker support not built because libG4Track not found (checked in ${G4PATH})")
  endif(G4TRACK)
endif(WITH_G4)
