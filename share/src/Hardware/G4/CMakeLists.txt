get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

option(WITH_G4 "Enable G4 Tracker" OFF)
set(G4ROOT $ENV{MLR_LIBPATH} CACHE PATH "Base dir for G4 tracker" )

if(WITH_G4)
  include(CheckLibraryExists)

  check_library_exists(G4Track g4_close_tracker ${G4ROOT}/Lib/Win32 G4TRACK_FOUND)
  if(G4TRACK_FOUND)
	set(HAVE_G4TRACK 1)
	link_directories(${G4ROOT}/Lib/Win32)
  endif(G4TRACK_FOUND)

  if(HAVE_G4TRACK)
    include(.sources.cmake)

    set(TARGET Hardware_${NAME})
    add_library(${TARGET} SHARED ${SOURCES})
    include_directories(${G4ROOT}/include ${G4ROOT}/Inc)
    add_definitions(-DG4_INSTALLED)
    target_link_libraries(${TARGET} 
        LINK_PRIVATE Core G4Track)

  else(HAVE_G4TRACK)
    message(STATUS "G4 tracker support not built because libG4Track not found (checked in ${G4PATH})")
  endif(HAVE_G4TRACK)

endif(WITH_G4)
