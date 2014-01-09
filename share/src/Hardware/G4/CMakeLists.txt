get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(WITH_G4)
  if(G4TRACK)
    include(.sources.cmake)

    set(TARGET Hardware_${NAME})
    add_library(${TARGET} SHARED ${SOURCES})
    include_directories(${G4ROOT}/include ${G4ROOT}/Inc)
    add_definitions(-DG4_INSTALLED)
    target_link_libraries(${TARGET} 
        LINK_PRIVATE Core LINK_PUBLIC ${G4TRACK})

  else(G4TRACK)
    message(STATUS "G4 tracker support not built because libG4Track not found (checked in ${G4PATH})")
  endif(G4TRACK)
endif(WITH_G4)
