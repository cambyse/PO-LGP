get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

option(WITH_FLYCAP "Enable FlyCapture-Based Camera capture" ON)
set(FLYCAP_ROOT $ENV{MLR_LIBPATH} CACHE PATH "Base dir for flycapture library" )

if(WITH_FLYCAP)
  find_package(FlyCapture)

  if(FLYCAPTURE_LIBRARY)
    message(STATUS "FlyCapture API location: ${FLYCAPTURE_LIBRARY}")
    include(.sources.cmake)

    set(TARGET Hardware_${NAME})
    add_definitions(-DFLYCAP_INSTALLED)
    add_library(${TARGET} SHARED ${SOURCES})
    target_link_libraries(${TARGET} 
            LINK_PRIVATE Core flycapture)

  else()
    message(STATUS "FlyCapture camera support not built because libflycapture or headers not found (checked in ${FLYCAP_ROOT})")
  endif()
endif()
