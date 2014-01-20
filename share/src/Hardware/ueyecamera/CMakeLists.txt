get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

option(WITH_UEYE "Enable UEye Camera" ON)
set(UEYE_ROOT $ENV{MLR_LIBPATH} CACHE PATH "Base dir for UEye library" )

if(WITH_UEYE)
  find_library(UEYE ueye_api PATHS ${UEYE_ROOT})
  message(STATUS "UEye API location: ${UEYE}")

  if(UEYE)
    include(.sources.cmake)

    set(TARGET Hardware_${NAME})
    add_library(${TARGET} SHARED ${SOURCES})
    target_link_libraries(${TARGET} 
            LINK_PRIVATE Core ueye_api)

  else(UEYE)
    message(STATUS "UEye camera support not built because libueye_api not found (checked in ${UEYE_ROOT})")
  endif(UEYE)
endif(WITH_UEYE)
