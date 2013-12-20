get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

option(WITH_G4 "Enable G4 Tracker" OFF)
set(G4ROOT $ENV{MLR_LIBPATH} CACHE PATH "Base dir for G4 tracker" )

if(WITH_G4)

find_library(G4TRACK G4Track PATHS ${G4ROOT})
message(STATUS "G4Track location: ${G4TRACK}")

if(G4TRACK)
include(.sources.cmake)

set(TARGET Hardware_${NAME})
add_library(${TARGET} SHARED ${SOURCES})
#include_directories(${DEPLIB_INCLUDE_DIRS})
#add_definitions(${DEPLIB_CFLAGS})
target_link_libraries(${TARGET} 
        LINK_PRIVATE Core ${G4TRACK})

else(G4TRACK)
message(STATUS "G4 tracker support not built because libG4Track not found (checked in ${G4ROOT})")
endif(G4TRACK)

endif(WITH_G4)
