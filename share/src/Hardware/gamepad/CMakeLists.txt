get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
include(.sources.cmake)

set(TARGET Hardware_${NAME})
add_library(${TARGET} SHARED ${SOURCES})
target_link_libraries(${TARGET} 
      LINK_PRIVATE Core)
