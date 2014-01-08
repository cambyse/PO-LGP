# select what to compile
include(.slicedef.cmake)

# work around bug in (older?) cmake's re PIC in static libs
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

foreach(DIR ${CURRENT_SLICE})
add_subdirectory(${DIR})
endforeach(DIR ${CURRENT_SLICE})

