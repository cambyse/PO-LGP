set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# select what to compile
include(.slicedef.cmake)

foreach(DIR ${CURRENT_SLICE})
add_subdirectory(${DIR})
endforeach(DIR ${CURRENT_SLICE})

