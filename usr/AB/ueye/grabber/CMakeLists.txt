project(grabAndCompress)
cmake_minimum_required(VERSION 2.8.7)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_BUILD_TYPE "Debug")

add_library(uEye SHARED IMPORTED)
set_target_properties(uEye PROPERTIES IMPORTED_LOCATION "/usr/lib/libueye_api64.so.4.20")
#set_target_properties(uEye PROPERTIES IMPORTED_LOCATION "/usr/lib/libueye_api.so")

set(uEye_LIBRARIES "libueye_api64")
#set(uEye_LIBRARIES "libueye_api")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS")

# all sources
set(grabAndCompress_SOURCES
    main.cpp
   )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(grabAndCompress ${grabAndCompress_SOURCES}) 

#link_directories(${uEye_LIBDIR})
#link_directories(${uEye_LIBRARIES})

link_directories(${uEye_LIBRARIES})
target_link_libraries(grabAndCompress uEye)
