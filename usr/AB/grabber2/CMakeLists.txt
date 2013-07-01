project(grabAndCompress)
cmake_minimum_required(VERSION 2.8.7)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_BUILD_TYPE "Debug")

add_library(Ximea SHARED IMPORTED)
set_target_properties(Ximea PROPERTIES IMPORTED_LOCATION "/opt/XIMEA/lib/libXIMEA_GenTL.so")

set(Ximea_LIBRARIES "libXIMEA_GenTL")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS -I/opt/XIMEA/include")

# all sources
set(grabAndCompress_SOURCES
    main.cpp
   )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(grabAndCompress ${grabAndCompress_SOURCES}) 

LINK_DIRECTORIES(${Ximea_LIBDIR})
TARGET_LINK_LIBRARIES(grabAndCompress Ximea)
