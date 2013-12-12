get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(MLR_${NAME})

include(CheckIncludeFiles)
if(LAPACK_FOUND)
add_definitions(-DMT_LAPACK)
endif(LAPACK_FOUND)

# check what we need for nanosleep
include(CheckFunctionExists)
check_function_exists(clock_nanosleep HAS_NANOSLEEP)
if(NOT HAS_NANOSLEEP)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_LIBRARIES rt)
check_function_exists(clock_nanosleep HAS_NANOSLEEP)
if(HAS_NANOSLEEP)
message(STATUS "Adding librt to ${NAME} build for clock_nanosleep")
target_link_libraries(${NAME} LINK_PRIVATE rt)
else(HAS_NANOSLEEP)
message(FATAL "Could not find clock_nanosleep in either libc or librt, stopping")
endif(HAS_NANOSLEEP)
cmake_pop_check_state()
endif(NOT HAS_NANOSLEEP)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(${NAME} SHARED ${SOURCES})
target_link_libraries(${NAME}
        LINK_PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
)

if(LAPACK_FOUND)
include_directories(${LAPACK_INCLUDE_DIR})
add_definitions(${LAPACK_DEFINITIONS})
target_link_libraries(${NAME} 
        LINK_PRIVATE 
        ${LAPACK_LINKER_FLAGS} ${LAPACK_LIBRARIES} 
)
endif(LAPACK_FOUND)

if(TESTING_ENABLED)
target_link_libraries(Core LINK_PUBLIC gtest)
endif(TESTING_ENABLED)

#if(SWIG_FOUND)
#include_directories(${CMAKE_SOURCE_DIR}/include/numpy ${PYTHON_INCLUDE_PATH})
#set_source_files_properties(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)
##set_source_files_properties(${SWIG_SOURCES} PROPERTIES SWIG_FLAGS "-module _${NAME}")
#string(REGEX REPLACE ".i" ".cxx " SWIG_GENERATED ${SWIG_SOURCES})
#set(CMAKE_SWIG_FLAGS -module _${NAME})
#swig_add_module(${NAME}py python ${SWIG_SOURCES})
#swig_link_libraries(${NAME}py ${NAME} ${PYTHON_LIBRARIES})
#endif(SWIG_FOUND)
