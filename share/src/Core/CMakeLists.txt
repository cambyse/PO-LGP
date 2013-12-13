get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(MLR_${NAME})

include(CheckIncludeFiles)
if(LAPACK_FOUND)
  add_definitions(-DMT_LAPACK)
endif(LAPACK_FOUND)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(${NAME} SHARED ${SOURCES})
target_link_libraries(${NAME}
        LINK_PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
)

# check what we need for nanosleep
include(CheckFunctionExists)
check_function_exists(clock_nanosleep NANOSLEEP_IN_LIBC)
if(NOT NANOSLEEP_IN_LIBC)
  set(CMAKE_REQUIRED_LIBRARIES rt)
  check_function_exists(clock_nanosleep NANOSLEEP_IN_RT)
  if(NANOSLEEP_IN_RT)
    message(STATUS "Adding librt to ${NAME} build for clock_nanosleep")
    target_link_libraries(${NAME} LINK_PUBLIC ${CMAKE_REQUIRED_LIBRARIES})
    unset(CMAKE_REQUIRED_LIBRARIES)
  else(NANOSLEEP_IN_RT)
    message(FATAL_ERROR "Could not find clock_nanosleep in either libc or librt, stopping")
  endif(NANOSLEEP_IN_RT)
endif(NOT NANOSLEEP_IN_LIBC)

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

# Python wrappers
if(PYTHON_OK)
  set(SWIG_SOURCES core.i)
  include_directories(${PYTHON_INCLUDES})
  set_source_files_properties(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)
  string(TOLOWER ${NAME}py PY)
  set(CMAKE_SWIG_FLAGS -module _${PY})
  swig_add_module(${PY} python ${SWIG_SOURCES})
  swig_link_libraries(${PY} ${NAME} ${PYTHON_LIBRARIES})
endif(PYTHON_OK)
