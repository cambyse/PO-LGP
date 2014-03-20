cmake_minimum_required(VERSION 2.8.1)
project(MLR_SHARE)
cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0002 OLD)

# default release type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Defaults to RelWithDebInfo" FORCE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DNDEBUG" CACHE STRING "Default flags including -DNDEBUG" FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# always warn
set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

# path for find macros
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# change target locations
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# perform common checks
find_package (Threads REQUIRED)
option(WITH_LAPACK "Enable use of lapack (default: if found)" ON)
if(WITH_LAPACK)
find_package(LAPACK)
if(LAPACK_FOUND)
add_definitions(-DMT_LAPACK)
endif(LAPACK_FOUND)
else(WITH_LAPACK)
message(STATUS "Use of LAPACK disabled by user (option WITH_LAPACK)")
endif(WITH_LAPACK)

include(FindPkgConfig)
option(WITH_OPENCV "Enable use of lapack (default: if found)" ON)
if(WITH_OPENCV)
  pkg_search_module(OPENCV opencv>=2.4)
  if(OPENCV_FOUND)
    message(STATUS "Found OpenCV ${OPENCV_VERSION}")
    add_definitions(-DMT_OPENCV -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS ${OPENCV_CFLAGS})
  endif(OPENCV_FOUND)
else(WITH_OPENCV)
  message(STATUS "Use of OpenCV disabled by user (option WITH_OPENCV)")
endif(WITH_OPENCV)

# allow users to switch off compilation of unit-tests
option(WITH_GTEST "Should we run unit-tests (default yes)" ON)
if(WITH_GTEST)
    option(EXAMPLES_AS_TESTS "Should examples be used as tests? (default yes)" OFF)
    include(CheckIncludeFileCXX)
    check_include_file_cxx("gtest/gtest.h" GTEST_H_FOUND)
    if(GTEST_H_FOUND)
        enable_testing()
        add_subdirectory(/usr/src/gtest gtest)
        link_directories(gtest)
        if(EXAMPLES_AS_TESTS)
          add_definitions(-DMLR_EXAMPLES_AS_TESTS)
        endif(EXAMPLES_AS_TESTS)
        set(TESTING_ENABLED ON)
    else(GTEST_H_FOUND)
        message(WARNING "Did not find gtest.h, cannot enable testing")
        set(TESTING_ENABLED)
    endif(GTEST_H_FOUND)
else(WITH_GTEST)
	set(EXAMPLES_AS_TESTS OFF)
endif(WITH_GTEST)
macro(mlr_example_test NAME)
if(TESTING_ENABLED AND EXAMPLES_AS_TESTS)
	add_test(x_${NAME}_test ${NAME} --gtest-output=xml:${NAME}-unit-xml)
endif()
endmacro()

# configure Python
option(WITH_PYTHON "Build python bindings using SWIG. Will be omitted if swig or NumPy are not found" ON)
if(WITH_PYTHON)
  find_package(SWIG 2.0)
  if(SWIG_FOUND)
    include(${SWIG_USE_FILE})
    find_package(PythonInterp 2.7)
    find_package(PythonLibs 2.7)
    find_package(NumPy)
    if(NUMPY_FOUND)
      set(PYTHON_OK ON CACHE INTERNAL BOOL)
      set(PYTHON_INCLUDES 
        ${PYTHON_INCLUDE_DIRS} 
        ${NUMPY_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include/numpy-${NUMPY_VERSION_MAJOR}.${NUMPY_VERSION_MINOR}
        ${CMAKE_SOURCE_DIR}/include/numpy
        CACHE INTERNAL STRING
    )

    if(${SWIG_VERSION} VERSION_LESS "2.0.8")
      add_definitions(-DSWIG_DEREFERENCE_MEMBERS)
    endif(${SWIG_VERSION} VERSION_LESS "2.0.8")

    else(NUMPY_FOUND)
    endif(NUMPY_FOUND)
  else(SWIG_FOUND)
    message(STATUS "swig not found, will not build Python bindings")
    set(PYTHON_OK OFF) # remove from cache, if earlier configured with python
  endif(SWIG_FOUND)
else(WITH_PYTHON)
   set(PYTHON_OK OFF) # remove from cache, if earlier configured with python 
endif(WITH_PYTHON)

option(WITH_ROS "Use ROS, if available" ON)
if(WITH_ROS)
  message(STATUS "Looking for rosbuild")
  find_file(ROSBUILD rosbuild.cmake HINTS $ENV{ROS_ROOT}/core/rosbuild/)
  if(ROSBUILD)
    message(STATUS "rosbuild found at ${ROSBUILD}")
  else(ROSBUILD)
    unset(ROSBUILD CACHE)
    message(STATUS "rosbuild not found, not compiling anything that requires it")
  endif(ROSBUILD)
else(WITH_ROS)
  unset(ROSBUILD CACHE)
endif(WITH_ROS)

#find_package(DOXYGEN)
#if(DOXYGEN_FOUND)

#else(DOXYGEN_FOUND)
#message(STATUS "Doxygen not found, documentation will not be generated")
#endif(DOXYGEN_FOUND)

# common configuration
include_directories(src)
include_directories(include)
set(GL_LIBRARIES glut GLU GL X11)
mark_as_advanced(GL_LIBRARIES)
if(ENV{MLR_LIBPATH})
link_directories(ENV{MLR_LIBPATH}/lib)
include_directories(ENV{MLR_LIBPATH}/include)
endif(ENV{MLR_LIBPATH})

# check wether OpenMP is available and use it be default it if it
option(WITH_OPENMP "Use OpenMP, if available (default ON)" ON)
if(WITH_OPENMP)
    message(STATUS "Looking for OpenMP")
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message(STATUS "Found OpenMP, enabling it")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp")
    else()
        message(STATUS "OpenMP not found, compiling without it")
    endif()
endif()

# our code uses C++ 10x stuff by now
set(MLR_CXX_FLAGS "-std=c++0x ${MLR_CXX_FLAGS}")

# set Gui-GL definition globally for convenience
include(src/Gui/gl.cmake OPTIONAL)
message(STATUS "Using OpenGL: ${HAVE_OPENGL}")
# need the result of this for the examples
include(src/Hardware/G4/find_G4.cmake OPTIONAL)
include(src/Hardware/kinect/freenect.cmake OPTIONAL)
include(src/Ors/physx.cmake OPTIONAL)

# set MT definitions depending on which external libs we have
find_file(SLICE_EXTERN .slicedef.cmake 
  PATHS ${CMAKE_SOURCE_DIR}/src/extern
  NO_DEFAULT_PATH)
if(SLICE_EXTERN)
include(${SLICE_EXTERN})
message(STATUS "External libs available: ${CURRENT_SLICE}")
foreach(DIR ${CURRENT_SLICE})
add_definitions(-DMT_extern_${DIR})
endforeach(DIR ${CURRENT_SLICE})
endif(SLICE_EXTERN)

# select what to compile
include(.slicedef.cmake)

message(STATUS "MLR CXX Flags: ${MLR_CXX_FLAGS}")

# always need this, always needs to be first (for variables)
add_subdirectory(src)

foreach(DIR ${CURRENT_SLICE})
    if(NOT ${DIR} MATCHES "src")
        add_subdirectory(${DIR})
    endif()
endforeach(DIR ${CURRENT_SLICE})


