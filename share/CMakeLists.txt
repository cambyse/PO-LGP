cmake_minimum_required(VERSION 2.8.1)
project(MLR_SHARE)
cmake_policy(SET CMP0015 NEW)

# path for find macros
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# perform common checks
find_package (Threads REQUIRED)
find_package(LAPACK)

# allow users to switch off compilation of unit-tests
option(TESTING_ENABLED "Should we run unit-tests (default yes)" ON)
if(TESTING_ENABLED)
        include(CheckIncludeFileCXX)
        check_include_file_cxx("gtest/gtest.h" GTEST_H_FOUND)
        if(GTEST_H_FOUND)
                enable_testing()
                add_subdirectory(/usr/src/gtest gtest)
                link_directories(gtest)
                add_definitions(-DMLR_EXAMPLES_AS_TESTS)
        else(GTEST_H_FOUND)
                        message(WARNING "Did not find gtest.h, cannot enable testing")
                set(TESTING_ENABLED)
        endif(GTEST_H_FOUND)
endif(TESTING_ENABLED)

# configure Python
option(ENABLE_PYTHON "Build python bindings using SWIG. Will be omitted of swig or NumPy are not found" ON)
if(ENABLE_PYTHON)
  find_package(SWIG 2.0)
  if(SWIG_FOUND)
    include(${SWIG_USE_FILE})
    find_package(PythonInterp 2.7)
    find_package(PythonLibs 2.7)
    find_package(NumPy)
    if(NUMPY_FOUND)
      set(PYTHON_OK ON)
      set(PYTHON_INCLUDES 
        ${PYTHON_INCLUDE_DIRS} 
        ${NUMPY_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include/numpy-${NUMPY_VERSION_MAJOR}.${NUMPY_VERSION_MINOR}
        ${CMAKE_SOURCE_DIR}/include/numpy
    )

    else(NUMPY_FOUND)
    endif(NUMPY_FOUND)
  else(SWIG_FOUND)
    message(STATUS "swig not found, will not build Python bindings")
  endif(SWIG_FOUND)
endif(ENABLE_PYTHON)

# common configuration
include_directories(src)
include_directories(include)
set(GL_LIBRARIES glut GLU GL X11)
mark_as_advanced(GL_LIBRARIES)
if(ENV{MLR_LIBPATH})
link_directories(ENV{MLR_LIBPATH}/lib)
include_directories(ENV{MLR_LIBPATH}/include)
endif(ENV{MLR_LIBPATH})

# our code uses C++ 10x stuff by now
set(MLR_CXX_FLAGS "-std=c++0x ${MLR_CXX_FLAGS}")

# set Gui-GL definition globally for convenience
include(src/Gui/gl.cmake OPTIONAL)
if(USE_OPENGL)
set(MLR_CXX_FLAGS "${MLR_CXX_FLAGS} -DMT_GL")
endif(USE_OPENGL)
message(STATUS "Using OpenGL: ${USE_OPENGL}, MLR CXX Flags: ${MLR_CXX_FLAGS}")

# target to create includes
add_custom_target(sourceinc ALL
  COMMAND ${CMAKE_SOURCE_DIR}/bin/update-includes.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# select what to compile
include(.slicedef.cmake)

foreach(DIR ${CURRENT_SLICE})
add_subdirectory(${DIR})
endforeach(DIR ${CURRENT_SLICE})

