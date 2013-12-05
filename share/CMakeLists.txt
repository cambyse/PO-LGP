cmake_minimum_required(VERSION 2.8.1)
project(MLR_SHARE)
cmake_policy(SET CMP0015 NEW)

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

# common configuration
include_directories(src)
include_directories(include)
set(GL_LIBRARIES glut GLU GL X11)
mark_as_advanced(GL_LIBRARIES)
if(ENV{MLR_LIBPATH})
link_directories(ENV{MLR_LIBPATH}/lib)
include_directories(ENV{MLR_LIBPATH}/include)
endif(ENV{MLR_LIBPATH})
# pick up config from source-tree
set(ENV{PKG_CONFIG_PATH} ${PROJECT_BINARY_DIR}/pkgconfig:$ENV{PKG_CONFIG_PATH})

# our code uses C++ 10x stuff by now
set(MLR_CXX_FLAGS "-std=c++11 ${MLR_CXX_FLAGS}")

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

