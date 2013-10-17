cmake_minimum_required(VERSION 2.8.1)
project(MLR_SHARE)
cmake_policy(SET CMP0015 NEW)

# almost all programs use threads
find_package (Threads)

if(CHECK_LAPACK)
find_package(LAPACK)
endif(CHECK_LAPACK)

# prepend search path
set(ENV{PKG_CONFIG_PATH} ${PROJECT_BINARY_DIR}/pkgconfig:ENV{PKG_CONFIG_PATH})

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

include_directories(src)
include_directories(include)

# configuration variables
set(GL_LIBRARIES glut GLU GL X11)

set(CURRENT_SLICE 
        src/Core
        src/Algo
        src/Ors
        src/Optim
        src/Infer
        src/Perception
        src/System
        src/Gui 
        src/Motion 
        src/relational
        src/extern/GJK
        src/extern/ply
        src/extern/SWIFT
        src/extern/SWIFT_decomposer
        src/Hardware        
        
        examples/Core 
        examples/Algo
        examples/Optim
        examples/Infer
        examples/Gui
        examples/Ors    
        examples/Motion
        examples/System
        examples/relational
)

foreach(DIR ${CURRENT_SLICE})
add_subdirectory(${DIR})
endforeach(DIR ${CURRENT_SLICE})

