cmake_minimum_required(VERSION 2.8.10)
project(MLR_SHARE)

# almost all programs use threads
find_package (Threads)

option(CHECK_LAPACK "Should we search for lapack (default yes)" ON)
if(CHECK_LAPACK)
find_package(LAPACK)
endif(CHECK_LAPACK)

# allow users to switch off compilation of unit-tests
option(TESTING_ENABLED "Should we run unit-tests (default yes)" ON)
if(TESTING_ENABLED)
        include(CheckIncludeFileCXX)
        check_include_file_cxx("gtest/gtest.h" GTEST_H_FOUND)
        if(GTEST_H_FOUND)
                enable_testing()
                add_subdirectory(/usr/src/gtest gtest)
                link_directories(gtest)
                add_definitions(-DMT_GTEST -DMLR_EXAMPLES_AS_TESTS)
        else(GTEST_H_FOUND)
                message(WARNING "Did not found gtest.h, cannot enable testing")
        endif(GTEST_H_FOUND)
endif(TESTING_ENABLED)

include_directories(src)
include_directories(include)

add_subdirectory(src/Core)
add_subdirectory(examples/Core/array)

