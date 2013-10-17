cmake_minimum_required(VERSION 2.8.1)
project(MLR_SHARE)
cmake_policy(SET CMP0015 NEW)

# almost all programs use threads
find_package (Threads)

option(CHECK_LAPACK "Should we search for lapack (default yes)" ON)
if(CHECK_LAPACK)
find_package(LAPACK)
endif(CHECK_LAPACK)

# TODO add automatic checking for presence
option(MT_PHYSX "Use PhysX" OFF)
if(MT_PHYSX)
        add_definitions(-DMT_PHYSX)
endif(MT_PHYSX)
option(MT_ODE "Use ODE" OFF)
if(MT_ODE)
        add_definitions(-DMT_ODE)
endif(MT_ODE)
option(MT_OPENMP "Use OpenMP" OFF)
if(MT_OPENMP)
        add_definitions(-fopenmp -DOPENMP)
endif(MT_OPENMP)
# TODO option(MT_CUDA "Use CUDA" OFF)

# Gui options
option(USE_FLTK "Use FLTK" OFF)
if(USE_FLTK)
        set(MT_FLTK -DMT_FLTK)
endif(USE_FLTK)
option(USE_GTKGL "Use GTK GL" ON)
if(USE_GTKGL)
        set(MT_GTKGL -DMT_GTKGL)
        set(USE_GL ON)
endif(USE_GTKGL)
option(USE_FREEGLUT "Use FreeGLUT" OFF)
if(USE_FREEGLUT)
        set(MT_FREEGLUT -DMT_FREEGLUT)
        set(USE_OPENGL ON)
endif(USE_FREEGLUT)
option(USE_OPENGL "Use OpenGL" ON)
if(USE_OPENGL)
        set(MT_GL -DMT_GL)
endif(USE_OPENGL)
option(USE_QHULL "Use QHULL" ON)
if(USE_QHULL)
        set(MT_QHULL -DMT_QHULL)
        set(QHULL_LIBRARIES qhull)
        set(PKG_QHULL -lqhull)
endif(USE_QHULL)
option(USE_GRAPHVIZ "Use graphviz visualization" ON)
if(USE_GRAPHVIZ)
endif(USE_GRAPHVIZ)

# TODO check for illegal combinations

configure_file(
        ${PROJECT_SOURCE_DIR}/pkgconfig/mlr_gui.pc.in
        ${PROJECT_BINARY_DIR}/pkgconfig/mlr_gui.pc
        @ONLY
)
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

set(CORE_DIRS src/Core src/Algo src/System)
set(CORE_SAMPLES examples/Core examples/Algo)
set(GUI_DIRS src/extern/GJK src/extern/ply src/Gui src/extern/SWIFT src/extern/SWIFT_decomposer)
set(GUI_SAMPLES examples/Gui)
set(ORS_DIRS src/Ors)
set(ORS_SAMPLES examples/Ors)
set(OPTIM_DIRS src/Optim)
set(OPTIM_SAMPLES examples/Optim)
set(INFER_DIRS src/Infer)
set(INFER_SAMPLES examples/Infer)
set(PERCEPTION_DIRS src/Perception)
set(MOTION_DIRS src/Motion)
set(MOTION_SAMPLES examples/Motion)
set(SYSTEM_SAMPLES examples/System)

set(CURRENT_SLICE ${CORE_DIRS} ${GUI_DIRS} ${ORS_DIRS} ${OPTIM_DIRS} 
        ${INFER_DIRS} ${MOTION_DIRS} ${PERCEPTION_DIRS}
        ${CORE_SAMPLES} ${GUI_SAMPLES} ${ORS_SAMPLES} ${OPTIM_SAMPLES} 
        ${INFER_SAMPLES} ${MOTION_SAMPLES} ${SYSTEM_SAMPLES}
)

foreach(DIR ${CURRENT_SLICE})
add_subdirectory(${DIR})
endforeach(DIR ${CURRENT_SLICE})

