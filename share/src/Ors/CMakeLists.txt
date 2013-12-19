# TODO add automatic checking for presence
option(USE_PHYSX "Use NVidia PhysX libraries" OFF)
if(USE_PHYSX)
        add_definitions(-DMT_PHYSX)
endif(USE_PHYSX)
option(USE_ODE "Use ODE" OFF)
if(USE_ODE)
        add_definitions(-DMT_ODE)
endif(USE_ODE)
option(USE_OPENMP "Use OpenMP" OFF)
if(USE_OPENMP)
        add_definitions(-fopenmp -DOPENMP)
endif(USE_OPENMP)
# TODO option(MT_CUDA "Use CUDA" OFF)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Ors SHARED ${SOURCES})
target_link_libraries(Ors LINK_PRIVATE Algo Core Gui extern_SWIFT)

# Python wrappers
if(PYTHON_OK)
  set(SWIG_SOURCES ors.i)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDES})
  set_source_files_properties(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)
  string(TOLOWER ${NAME}py PY)
  set(CMAKE_SWIG_FLAGS -module _${PY})
  swig_add_module(${PY} python ${SWIG_SOURCES})
  swig_link_libraries(${PY} ${NAME} ${PYTHON_LIBRARIES})
endif(PYTHON_OK)
