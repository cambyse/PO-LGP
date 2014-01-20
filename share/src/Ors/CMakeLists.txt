include(CheckIncludeFile)
include(CheckIncludeFileCXX)

option(WITH_PHYSX "Use NVidia PhysX libraries" OFF)
set(PHYSX_ROOT $ENV{MLR_LIBPATH} CACHE PATH "Root path for searching PhysX")
if(WITH_PHYSX)
  if(PHYSX_ROOT)
    message(STATUS "PhysX root: ${PHYSX_ROOT}")        
    set(CMAKE_REQUIRED_INCLUDES ${PHYSX_ROOT}/include ${PHYSX_ROOT}/include/physx)
    set(CMAKE_REQUIRED_DEFINITIONS -D_DEBUG)
  else(PHYSX_ROOT)
    message(STATUS "No PhysX root given")        
  endif(PHYSX_ROOT)
        
  check_include_file_cxx("physx/PxPhysicsAPI.h" PHYSX_H_FOUND)

  if(PHYSX_H_FOUND)
    add_definitions(-DMT_PHYSX)
    # check if we already have a debug flag set
    string(FIND ${CMAKE_BUILD_TYPE} "DEBUG" DEBUG_BUILD)
    if(NOT DEBUG_BUILD)
      add_definitions(-D_DEBUG)
    endif(NOT DEBUG_BUILD)
    include_directories(${CMAKE_REQUIRED_INCLUDES})
    link_directories(${PHYSX_ROOT}/lib)
    set(PHYSX_LIBRARIES  
        PvdRuntimeCHECKED 
        SimulationControllerCHECKED
        SceneQueryCHECKED
        LowLevelCHECKED
        LowLevelClothCHECKED
        PhysX3VehicleCHECKED
        PhysX3CookingCHECKED
        PhysX3ExtensionsCHECKED
        PhysX3CharacterKinematicCHECKED
        RepX3CHECKED
        RepXUpgrader3CHECKED
        PhysXProfileSDKCHECKED
        PxTaskCHECKED
        PhysX3        
        PhysX3CommonCHECKED
    )
  else(PHYSX_H_FOUND)
      message(STATUS "PhysX header not found")
  endif(PHYSX_H_FOUND)                
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_DEFINITIONS)
endif(WITH_PHYSX)

# check for ODE
option(WITH_ODE "Use ODE" OFF)
set(ODE_ROOT $ENV{MLR_LIBPATH} CACHE PATH "Root path for searching ODE")
if(WITH_ODE)
        if(ODE_ROOT)
          set(CMAKE_REQUIRED_INCLUDES ${ODE_ROOT}/include)
        endif(ODE_ROOT)
        check_include_file("ode/ode.h" ODE_H_FOUND)
        if(ODE_H_FOUND)
          check_include_file("ode/internal/objects.h" ODE_INTERNAL_H_FOUND)
          if(ODE_INTERNAL_H_FOUND)
            add_definitions(-DMT_ODE)
            include_directories(${CMAKE_REQUIRED_INCLUDES})
            link_directories(${ODE_ROOT}/lib)
            set(ODE_LIBRARIES ode)
          else(ODE_INTERNAL_H_FOUND)
            message(STATUS "ode.h found, but ode/internal directory missing -- you need to build ODE from source for Ors. Skipping ODE support")
          endif(ODE_INTERNAL_H_FOUND)
        else(ODE_H_FOUND)
            message(STATUS "ODE headers not found")
        endif(ODE_H_FOUND)
        unset(CMAKE_REQUIRED_INCLUDES)                        
endif(WITH_ODE)
option(WITH_OPENMP "Use OpenMP" OFF)
if(WITH_OPENMP)
        add_definitions(-fopenmp -DOPENMP)
endif(WITH_OPENMP)
# TODO option(MT_CUDA "Use CUDA" OFF)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Ors SHARED ${SOURCES})
target_link_libraries(Ors LINK_PUBLIC ${PHYSX_LIBRARIES} LINK_PRIVATE Algo Core Gui extern_SWIFT ${ODE_LIBRARIES})

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
