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
    # okay, PhysX library prefixes are a bit variable so lets try some
    set(PHYSX_LIB_HINTS ${PHYSX_ROOT}/lib ${PHYSX_ROOT}/Lib)	

    find_library(PHYSX_LIB PvdRuntimeCHECKED 
        HINTS	 ${PHYSX_LIB_HINTS})

    if(NOT PHYSX_LIB)
        message(STATUS "Px lib not found, trying again with architecture suffix")

        if(CMAKE_SYSTEM_NAME MATCHES "Linux")
            if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
                set(CMAKE_LIBRARY_ARCHITECTURE "linux64")
            else()
                set(CMAKE_LIBRARY_ARCHITECTURE "linux32")
            endif()	
        endif() # linux
        find_library(PHYSX_LIB PvdRuntimeCHECKED
            PATHS ${PHYSX_LIB_HINTS})
    endif()
    
    if(PHYSX_LIB)
        # okay, got one, remember it...
        set(PHYSX_LIBRARIES ${PHYSX_LIB})
        string(REGEX MATCH ".*\\.a$" IS_STATIC ${PHYSX_LIB})
        if(IS_STATIC)
            set(PHSYX_LINK "LINK_PUBLIC")
        else()
            set(PHYSX_LINK "LINK_PRIVATE")
        endif()
        # ...and try the rest with these settings
        foreach(PX_CHECKLIB             LowLevelCHECKED 
            LowLevelClothCHECKED        PhysX3CharacterKinematicCHECKED
            PhysX3CHECKED               PhysX3CommonCHECKED
            PhysX3CookingCHECKED        PhysX3ExtensionsCHECKED 
            PhysX3VehicleCHECKED        PhysXProfileSDKCHECKED
            PhysXVisualDebuggerSDKCHECKED PvdRuntimeCHECKED 
            PxTaskCHECKED               SceneQueryCHECKED
            SimulationControllerCHECKED 
        )
            unset(PHYSX_LIB) 
            find_library(PHYSX_LIB ${PX_CHECKLIB} PATHS ${PHYSX_LIB_HINTS})
            if(PHYSX_LIB)
                    set(PHYSX_LIBRARIES ${PHYSX_LIBRARIES} ${PHYSX_LIB})
            else()
                message(FATAL "Missing PhysX library ${PX_CHECKLIB} from the directory where the rest are found")
            endif()
        endforeach()

        set(PHYSX_FOUND TRUE)
    endif()

    if(PHYSX_FOUND)        
        set(PHYSX_DEFINES -DMT_PHYSX)
        # check if we already have a debug flag set
        string(FIND ${CMAKE_BUILD_TYPE} "DEBUG" DEBUG_BUILD)
        if(NOT DEBUG_BUILD)
          set(PHYSX_DEFINES "${PHYSX_DEFINES} -D_DEBUG")
        endif(NOT DEBUG_BUILD)
        set(PHYSX_INCLUDE_DIRS ${CMAKE_REQUIRED_INCLUDES}) 
        set(PHYSX_LIBRARY_DIRS ${PHYSX_ROOT}/lib)
      endif(PHYSX_FOUND)               
  endif() 
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_DEFINITIONS)
else(WITH_PHYSX)
  unset(PHYSX_INCLUDE_DIRS CACHE)
  unset(PHYSX_LIBRARY_DIR CACHE)
  unset(PHYSX_LIBRARIES CACHE)
endif(WITH_PHYSX)

