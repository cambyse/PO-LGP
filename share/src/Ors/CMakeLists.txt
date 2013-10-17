option(ORS_PHYSX "Use NVidia PhysX libraries" OFF)
option(ORS_ODE "Use ODE" OFF)

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

set(SOURCES     ${SOURCES}
                ors_actionInterface.cpp
                ors_blender.cpp
                ors.cpp
                ors_featherstone.cpp
                ors_lewiner.cpp
                ors_ode.cpp
                ors_opengl.cpp
                ors_physx.cpp
                ors_sceneGui.cpp
                ors_swift.cpp
                ors_taskVariables.cpp
                roboticsCourse.cpp
                )

add_library(Ors SHARED ${SOURCES})
target_link_libraries(Ors LINK_PRIVATE Algo Core Gui extern_SWIFT)
add_definitions(-DMT_ALGO 
        -DMT_Core 
        -DMT_Gui 
        -DMT_Extern_SWIFT
        -DMT_GL
        )


