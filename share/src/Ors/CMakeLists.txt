option(ORS_PHYSX "Use NVidia PhysX libraries" OFF)
option(ORS_ODE "Use ODE" OFF)

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


