find_package(GTK2 REQUIRED gtk)

include(CheckLibraryExists)
include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(FindPkgConfig)

# Gui options
option(USE_QHULL "Use QHULL" ON)
if(USE_QHULL)
        set(MT_QHULL -DMT_QHULL)
        set(QHULL_LIBRARIES qhull)
        set(PKG_QHULL -lqhull)
endif(USE_QHULL)
option(USE_GRAPHVIZ "Use graphviz visualization" ON)
if(USE_GRAPHVIZ)
endif(USE_GRAPHVIZ)

pkg_check_modules(GTKGL REQUIRED gtkglext-1.0)

# first check lib then header for qhull
check_library_exists(qhull qh_meminit "" QHULL_FOUND)
if(QHULL_FOUND)
	check_include_file(qhull/qhull.h QHULL_FOUND)
endif(QHULL_FOUND)
if(NOT QHULL_FOUND)
	message(FATAL_ERROR "qhull library not found, cannot build gui")
endif(NOT QHULL_FOUND)

# first check lib then header for graphviz
check_include_file_CXX(graphviz/graphviz_version.h GRAPHVIZ_FOUND)
if(NOT GRAPHVIZ_FOUND)
	message(FATAL_ERROR "graph library not found, cannot build gui")
endif(NOT GRAPHVIZ_FOUND)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Gui SHARED ${SOURCES})
target_link_libraries(Gui LINK_PRIVATE Core 
        extern_GJK extern_ply 
        ${GTKGL_LIBRARY_DIRS} 
        ${GTKGL_LIBRARIES} 
        ${GTK2_LIBRARIES} 
        graph cgraph gvc 
        qhull 
        LINK_PUBLIC
        ${GL_LIBRARIES}
)
add_definitions(-DMT_GTKGL 
        -DMT_GTK 
        -DMT_GL
        -DMT_QHULL 
        -DMT_GRAPHVIZ 
        -DMT_extern_GJK 
        -DMT_extern_ply
)

include_directories(${GTK2_INCLUDE_DIRS} ${GTKGL_INCLUDE_DIRS})
add_definitions(${LAPACK_DEFINITIONS} ${GTK2_DEFINITIONS})

