find_package(GTK2 REQUIRED gtk)

include(CheckLibraryExists)
include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(FindPkgConfig)

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


set(SOURCES     ${SOURCES}
                geo_qhull.cpp
		graphview.cpp
		gtk.cpp
		mesh.cpp
		opengl.cpp
		ors_opengl.cpp
		plot.cpp
                )

add_library(Gui SHARED ${SOURCES})
target_link_libraries(Gui Core 
        extern_GJK extern_ply 
        ${GTKGL_LIBRARY_DIRS} 
        ${GTKGL_LIBRARIES} 
        ${GTK2_LIBRARIES} 
        graph cgraph gvc 
        qhull 
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

