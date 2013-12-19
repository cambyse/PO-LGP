get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

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
        extern_GJK extern_ply extern_Lewiner
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
)

include_directories(${GTK2_INCLUDE_DIRS} ${GTKGL_INCLUDE_DIRS})
add_definitions(${LAPACK_DEFINITIONS} ${GTK2_DEFINITIONS})

# Python wrappers
if(PYTHON_OK)
  set(SWIG_SOURCES gui.i)
  # add local include for mesh.h
  include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDES})
  set_source_files_properties(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)
  string(TOLOWER ${NAME}py PY)
  set(CMAKE_SWIG_FLAGS -module _${PY})
  swig_add_module(${PY} python ${SWIG_SOURCES})
  swig_link_libraries(${PY} ${NAME} ${PYTHON_LIBRARIES})
endif(PYTHON_OK)
