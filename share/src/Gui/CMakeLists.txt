get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

find_package(GTK2 REQUIRED gtk)

include(CheckLibraryExists)
include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(FindPkgConfig)

# Gui options
option(WITH_QHULL "Use QHULL" ON)
if(WITH_QHULL)
  # first check lib then header for qhull
  check_library_exists(qhull qh_meminit "" QHULL_FOUND)
  if(QHULL_FOUND)
  check_include_file_cxx("qhull/qhull_a.h" QHULL_H_FOUND)
    if(QHULL_H_FOUND)    
      add_definitions(-DMT_QHULL)
	    set(MT_QHULL -DMT_QHULL)
      set(QHULL_LIBRARIES qhull)
      set(PKG_QHULL -lqhull)
    else(QHULL_H_FOUND)
      message(STATUS "qhull library found, but headers missing. -dev package missing? will omit qhull")
    endif(QHULL_H_FOUND)
  endif(QHULL_FOUND)
endif(WITH_QHULL)
option(WITH_GRAPHVIZ "Use graphviz visualization" ON)
if(WITH_GRAPHVIZ)
  # first check lib then header for graphviz
  check_include_file_CXX(graphviz/graphviz_version.h GRAPHVIZ_FOUND)
  if(GRAPHVIZ_FOUND)
    add_definitions(-DMT_GRAPHVIZ)
  endif(GRAPHVIZ_FOUND)
endif(WITH_GRAPHVIZ)

pkg_check_modules(GTKGL gtkglext-1.0)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_definitions(-DMT_GTK )
if(WITH_OPENGL)
if(FOUND_GTKGL)
add_definitions(-DMT_GTKGL)
endif(FOUND_GTKGL)
endif(WITH_OPENGL)

include_directories(${GTK2_INCLUDE_DIRS} ${GTKGL_INCLUDE_DIRS})
add_definitions(${GTK2_DEFINITIONS})
add_library(Gui SHARED ${SOURCES})

target_link_libraries(Gui LINK_PRIVATE Core 
        extern_GJK extern_ply extern_Lewiner
        ${GTKGL_LIBRARY_DIRS} 
        ${GTKGL_LIBRARIES} 
        ${GTK2_LIBRARIES} 
        cgraph gvc 
        ${QHULL_LIBRARIES}
        LINK_PUBLIC
        ${GL_LIBRARIES}
	GLEW
)

# Python wrappers
if(PYTHON_OK)
  set(SWIG_SOURCES _guipy.i)
  # add local include for mesh.h
  include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDES})
  set_source_files_properties(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)
  string(TOLOWER ${NAME}py PY)
  set(CMAKE_SWIG_FLAGS -module _${PY})
  swig_add_module(${PY} python ${SWIG_SOURCES})
  swig_link_libraries(${PY} ${NAME} ${PYTHON_LIBRARIES})
endif(PYTHON_OK)
