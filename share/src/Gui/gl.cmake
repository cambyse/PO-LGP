option(WITH_FLTK "Use FLTK" OFF)
if(WITH_FLTK)
        add_definitions(-DMT_FLTK)
endif(WITH_FLTK)
option(WITH_OPENGL "Use OpenGL" ON)
if(WITH_OPENGL)
    set(HAVE_OPENGL ON CACHE BOOL "OpenGL is available on the system (currently not checked)")
    add_definitions(-DMT_GL)
    option(WITH_GTKGL "Use GTK GL" ON)
    if(WITH_GTKGL)
            add_definitions(-DMT_GTKGL)            
    endif(WITH_GTKGL)
    option(WITH_FREEGLUT "Use FreeGLUT" OFF)
    if(WITH_FREEGLUT)
            add_definitions(-DMT_FREEGLUT)
    endif(WITH_FREEGLUT)
endif(WITH_OPENGL)

