find_package(TCL REQUIRED)

get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

include(.sources.cmake)
set(SOURCES ${SOURCES} RAPID/build.cpp RAPID/collide.cpp RAPID/overlap.cpp
        RAPID/RAPID.cpp)

add_library(extern_${NAME} STATIC ${SOURCES})
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_definitions(-DDECOMP_GRAPHICS -DSWIFT_DECOMP)
include_directories(. RAPID ${TCL_INCLUDE_PATH} ${TK_INCLUDE_PATH})
target_link_libraries(extern_${NAME} ${TCL_LIBRARY} ${TK_LIBRARY} ${GL_LIBRARIES})

#add_executable(decomposer.exe main.cxx)

