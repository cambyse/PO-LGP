cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++14)

#################################
# Define executable
#################################

file(GLOB SOURCES *.h *.tpp)

add_library(${project_name} STATIC ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
)
set_target_properties(${project_name} PROPERTIES LINKER_LANGUAGE CXX)
add_subdirectory(test)
