get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

if(G4TRACK)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES})

target_link_libraries(x_${NAME} Core Algo Hardware_G4 Perception)

add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

endif(G4TRACK)
