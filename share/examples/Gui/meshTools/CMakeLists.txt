get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

add_executable(${NAME} main.cpp)

target_link_libraries(${NAME} Core Gui ${GL_LIBRARIES} qhull extern_SWIFT 
        extern_SWIFT_decomposer)
add_definitions(-DMT_GL -DMT_Core -DMT_Gui -DMT_qhull -DMT_extern_SWIFT 
        -DMT_extern_SWIFT_decomposer)

add_test(${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

