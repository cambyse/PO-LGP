set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

add_subdirectory(constraint)
add_subdirectory(grasp)
add_subdirectory(grasp_PR2)
add_subdirectory(motion)
add_subdirectory(pfc)

