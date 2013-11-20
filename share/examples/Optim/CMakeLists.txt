set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

add_subdirectory(constrained)
add_subdirectory(exercises)
add_subdirectory(kOrderMarkov)
add_subdirectory(optimization)

