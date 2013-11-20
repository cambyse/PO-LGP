set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

add_subdirectory(relational_armani_basic)
add_subdirectory(relational_armani_symbolic)
add_subdirectory(relational_basic)
add_subdirectory(relational_learn)
add_subdirectory(relational_nonBinarySymbols)
add_subdirectory(relational_plan)

