cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

add_definitions(-std=c++0x)

include_directories(/home/robert/MLR_git/share/include)

add_executable(GTest
    main.cpp
    CommanderTest.cpp
    EnvironmentTest.cpp
    FeatureTest.cpp
    InstanceTest.cpp
    LBFGSTest.cpp
    LearnerTest.cpp
    PlannerTest.cpp
    RandomElements.cpp
    RandomElements.h
    RepresentationTest.cpp
    SandBox.cpp
    SandBox.h
    MinimalEnvironmentExample/MinimalAction.cpp
    MinimalEnvironmentExample/MinimalAction.h
    MinimalEnvironmentExample/MinimalEnvironment.h
    MinimalEnvironmentExample/MinimalObservation.cpp
    MinimalEnvironmentExample/MinimalObservation.h
    MinimalEnvironmentExample/MinimalReward.cpp
    MinimalEnvironmentExample/MinimalReward.h
    ../util/unit_tests.cpp
    ../ActiveOnlineSearch/unit_tests.cpp
    ~/MLR_git/share/include/MCTS_Environment/AbstractEnvironment.h
    ~/MLR_git/share/include/MCTS_Environment/unit_tests.cpp
)

target_include_directories(GTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(GTest PUBLIC
    ActiveOnlineSearch
    ButtonWorld
    CheeseMaze
    environment
    learner
    Maze
    optimization
    planning
    representation
    util
    -lgtest
    -llbfgs
    -lemon
    -larmadillo
    -lgomp
    -lgsl
    -lgslcblas
    Qt5::Core
    Qt5::Widgets
    Qt5::Svg
)
