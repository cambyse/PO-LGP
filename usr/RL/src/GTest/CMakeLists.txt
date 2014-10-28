cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_definitions(-DUNIT_TESTS)

add_executable(GTest
    CommanderTest.cpp
    EnvironmentTest.cpp
    Examples.cpp
    Examples.h
    FeatureTest.cpp
    InstanceTest.cpp
    LBFGSTest.cpp
    LearnerTest.cpp
    main.cpp
    PlannerTest.cpp
    RandomElements.cpp
    RandomElements.h
    RepresentationTest.cpp
    SandBox.cpp
    SandBox.h
    TemplateTest.cpp
    UtilTest.cpp
    MinimalEnvironmentExample/MinimalAction.cpp
    MinimalEnvironmentExample/MinimalAction.h
    MinimalEnvironmentExample/MinimalEnvironment.h
    MinimalEnvironmentExample/MinimalObservation.cpp
    MinimalEnvironmentExample/MinimalObservation.h
    MinimalEnvironmentExample/MinimalReward.cpp
    MinimalEnvironmentExample/MinimalReward.h
)

target_include_directories(GTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(GTest PUBLIC
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
