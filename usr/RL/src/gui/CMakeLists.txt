# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Create Qt user interfaces Wrappers
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(MazeGUI
    main.cpp
    testmaze_ii.h
    testmaze_ii.cpp
)

target_include_directories(MazeGUI PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(MazeGUI
    ButtonWorld
    CheeseMaze
    environment
    learner
    Maze
    optimization
    planning
    qcustomplot
    representation
    util
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::Svg
    -llbfgs
    -lemon
    -larmadillo
    -lgomp
    -lgsl
    -lgslcblas
    -lgsl
#    -cblas
)
