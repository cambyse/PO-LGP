TEMPLATE = app
TARGET = TestMaze
QT += core \
    gui
HEADERS += Visualization/VisualizeMaze.h \
    Utils/Identifier.h \
    WorldModel/MazeModel.h \
    WorldModel/MazeTransition.h \
    WorldModel/MazeAction.h \
    WorldModel/MazeState.h \
    testmaze.h
SOURCES += Visualization/VisualizeMaze.cpp \
    WorldModel/MazeModel.cpp \
    main.cpp \
    testmaze.cpp
FORMS += testmaze.ui
RESOURCES += 
