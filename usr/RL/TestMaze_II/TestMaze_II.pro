TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg
HEADERS += ValueIteration.h \
    util.h \
    Maze.h \
    testmaze_ii.h
SOURCES += main.cpp \
    testmaze_ii.cpp
FORMS += testmaze_ii.ui
RESOURCES += 
QMAKE_CXXFLAGS+=-std=c++11