TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg
HEADERS += ValueIteration.h \
    util.h \
    Maze.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    Feature.h \
    Data.h
SOURCES += main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    Feature.cpp
LIBS += -llbfgs
FORMS += testmaze_ii.ui
RESOURCES += 
QMAKE_CXXFLAGS+=-std=c++11