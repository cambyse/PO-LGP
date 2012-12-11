TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg
HEADERS += QIteration.h \
    lbfgs_codes.h \
    Data.h \
    ValueIteration.h \
    util.h \
    Maze.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    Feature.h
SOURCES += QIteration.cpp \
    Data.cpp \
    ValueIteration.cpp \
    main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    Feature.cpp
LIBS += -llbfgs
FORMS += testmaze_ii.ui
RESOURCES += 
QMAKE_CXXFLAGS += -std=c++0x
