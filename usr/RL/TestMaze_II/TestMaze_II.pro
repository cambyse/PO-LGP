CONFIG += debug
TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg
HEADERS += LookAheadSearch.h \
    BatchMaze.h \
    lbfgs_codes.h \
    Data.h \
    util.h \
    Maze.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    UTree.h \
    Feature.h \
    Representation/Action.h \
    Representation/State.h \
    Representation/Reward.h \
    Representation/Instance.h
SOURCES += LookAheadSearch.cpp \
    util.cpp \
    BatchMaze.cpp \
    Data.cpp \
    main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    UTree.cpp \
    Feature.cpp \
    Representation/Action.cpp \
    Representation/State.cpp \
    Representation/Reward.cpp \
    Representation/Instance.cpp
LIBS += -llbfgs \
    -lemon
FORMS += testmaze_ii.ui
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x
