CONFIG += debug
TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg
HEADERS += LookAheadSearch.h \
    LookAheadGraph.h \
    LookAheadTree.h \
    MCTS.h \
    KMDPState.h \
    BatchMaze.h \
    QIteration.h \
    lbfgs_codes.h \
    Data.h \
    ValueIteration.h \
    util.h \
    Maze.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    Feature.h \
    Representation/Action.h \
    Representation/State.h \
    Representation/Reward.h \
    Representation/Instance.h
SOURCES += LookAheadSearch.cpp \
    LookAheadGraph.cpp \
    LookAheadTree.cpp \
    MCTS.cpp \
    KMDPState.cpp \
    util.cpp \
    BatchMaze.cpp \
    QIteration.cpp \
    Data.cpp \
    ValueIteration.cpp \
    main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
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
