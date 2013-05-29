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
    LinearQ.h \
    Feature.h \
    Representation/Action.h \
    Representation/State.h \
    Representation/Reward.h \
    Representation/Instance.h \
    util/KolmogorovSmirnovTest.h \
    util/ChiSquareTest.h
SOURCES += LookAheadSearch.cpp \
    util.cpp \
    BatchMaze.cpp \
    lbfgs_codes.cpp \
    Data.cpp \
    main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    UTree.cpp \
    LinearQ.cpp \
    Feature.cpp \
    Representation/Action.cpp \
    Representation/State.cpp \
    Representation/Reward.cpp \
    Representation/Instance.cpp \
    util/KolmogorovSmirnovTest.cpp \
    util/ChiSquareTest.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo
FORMS += testmaze_ii.ui
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x
