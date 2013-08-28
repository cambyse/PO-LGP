CONFIG += debug
TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg \
    printsupport
HEADERS += LookAheadSearch.h \
    BatchMaze.h \
    lbfgs_codes.h \
    Config.h \
    util.h \
    Maze.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    UTree.h \
    LinearQ.h \
    Feature.h \
    Representation/Action.h \
    Representation/State.h \
    Representation/SequentialReward.h \
    Representation/EnumeratedReward.h \
    Representation/Instance.h \
    util/KolmogorovSmirnovTest.h \
    util/ChiSquareTest.h \
    util/ProgressBar.h \
    DelayDistribution.h \
    HistoryObserver.h \
    qcustomplot.h \
    SmoothingKernelSigmoid.h
SOURCES += LookAheadSearch.cpp \
    util.cpp \
    BatchMaze.cpp \
    lbfgs_codes.cpp \
    Config.cpp \
    main.cpp \
    Maze.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    UTree.cpp \
    LinearQ.cpp \
    Feature.cpp \
    Representation/Action.cpp \
    Representation/State.cpp \
    Representation/SequentialReward.cpp \
    Representation/EnumeratedReward.cpp \
    Representation/Instance.cpp \
    util/KolmogorovSmirnovTest.cpp \
    util/ChiSquareTest.cpp \
    util/ProgressBar.cpp \
    DelayDistribution.cpp \
    HistoryObserver.cpp \
    qcustomplot.cpp \
    SmoothingKernelSigmoid.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp
INCLUDEPATH +=
FORMS += testmaze_ii.ui
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp
