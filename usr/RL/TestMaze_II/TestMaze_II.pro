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
    VisualWorld.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    UTree.h \
    LinearQ.h \
    Feature.h \
    Representation/Action.h \
    Representation/Observation.h \
    Representation/AugmentedObservation.h \
    Representation/SequentialReward.h \
    Representation/EnumeratedReward.h \
    Representation/Instance.h \
    util/KolmogorovSmirnovTest.h \
    util/ChiSquareTest.h \
    util/ProgressBar.h \
    DelayDistribution.h \
    HistoryObserver.h \
    qcustomplot.h \
    SmoothingKernelSigmoid.h \
    QtUtil.h \
    optimization/LBFGS_Optimizer.h
SOURCES += LookAheadSearch.cpp \
    util.cpp \
    BatchMaze.cpp \
    lbfgs_codes.cpp \
    Config.cpp \
    main.cpp \
    Maze.cpp \
    VisualWorld.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    UTree.cpp \
    LinearQ.cpp \
    Feature.cpp \
    Representation/Action.cpp \
    Representation/Observation.cpp \
    Representation/AugmentedObservation.cpp \
    Representation/SequentialReward.cpp \
    Representation/EnumeratedReward.cpp \
    Representation/Instance.cpp \
    util/KolmogorovSmirnovTest.cpp \
    util/ChiSquareTest.cpp \
    util/ProgressBar.cpp \
    DelayDistribution.cpp \
    HistoryObserver.cpp \
    qcustomplot.cpp \
    SmoothingKernelSigmoid.cpp \
    QtUtil.cpp \
    optimization/LBFGS_Optimizer.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp
INCLUDEPATH +=
LIBPATH += /usr/share/lib
FORMS += testmaze_ii.ui
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp
QMAKE_CC = gcc-4.8
QMAKE_CXX = g++-4.8
