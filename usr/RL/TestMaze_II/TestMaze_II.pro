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
    Maze/Maze.h \
    VisualEnvironment.h \
    PredictiveEnvironment.h \
    testmaze_ii.h \
    KMarkovCRF.h \
    UTree.h \
    LinearQ.h \
    Feature.h \
    Instance.h \
    util/KolmogorovSmirnovTest.h \
    util/ChiSquareTest.h \
    util/ProgressBar.h \
    DelayDistribution.h \
    HistoryObserver.h \
    qcustomplot.h \
    SmoothingKernelSigmoid.h \
    QtUtil.h \
    optimization/LBFGS_Optimizer.h \
    FeatureLearner.h
SOURCES += LookAheadSearch.cpp \
    util.cpp \
    BatchMaze.cpp \
    lbfgs_codes.cpp \
    main.cpp \
    Maze/Maze.cpp \
    VisualEnvironment.cpp \
    testmaze_ii.cpp \
    KMarkovCRF.cpp \
    UTree.cpp \
    LinearQ.cpp \
    Feature.cpp \
    Instance.cpp \
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
