CONFIG += debug
TEMPLATE = app
TARGET = TestMaze_II
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    Config.h \
    \
    util.h \
    util/Macro.h \
    util/ProgressBar.h \
    util/ColorOutput.h \
    util/QtUtil.h \
    util/lbfgs_codes.h \
    util/KolmogorovSmirnovDist.h \
    util/KolmogorovSmirnovTest.h \
    \
    SmoothingKernelSigmoid.h \
    qcustomplot.h \
    DelayDistribution.h \
    \
    optimization/LBFGS_Optimizer.h \
    \
    Feature.h \
    Instance.h \
    AbstractAction.h \
    AbstractObservation.h \
    AbstractReward.h \
    ListedReward.h \
    \
    Predictor.h \
    Environment.h \
    PredictiveEnvironment.h \
    Visualizer.h \
    HistoryObserver.h \
    FeatureLearner.h \
    \
    Maze/Maze.h \
    Maze/MazeAction.h \
    Maze/AugmentedMazeAction.h \
    AbstractObservation.h \
    Maze/MazeObservation.h \
    \
    LookAheadSearch.h \
    \
    KMarkovCRF.h \
    UTree.h \
    LinearQ.h \
    \
    BatchMaze.h \
    testmaze_ii.h
SOURCES += \
    main.cpp \
    \
    util.cpp \
    util/ProgressBar.cpp \
    util/ColorOutput.cpp \
    util/QtUtil.cpp \
    util/lbfgs_codes.cpp \
    util/KolmogorovSmirnovDist.cpp \
    util/KolmogorovSmirnovTest.cpp \
    \
    SmoothingKernelSigmoid.cpp \
    qcustomplot.cpp \
    DelayDistribution.cpp \
    \
    optimization/LBFGS_Optimizer.cpp \
    \
    Feature.cpp \
    Instance.cpp \
    AbstractAction.cpp \
    AbstractObservation.cpp \
    AbstractReward.cpp \
    ListedReward.cpp \
    \
    Environment.cpp \
    PredictiveEnvironment.cpp \
    Visualizer.cpp \
    HistoryObserver.cpp \
    FeatureLearner.cpp \
    \
    Maze/Maze.cpp \
    Maze/MazeAction.cpp \
    Maze/AugmentedMazeAction.cpp \
    Maze/MazeObservation.cpp \
    \
    LookAheadSearch.cpp \
    \
    KMarkovCRF.cpp \
    UTree.cpp \
    LinearQ.cpp \
    \
    BatchMaze.cpp \
    testmaze_ii.cpp
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
