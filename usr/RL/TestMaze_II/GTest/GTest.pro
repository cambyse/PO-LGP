CONFIG += debug
TARGET = GTest
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    ../LookAheadSearch.h \
    ../BatchMaze.h \
    ../lbfgs_codes.h \
    ../Config.h \
    ../util.h \
    ../Maze.h \
    ../VisualWorld.h \
    ../testmaze_ii.h \
    ../KMarkovCRF.h \
    ../UTree.h \
    ../LinearQ.h \
    ../Feature.h \
    ../Representation/Action.h \
    ../Representation/Observation.h \
    ../Representation/AugmentedObservation.h \
    ../Representation/SequentialReward.h \
    ../Representation/EnumeratedReward.h \
    ../Representation/Instance.h \
    ../util/KolmogorovSmirnovTest.h \
    ../util/ChiSquareTest.h \
    ../util/ProgressBar.h \
    ../DelayDistribution.h \
    ../HistoryObserver.h \
    ../qcustomplot.h \
    ../SmoothingKernelSigmoid.h \
    ../QtUtil.h \
    ../optimization/LBFGS_Optimizer.h
SOURCES += \
    main.cpp \
    ../LookAheadSearch.cpp \
    ../util.cpp \
    ../BatchMaze.cpp \
    ../lbfgs_codes.cpp \
    ../Config.cpp \
    ../Maze.cpp \
    ../VisualWorld.cpp \
    ../testmaze_ii.cpp \
    ../KMarkovCRF.cpp \
    ../UTree.cpp \
    ../LinearQ.cpp \
    ../Feature.cpp \
    ../Representation/Action.cpp \
    ../Representation/Observation.cpp \
    ../Representation/AugmentedObservation.cpp \
    ../Representation/SequentialReward.cpp \
    ../Representation/EnumeratedReward.cpp \
    ../Representation/Instance.cpp \
    ../util/KolmogorovSmirnovTest.cpp \
    ../util/ChiSquareTest.cpp \
    ../util/ProgressBar.cpp \
    ../DelayDistribution.cpp \
    ../HistoryObserver.cpp \
    ../qcustomplot.cpp \
    ../SmoothingKernelSigmoid.cpp \
    ../QtUtil.cpp \
    ../optimization/LBFGS_Optimizer.cpp \
    FeatureTest.cpp \
    AbstractIteratableSpaceTest.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp

