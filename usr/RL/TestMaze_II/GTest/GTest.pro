CONFIG += debug
TARGET = GTest
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    ../Config.h \
    ../util.h \
    ../util/ProgressBar.h \
    ../util/ColorOutput.h \
    ../util/QtUtil.h \
    ../util/lbfgs_codes.h \
    ../util/KolmogorovSmirnovDist.h \
    ../util/KolmogorovSmirnovTest.h \
    \
    ../Feature.h \
    ../Instance.h \
    ../AbstractAction.h \
    ../AbstractObservation.h \
    ../AbstractReward.h \
    ../ListedReward.h \
    \
    ../PredictiveEnvironment.h \
    ../VisualEnvironment.h \
    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../AbstractObservation.h \
    ../Maze/MazeObservation.h \
    \
    ../LookAheadSearch.h \
    \
    ../HistoryObserver.h \
    ../FeatureLearner.h \
    ../KMarkovCRF.h \
    ../UTree.h \
    MinimalEnvironmentExample/MinimalAction.h \
    MinimalEnvironmentExample/MinimalObservation.h \
    MinimalEnvironmentExample/MinimalReward.h \
    MinimalEnvironmentExample/MinimalEnvironment.h
SOURCES += \
    main.cpp \
    ../util.cpp \
    ../util/ProgressBar.cpp \
    ../util/ColorOutput.cpp \
    ../util/QtUtil.cpp \
    ../util/lbfgs_codes.cpp \
    ../util/KolmogorovSmirnovDist.cpp \
    ../util/KolmogorovSmirnovTest.cpp \
    \
    ../Feature.cpp \
    ../Instance.cpp \
    ../AbstractAction.cpp \
    ../AbstractObservation.cpp \
    ../AbstractReward.cpp \
    ../ListedReward.cpp \
    \
    ../PredictiveEnvironment.cpp \
    ../VisualEnvironment.cpp \
    ../Maze/Maze.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp \
    ../Maze/MazeObservation.cpp \
    \
    ../LookAheadSearch.cpp \
    \
    ../HistoryObserver.cpp \
    ../FeatureLearner.cpp \
    ../KMarkovCRF.cpp \
    ../UTree.cpp \
    \
    AbstractIteratableSpaceTest.cpp \
    FeatureTest.cpp \
#    TemplateTest.cpp \
    EnvironmentTest.cpp \
    PlannerTest.cpp \
    LearnerTest.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp

