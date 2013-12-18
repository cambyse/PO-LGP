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
    ../Feature.h \
    ../Instance.h \
    ../AbstractAction.h \
    ../AbstractObservation.h \
    ../AbstractReward.h \
    ../ListedReward.h \
    ../PredictiveEnvironment.h \
    ../FeatureLearner.h \
    ../VisualEnvironment.h \
    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../AbstractObservation.h \
    ../Maze/MazeObservation.h \
    ../LookAheadSearch.h \
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
    ../Feature.cpp \
    ../Instance.cpp \
    ../AbstractAction.cpp \
    ../AbstractObservation.cpp \
    ../AbstractReward.cpp \
    ../ListedReward.cpp \
    ../PredictiveEnvironment.cpp \
    ../FeatureLearner.cpp \
    ../VisualEnvironment.cpp \
    ../Maze/Maze.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp \
    ../Maze/MazeObservation.cpp \
    ../LookAheadSearch.cpp \
    AbstractIteratableSpaceTest.cpp \
    FeatureTest.cpp \
#    TemplateTest.cpp \
    EnvironmentTest.cpp \
    PlannerTest.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
#LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp

