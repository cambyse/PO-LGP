CONFIG += debug
TARGET = GTest
QT += core \
    gui \
    svg \
    printsupport
    HEADERS += \
    ../Config.h \
    ../util.h \
    ../Feature.h \
    ../Instance.h \
    ../AbstractAction.h \
    ../AbstractObservation.h \
    ../AbstractReward.h \
    ../ListedReward.h \
    ../PredictiveEnvironment.h \
    ../FeatureLearner.h \
    ../VisualEnvironment.h \
#    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../AbstractObservation.h \
    ../Maze/MazeObservation.h
SOURCES += \
    main.cpp \
    ../util.cpp \
    ../Feature.cpp \
    ../Instance.cpp \
    ../AbstractAction.cpp \
    ../AbstractObservation.cpp \
    ../AbstractReward.cpp \
    ../ListedReward.cpp \
    ../FeatureLearner.cpp \
    ../VisualEnvironment.cpp \
#    ../Maze/Maze.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp \
    ../Maze/MazeObservation.cpp \
    PlayGround.cpp \
    AbstractIteratableSpaceTest.cpp \
    FeatureTest.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp

