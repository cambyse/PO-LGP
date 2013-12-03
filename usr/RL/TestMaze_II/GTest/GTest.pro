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
    ../Representation/Action.h \
    ../Representation/Observation.h \
    ../Representation/AugmentedObservation.h \
    ../Representation/SequentialReward.h \
    ../Representation/EnumeratedReward.h \
    ../Representation/Instance.h \
    ../AbstractAction.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h
SOURCES += \
    main.cpp \
    ../Config.cpp \
    ../util.cpp \
    ../Feature.cpp \
    ../Representation/Action.cpp \
    ../Representation/Observation.cpp \
    ../Representation/AugmentedObservation.cpp \
    ../Representation/SequentialReward.cpp \
    ../Representation/EnumeratedReward.cpp \
    ../Representation/Instance.cpp \
    FeatureTest.cpp \
    AbstractIteratableSpaceTest.cpp \
    PlayGround.cpp \
    ../AbstractAction.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp

