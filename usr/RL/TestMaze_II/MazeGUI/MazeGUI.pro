CONFIG += debug
TEMPLATE = app
TARGET = MazeGUI
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    ../Config.h \
    \
    ../util/util.h \
    ../util/Macro.h \
    ../util/ProgressBar.h \
    ../util/ColorOutput.h \
    ../util/QtUtil.h \
    ../util/lbfgs_codes.h \
    ../util/KolmogorovSmirnovDist.h \
    ../util/KolmogorovSmirnovTest.h \
    \
    ../SmoothingKernelSigmoid.h \
    ../qcustomplot.h \
    ../DelayDistribution.h \
    \
    ../optimization/LBFGS_Optimizer.h \
    \
    ../Representation/Feature.h \
    \
    ../Representation/AbstractInstance.h \
    ../Representation/DoublyLinkedInstance.h \
    ../Representation/AbstractAction.h \
    ../Representation/AbstractObservation.h \
    ../Representation/AbstractReward.h \
    ../Representation/ListedReward.h \
    \
    ../Predictor.h \
    ../Environment.h \
    ../PredictiveEnvironment.h \
    ../Visualizer.h \
    ../HistoryObserver.h \
    ../Learner/FeatureLearner.h \
    ../SpaceManager.h \
    \
    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../Maze/MazeObservation.h \
    ../CheeseMaze/CheeseMaze.h \
    ../CheeseMaze/CheeseMazeAction.h \
    ../CheeseMaze/CheeseMazeObservation.h \
    \
    ../Planning/Policy.h \
    ../Planning/RandomPolicy.h \
    ../Planning/LookAheadSearch.h \
    ../Planning/LookAheadPolicy.h \
    ../Planning/GoalIteration.h \
    \
    ../Learner/KMarkovCRF.h \
    ../Learner/UTree.h \
    ../Learner/LinearQ.h \
    \
    ../BatchMaze.h \
    ../testmaze_ii.h
SOURCES += \
    main.cpp \
    \
    ../util/util.cpp \
    ../util/ProgressBar.cpp \
    ../util/ColorOutput.cpp \
    ../util/QtUtil.cpp \
    ../util/lbfgs_codes.cpp \
    ../util/KolmogorovSmirnovDist.cpp \
    ../util/KolmogorovSmirnovTest.cpp \
    \
    ../SmoothingKernelSigmoid.cpp \
    ../qcustomplot.cpp \
    ../DelayDistribution.cpp \
    \
    ../optimization/LBFGS_Optimizer.cpp \
    \
    ../Representation/Feature.cpp \
    \
    ../Representation/AbstractInstance.cpp \
    ../Representation/DoublyLinkedInstance.cpp \
    ../Representation/AbstractAction.cpp \
    ../Representation/AbstractObservation.cpp \
    ../Representation/AbstractReward.cpp \
    ../Representation/ListedReward.cpp \
    \
    ../PredictiveEnvironment.cpp \
    ../Visualizer.cpp \
    ../HistoryObserver.cpp \
    ../Learner/FeatureLearner.cpp \
    ../SpaceManager.cpp \
    \
    ../Maze/Maze.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp \
    ../Maze/MazeObservation.cpp \
    ../CheeseMaze/CheeseMaze.cpp \
    ../CheeseMaze/CheeseMazeAction.cpp \
    ../CheeseMaze/CheeseMazeObservation.cpp \
    \
    ../Planning/LookAheadSearch.cpp \
    ../Planning/LookAheadPolicy.cpp \
    ../Planning/GoalIteration.cpp \
    \
    ../Learner/KMarkovCRF.cpp \
    ../Learner/UTree.cpp \
    ../Learner/LinearQ.cpp \
    \
    ../BatchMaze.cpp \
    ../testmaze_ii.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp
INCLUDEPATH +=
LIBPATH += /usr/share/lib
FORMS += ../testmaze_ii.ui
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp
QMAKE_CC = $$system( if [ `which gcc-4.8 2>/dev/null` ]; then echo "gcc-4.8"; else echo "gcc"; fi )
QMAKE_CXX = $$system( if [ `which g++-4.8 2>/dev/null` ]; then echo "g++-4.8"; else echo "g++"; fi )
message("Project $$TARGET uses:")
message("--> $$QMAKE_CC")
message("--> $$QMAKE_CXX")
