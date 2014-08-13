CONFIG += debug
TEMPLATE = app
TARGET = BatchWorker_quiet
DEFINES += BATCH_MODE_QUIET
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    ../Config.h \
    \
    ../util/util.h \
    ../util/ProgressBar.h \
    ../util/ColorOutput.h \
    ../util/QtUtil.h \
    ../util/lbfgs_codes.h \
    ../util/KolmogorovSmirnovDist.h \
    ../util/KolmogorovSmirnovTest.h \
    ../util/debug.h \
    ../util/debug_exclude.h \
    \
    ../optimization/LBFGS_Optimizer.h \
    ../optimization/LBFGS_Object.h \
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
    ../SpaceManager.h \
    \
    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../Maze/MazeObservation.h \
    ../CheeseMaze/CheeseMaze.h \
    ../CheeseMaze/CheeseMazeAction.h \
    ../CheeseMaze/CheeseMazeObservation.h \
    ../ButtonWorld/ButtonWorld.h \
    ../ButtonWorld/ButtonAction.h \
    ../ButtonWorld/ButtonObservation.h \
    \
    ../Planning/Policy.h \
    ../Planning/RandomPolicy.h \
    ../Planning/LookAheadSearch.h \
    ../Planning/LookAheadPolicy.h \
    \
    ../Learner/FeatureLearner.h \
    ../Learner/UTree.h \
    ../Learner/TemporallyExtendedModel.h \
    ../Learner/TemporallyExtendedFeatureLearner.h \
    ../Learner/TemporallyExtendedLinearQ.h \
    ../Learner/AdjacencyOperator.h \
    ../Learner/ConjunctiveAdjacency.h \
    \
    ../BatchWorker.h
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
    ../util/Commander.cpp \
    \
    ../optimization/LBFGS_Optimizer.cpp \
    ../optimization/LBFGS_Object.cpp \
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
    ../Predictor.cpp \
    ../PredictiveEnvironment.cpp \
    ../Visualizer.cpp \
    ../HistoryObserver.cpp \
    ../SpaceManager.cpp \
    \
    ../Maze/Maze.cpp \
    ../Maze/MazeAction.cpp \
    ../Maze/AugmentedMazeAction.cpp \
    ../Maze/MazeObservation.cpp \
    ../CheeseMaze/CheeseMaze.cpp \
    ../CheeseMaze/CheeseMazeAction.cpp \
    ../CheeseMaze/CheeseMazeObservation.cpp \
    ../ButtonWorld/ButtonWorld.cpp \
    ../ButtonWorld/ButtonAction.cpp \
    ../ButtonWorld/ButtonObservation.cpp \
    \
    ../Planning/LookAheadSearch.cpp \
    ../Planning/LookAheadPolicy.cpp \
    \
    ../Learner/FeatureLearner.cpp \
    ../Learner/UTree.cpp \
    ../Learner/TemporallyExtendedModel.cpp \
    ../Learner/TemporallyExtendedFeatureLearner.cpp \
    ../Learner/TemporallyExtendedLinearQ.cpp \
    ../Learner/ConjunctiveAdjacency.cpp \
    \
    ../BatchWorker.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -llapack \
    -lgomp \
    -lgsl \
    -lgslcblas
INCLUDEPATH +=
LIBPATH += /usr/share/lib
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp
QMAKE_CC = $$system( if [ `which gcc-4.8 2>/dev/null` ]; then echo "gcc-4.8"; else echo "gcc"; fi )
QMAKE_CXX = $$system( if [ `which g++-4.8 2>/dev/null` ]; then echo "g++-4.8"; else echo "g++"; fi )
message("Project $$TARGET uses:")
message("--> $$QMAKE_CC")
message("--> $$QMAKE_CXX")
