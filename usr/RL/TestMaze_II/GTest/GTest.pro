CONFIG += debug
TARGET = GTest
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
    ../util/debug.h \
    ../util/debug_exclude.h \
    \
    ../optimization/LBFGS_Optimizer.h \
    ../optimization/LBFGS_Object.h \
    \
    ../Representation/Feature.h \
    ../Instance.h \
    ../Representation/AbstractInstance.h \
    ../Representation/AbstractAction.h \
    ../Representation/AbstractObservation.h \
    ../Representation/AbstractReward.h \
    ../Representation/ListedReward.h \
    \
    ../Predictor.h \
    ../Environment.h \
    ../PredictiveEnvironment.h \
    ../Visualizer.h \
    ../Maze/Maze.h \
    ../Maze/MazeAction.h \
    ../Maze/AugmentedMazeAction.h \
    ../Maze/MazeObservation.h \
    ../CheeseMaze/CheeseMaze.h \
    ../CheeseMaze/CheeseMazeAction.h \
    ../CheeseMaze/CheeseMazeObservation.h \
    \
    ../Planning/Policy.h \
    ../Planning/LookAheadSearch.h \
    ../Planning/LookAheadPolicy.h \
    \
    ../HistoryObserver.h \
    ../SpaceManager.h \
    \
    ../Learner/FeatureLearner.h \
    ../Learner/KMarkovCRF.h \
    ../Learner/UTree.h \
    ../Learner/LinearQ.h \
    \
    MinimalEnvironmentExample/MinimalAction.h \
    MinimalEnvironmentExample/MinimalObservation.h \
    MinimalEnvironmentExample/MinimalReward.h \
    MinimalEnvironmentExample/MinimalEnvironment.h \
    RandomElements.h
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
    ../optimization/LBFGS_Optimizer.cpp \
    ../optimization/LBFGS_Object.cpp \
    \
    ../Representation/Feature.cpp \
    ../Instance.cpp \
    ../Representation/AbstractInstance.cpp \
    ../Representation/AbstractAction.cpp \
    ../Representation/AbstractObservation.cpp \
    ../Representation/AbstractReward.cpp \
    ../Representation/ListedReward.cpp \
    \
    ../PredictiveEnvironment.cpp \
    ../Visualizer.cpp \
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
    \
    ../HistoryObserver.cpp \
    ../SpaceManager.cpp \
    \
    ../Learner/FeatureLearner.cpp \
    ../Learner/KMarkovCRF.cpp \
    ../Learner/UTree.cpp \
    ../Learner/LinearQ.cpp \
    \
    MinimalEnvironmentExample/MinimalAction.cpp \
    MinimalEnvironmentExample/MinimalObservation.cpp \
    MinimalEnvironmentExample/MinimalReward.cpp \
    RandomElements.cpp \
    \
    RepresentationTest.cpp \
    FeatureTest.cpp \
    EnvironmentTest.cpp \
    PlannerTest.cpp \
    LearnerTest.cpp \
    LBFGSTest.cpp \
    \
#    TemplateTest.cpp \
    SandBox.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
    -fopenmp
QMAKE_CC = $$system( if [ `which gcc-4.8 2>/dev/null` ]; then echo "gcc-4.8"; else echo "gcc"; fi )
QMAKE_CXX = $$system( if [ `which g++-4.8 2>/dev/null` ]; then echo "g++-4.8"; else echo "g++"; fi )
message("Project $$TARGET uses:")
message("--> $$QMAKE_CC")
message("--> $$QMAKE_CXX")
