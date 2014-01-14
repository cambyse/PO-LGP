CONFIG += debug
TARGET = GTest
QT += core \
    gui \
    svg \
    printsupport
HEADERS += \
    ../Config.h \
    \
    ../util.h \
    ../util/Macro.h \
    ../util/ProgressBar.h \
    ../util/ColorOutput.h \
    ../util/QtUtil.h \
    ../util/lbfgs_codes.h \
    ../util/KolmogorovSmirnovDist.h \
    ../util/KolmogorovSmirnovTest.h \
    \
    ../optimization/LBFGS_Optimizer.h \
    \
    ../Feature.h \
    ../Instance.h \
    ../AbstractAction.h \
    ../AbstractObservation.h \
    ../AbstractReward.h \
    ../ListedReward.h \
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
    ../LookAheadSearch.h \
    \
    ../HistoryObserver.h \
    ../FeatureLearner.h \
    ../KMarkovCRF.h \
    ../UTree.h \
    ../LinearQ.h \
    \
    MinimalEnvironmentExample/MinimalAction.h \
    MinimalEnvironmentExample/MinimalObservation.h \
    MinimalEnvironmentExample/MinimalReward.h \
    MinimalEnvironmentExample/MinimalEnvironment.h \
    RandomElements.h
SOURCES += \
    main.cpp \
    \
    ../util.cpp \
    ../util/ProgressBar.cpp \
    ../util/ColorOutput.cpp \
    ../util/QtUtil.cpp \
    ../util/lbfgs_codes.cpp \
    ../util/KolmogorovSmirnovDist.cpp \
    ../util/KolmogorovSmirnovTest.cpp \
    \
    ../optimization/LBFGS_Optimizer.cpp \
    \
    ../Feature.cpp \
    ../Instance.cpp \
    ../AbstractAction.cpp \
    ../AbstractObservation.cpp \
    ../AbstractReward.cpp \
    ../ListedReward.cpp \
    \
    ../Environment.cpp \
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
    ../LookAheadSearch.cpp \
    \
    ../HistoryObserver.cpp \
    ../FeatureLearner.cpp \
    ../KMarkovCRF.cpp \
    ../UTree.cpp \
    ../LinearQ.cpp \
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
QMAKE_CC = $$system( if [ "$(which gcc-4.8)"!="" ]; then echo "gcc-4.8"; else echo "gcc"; fi )
QMAKE_CXX = $$system( if [ "$(which g++-4.8)"!="" ]; then echo "g++-4.8"; else echo "g++"; fi )
message("Project $$TARGET uses:")
message("--> $$QMAKE_CC")
message("--> $$QMAKE_CXX")
