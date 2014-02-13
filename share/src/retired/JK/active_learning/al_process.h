#ifndef ACTIVE_LEARNING_PROCESS_H_
#define ACTIVE_LEARNING_PROCESS_H_

#include <System/biros.h>
#include <Ors/ors.h>

class ActiveLearner;
class ActiveLearningProblem;
class GuiDataV;
class TrainingsDataV: public Variable {
  public:
    TrainingsDataV();
    MT::Array<arr> data;
    intA classes;
};

class ActiveLearningV: public Variable {
  public:
    ActiveLearningV();
    ActiveLearner* classificator;
    ActiveLearningProblem* problem;
};


class ActiveLearningP: public Process {
  public:
    ActiveLearningP();

    void open();
    void step();
    void close();

    ors::KinematicWorld* ors; 
    TrainingsDataV* traindata;
    ActiveLearningV* al;
    GuiDataV* guiData;

};

#endif


