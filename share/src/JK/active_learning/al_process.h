#ifndef ACTIVE_LEARNING_PROCESS_H_
#define ACTIVE_LEARNING_PROCESS_H_

#include <biros/biros.h>
#include <MT/ors.h>

class ActiveLearner;
class Oracle;
class Tester;
class GuiDataV;
class TrainingsDataV: public Variable {
  public:
    TrainingsDataV();
    MT::Array<arr> data;
    intA classes;
};

class ClassificatorV: public Variable {
  public:
    ClassificatorV();
    ActiveLearner* classificator;
    Tester* tester;
};


class ActiveLearningP: public Process {
  public:
    ActiveLearningP();

    void open();
    void step();
    void close();

    ors::Graph* ors; 
    TrainingsDataV* traindata;
    ClassificatorV* classificator;
    GuiDataV* guiData;

};

#endif


