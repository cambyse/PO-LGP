#ifndef ACTIVE_LEARNING_PROCESS_H_
#define ACTIVE_LEARNING_PROCESS_H_

#include <MT/process.h>
#include <MT/ors.h>

class ActiveLearner;
class Oracle;
class TrainingsDataV: public Variable {
  public:
    TrainingsDataV();
    MT::Array<arr> data;
    intA classes;
};

class ClassificatorV: public Variable {
  public:
    ClassificatorV();
    ActiveLearner* cl;
    Oracle* o;
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

};

#endif


