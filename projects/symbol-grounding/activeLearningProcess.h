#ifndef ACTIVE_LEARNING_PROCESS_H_
#define ACTIVE_LEARNING_PROCESS_H_

#include <MT/process.h>
#include <MT/ors.h>

class LearningDataVariable : public Variable {
  public:
    LearningDataVariable();
    ors::Vector pos1;
    ors::Vector pos2;
};

class sActiveLearningProcess;

class ActiveLearningProcess : public Process {
  private:
    sActiveLearningProcess* s;

  public:
    ActiveLearningProcess();

    void open();
    void step();
    void close();

    LearningDataVariable* data; 
};

#endif


