#include "activeLearningProcess.h"

#include "activeLearner.h"
#include "oracle.h"

ActiveLearningP::ActiveLearningP() : Process("Active Learning Process") {}

void ActiveLearningP::open() {
  CHECK(traindata, "No trainingsdata available for active learner.");
  CHECK(classificator, "No classificator available for active learner.");
  
  classificator->writeAccess(this);
  classificator->cl->setTrainingsData(traindata->data, traindata->classes);
  classificator->deAccess(this);
}

void ActiveLearningP::step() {
  MT::Array<arr> sample;
  classificator->writeAccess(this);
  if (classificator->cl->nextSample(sample)) {
    int _class = classificator->o->classify(sample); 
    classificator->cl->addData(sample, _class);
    std::cout << "Add sample as " << _class << std::endl;
  }
  else {
    std::cout << "Nothing interesting" << std::endl;  
  }
  classificator->deAccess(this);
}

void ActiveLearningP::close() {
    
}

TrainingsDataV::TrainingsDataV() : Variable("Trainings Data Variable") {}
ClassificatorV::ClassificatorV() : Variable("Classificator Variable") {}

