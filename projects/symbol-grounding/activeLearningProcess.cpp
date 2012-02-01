#include "activeLearningProcess.h"

#include "activeLearner.h"
#include "oracle.h"
#include "tester.h"

ActiveLearningP::ActiveLearningP() : Process("Active Learning Process") {}

void ActiveLearningP::open() {
  CHECK(traindata, "No trainingsdata available for active learner.");
  CHECK(classificator, "No classificator available for active learner.");
  
  classificator->writeAccess(this);
  classificator->classificator->setTrainingsData(traindata->data, traindata->classes);
  classificator->deAccess(this);
}

void ActiveLearningP::step() {
  MT::Array<arr> sample;
  classificator->writeAccess(this);
  if (classificator->classificator->nextSample(sample)) {
    int _class = classificator->oracle->classify(sample); 
    classificator->classificator->addData(sample, _class);
    std::cout << "Add sample as " << _class << std::endl;
  }
  else {
    std::cout << "Nothing interesting" << std::endl;  
  }
  if (classificator->tester) classificator->tester->test(classificator);
  classificator->deAccess(this);
}

void ActiveLearningP::close() {
    
}

TrainingsDataV::TrainingsDataV() : Variable("Trainings Data Variable") {}
ClassificatorV::ClassificatorV() : Variable("Classificator Variable"), tester(NULL) {}

