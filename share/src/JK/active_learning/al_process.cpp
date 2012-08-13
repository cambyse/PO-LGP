#include "al_process.h"

#include "al.h"
#include "al_tester.h"
#include "al_gui.h"

#include <JK/utils/oracle.h>

#include <iostream>

ActiveLearningP::ActiveLearningP() : Process("Active Learning Process"),
  guiData(NULL)
{}

void ActiveLearningP::open() {
  CHECK(traindata, "No trainingsdata available for active learner.");
  CHECK(classificator, "No classificator available for active learner.");
  
  classificator->writeAccess(this);
  classificator->classificator->setTrainingsData(traindata->data, traindata->classes);
  classificator->deAccess(this);
}

void ActiveLearningP::step() {
  std::ofstream os("gp.data", std::ios_base::app);
  MT::Array<arr> sample;
  classificator->writeAccess(this);
  if (classificator->classificator->nextSample(sample)) {
    int _class = classificator->classificator->problem.oracle->classify(sample); 
    classificator->classificator->addData(sample, _class);
    os << sample << _class << std::endl;
    std::cout << "Add sample as " << _class << std::endl;
  }
  else {
    std::cout << "Nothing interesting" << std::endl;  
  }
  if (classificator->tester) classificator->tester->test(classificator);
  classificator->deAccess(this);
  if(guiData) {
    guiData->writeAccess(this);
    guiData->sample = new MT::Array<arr>(sample);
    guiData->deAccess(this);
  }
}

void ActiveLearningP::close() {
    
}

TrainingsDataV::TrainingsDataV() : Variable("Trainings Data Variable") {}
ClassificatorV::ClassificatorV() : Variable("Classificator Variable"), tester(NULL) {}

