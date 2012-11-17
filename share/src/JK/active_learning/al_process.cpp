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
  CHECK(al, "No classificator available for active learner.");
  
  al->writeAccess(this);
  al->classificator->setTrainingsData(traindata->data, traindata->classes);
  al->deAccess(this);
}

void ActiveLearningP::step() {
  std::ofstream os("gp.data", std::ios_base::app);
  MT::Array<arr> sample;
  al->writeAccess(this);
  if (al->classificator->nextSample(sample)) {
    int _class = al->classificator->problem.oracle->classify(sample); 
    al->classificator->addData(sample, _class);
    os << sample << _class << std::endl;
    std::cout << "Add sample as " << _class << std::endl;
  }
  else {
    std::cout << "Nothing interesting" << std::endl;  
  }
  test(MT::getParameter<uint>("numTests", 5000), al->classificator, al->problem, MT::getParameter<MT::String>("dataFile", MT::String("classification.data")));
  al->deAccess(this);
  if(guiData) {
    guiData->writeAccess(this);
    guiData->sample = new MT::Array<arr>(sample);
    guiData->deAccess(this);
  }
}

void ActiveLearningP::close() {
    
}

TrainingsDataV::TrainingsDataV() : Variable("Trainings Data Variable") {}
ActiveLearningV::ActiveLearningV() : Variable("Classificator Variable") {}

