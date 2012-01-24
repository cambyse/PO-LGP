#include "activeLearningProcess.h"
#include "naiveBayesClassificator.h"
#include "dataReader.h"
#include "blocksWorld.h"

#include <MT/array.h>
#include <MT/array_t.cpp>
#include <JK/util.h>

class sActiveLearningProcess {
  public:
    NaiveBayesClassificator cl;
    DataReader d;
    int questions;
};

LearningDataVariable::LearningDataVariable() : Variable("Learning Data Variable") {
  pos1 = ARR(0.,0.,0.);
  pos2 = ARR(0.,0.,0.);
}

ActiveLearningProcess::ActiveLearningProcess() : Process("Active Learning Process") {}

void ActiveLearningProcess::open() {
  s = new sActiveLearningProcess;

	s->d.readDataFile("s120116-1029-0.dat", "s120116-1029-0.dat.rel");
  s->d.readDataFile("s120116-1029-1.dat", "s120116-1029-1.dat.rel");
	s->d.readDataFile("s120116-1029-2.dat", "s120116-1029-2.dat.rel");
	s->d.readDataFile("s120116-1029-3.dat", "s120116-1029-3.dat.rel");
	s->d.readDataFile("s120116-1029-4.dat", "s120116-1029-4.dat.rel");
  s->d.readDataFile("s120116-1029-5.dat", "s120116-1029-5.dat.rel");
  s->d.readDataFile("s120116-1029-6.dat", "s120116-1029-6.dat.rel");
  s->d.readDataFile("s120116-1029-7.dat", "s120116-1029-7.dat.rel");
	s->d.readDataFile("s120116-1029-8.dat", "s120116-1029-8.dat.rel");
  s->d.readDataFile("s120116-1029-9.dat", "s120116-1029-9.dat.rel");

  s->cl.setTrainingsData(s->d.getData(), s->d.getClasses());
  JK_DEBUG(s->d.getData());
  JK_DEBUG(s->d.getClasses());
}

void ActiveLearningProcess::step() {
  s->questions++;

  MT::Array<arr> sample;
  s->cl.nextSample(sample);

  data->writeAccess(this);
  data->pos1 = sample(0);
  data->pos2 = sample(1);
  data->deAccess(this);

  JK_DEBUG(sample);
  std::cout << "Does 'on' hold? [y, n]" <<std::endl;
  char answer;
  std::cin >> answer;

  if (answer == 'y') { s->cl.addData(sample, s->d.getClass("on")); }
  else { s->cl.addData(sample, s->d.getClass("noton"));}

  if (s -> questions % 10 == 0) {
    int correct = 0;
    std::cout << "on is " << s->d.getClass("on") << std::endl;
    std::cout << "noton is " << s->d.getClass("noton") << std::endl;
    for (uint i = 0; i < 200; ++i) {
      generateBlocksSample(sample, 2);
      int classified = s->cl.classify(sample);
      if((sample(1)(2) == 0.848 && classified == s->d.getClass("on")) ||
         (sample(1)(2) == 0.74 && classified == s->d.getClass("noton"))){
        correct++; 
      }
    }
    JK_DEBUG(correct/200.);
    data->writeAccess(this);
    data->pos1 = sample(0);
    data->pos2 = sample(1);
    data->deAccess(this);
    char unused;
    std::cin >> unused;
  }
}

void ActiveLearningProcess::close() {
  delete s;  
}

