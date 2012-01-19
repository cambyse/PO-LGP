#include "activeLearningProcess.h"
#include "naiveBayesClassificator.h"
#include "DataReader.h"

#include <MT/array.h>

class sActiveLearningProcess {
  public:
    NaiveBayesClassificator cl;
    DataReader d;
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
}

void ActiveLearningProcess::step() {
  MT::Array<arr> sample;
  s->cl.nextSample(sample);

  data->writeAccess(this);
  data->pos1 = sample(0);
  data->pos2 = sample(1);
  data->deAccess(this);

  std::cout << "Does 'on' hold? [y, n]" <<std::endl;
  char answer;
  std::cin >> answer;

  if (answer == 'y') { s->cl.addData(sample, s->d.getClass("on")); }
  else { s->cl.addData(sample, s->d.getClass("noton"));}
}

void ActiveLearningProcess::close() {
  delete s;  
}

