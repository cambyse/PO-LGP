#include <MT/opengl.h>

#include "naiveBayesClassificator.h"
#include "dataReader.h"
#include "activeLearningProcess.h"
#include "oracle.h"
#include "tester.h"

#include <MT/array_t.cpp>
#include <MT/ors.h>

int main(int argn, char** argv) {
  
  DataReader d;

	d.readDataFile("s120116-1029-0.dat", "s120116-1029-0.dat.rel");
  d.readDataFile("s120116-1029-1.dat", "s120116-1029-1.dat.rel");
	d.readDataFile("s120116-1029-2.dat", "s120116-1029-2.dat.rel");
	d.readDataFile("s120116-1029-3.dat", "s120116-1029-3.dat.rel");
	d.readDataFile("s120116-1029-4.dat", "s120116-1029-4.dat.rel");
  d.readDataFile("s120116-1029-5.dat", "s120116-1029-5.dat.rel");
  d.readDataFile("s120116-1029-6.dat", "s120116-1029-6.dat.rel");
  d.readDataFile("s120116-1029-7.dat", "s120116-1029-7.dat.rel");
	d.readDataFile("s120116-1029-8.dat", "s120116-1029-8.dat.rel");
  d.readDataFile("s120116-1029-9.dat", "s120116-1029-9.dat.rel");

  TrainingsDataV train;

  train.data = d.getData();
  train.classes = d.getClasses();

  ClassificatorV cl;
  cl.classificator = new NaiveBayesClassificator(new BlocksWorldSampler);
  cl.oracle = new OnOracle();
  cl.tester = new Tester(5000);

  ActiveLearningP alp;

  alp.traindata = &train;
  alp.classificator = &cl;

  alp.threadOpen();
  alp.threadLoop();
  while (true) { 
    sleep(1.);
  }
}
