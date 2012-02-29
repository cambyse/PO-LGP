#include <MT/opengl.h>

//#include "logisticRegression.h"
#include "naiveBayesClassificator.h"
#include "gaussProcClassificator.h"
#include "dataReader.h"
#include "activeLearningProcess.h"
#include "oracle.h"
#include "tester.h"
#include "sampler.h"
#include "gui.h"

#include <MT/array_t.cpp>
#include <MT/ors.h>

#include <csignal>

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}
int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  BlocksWorldSampler sampler;
  OnOracle o;

  //Gui gui("situation.ors");
  //GuiDataV guiData;
  //gui.guiData = &guiData;

  TrainingsDataV train;
  sampler.sample(train.data);
  intA classes;
  classes.append(o.classify(train.data, 0));
  train.classes = classes;

  ClassificatorV cl;
  cl.classificator = new GaussianProcessAL(new BlocksWorldSampler);
  cl.oracle = new OnOracle();
  cl.tester = new Tester(50000);

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
  //alp.guiData = &guiData;
 
  alp.threadOpen();
  alp.threadLoop();

  //gui.threadOpen();
  //gui.threadLoop();

  MT::wait(60);

  alp.threadClose();
}

