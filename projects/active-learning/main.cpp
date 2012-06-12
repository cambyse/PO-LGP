#define LOG_STRING (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line <<  " | "  << name << " (" << level_str[0] << ") | " << current_time << " | " << msg << " ]" <<  std::endl; 
#include <MT/opengl.h>

#include <JK/active_learning/al_logistic_regression.h>
#include <JK/active_learning/al_gaussian_process.h>
#include <JK/active_learning/al_grounded_symbol.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_gui.h>
#include <JK/active_learning/al_process.h>
#include <JK/active_learning/al_tester.h>

#include <JK/utils/oracle.h>
#include <JK/utils/sampler.h>
#include <biros/logging.h>

//#include <relational/robotManipulationSimulator.h>
//#include <JK/al/al_logistic_regression.h>
//#include "naiveBayesClassificator.h"
//#include "gaussProcClassificator.h"
//#include "dataReader.h"
//#include "activeLearningProcess.h"
//#include "oracle.h"
//#include "tester.h"
//#include "sampler.h"
//#include "gui.h"

#include <MT/array_t.cxx>
#include <MT/ors.h>

#include <csignal>

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}
int main(int argc, char** argv) {
  ClassifyData d;
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  double seed = MT::getParameter<double>("seed", time(NULL));
  int n_steps = MT::getParameter<int>("steps", 20);
  cout << n_steps << endl;

  srand(seed);

  MT::String filename =  MT::getParameter<MT::String>("dataFile", MT::String("classification.data"));
  bool gaussproc = MT::getParameter<bool>("gauss", true);

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
  if(gaussproc) 
    cl.classificator = new GaussianProcessAL(new BlocksWorldSampler);
  else
    cl.classificator = new LogisticRegression(new BlocksWorldSampler);
  cl.oracle = new OnOracle();
  cl.tester = new Tester(5000, filename);

  DEBUG_VAR(classify, d.get_numOfWorkingJobs(NULL));
  DEBUG_VAR(classify, &d);

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
  //alp.guiData = &guiData;
 
  alp.threadOpen();
  alp.threadLoop();

  while(true) {
    MT::wait(.1);
  }

  //gui.threadOpen();
  //gui.threadLoop();


  alp.threadClose();
}


