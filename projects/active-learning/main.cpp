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
SET_LOG(main, DEBUG);

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

  TraySampler sampler;
  InsideOracle o;

  Gui gui(MT::getParameter<MT::String>("orsFile", MT::String("schunk-armani.ors")));
  GuiDataV guiData;
  gui.guiData = &guiData;

  TrainingsDataV train;
  //MT::Array<arr> sample;
  //ifstream is("samples.data");
  //for (int i=0; i<MT::getParameter<int>("sample_number"); ++i) {
    //is >> sample;  
  //}
  //train.data = sample;
  //train.data.reshape(1,4);
  do {
    sampler.sample(train.data);  
  } while (!o.classify(train.data, 0)) ;
  //train.data.append(ARR(0.159201, -1.00565, 0.838));
  //train.data.append(ARR( 0.08)); 
  //train.data.append(ARR(0.157674, -1.00132, 0.73 ));
  //train.data.append(ARR(0.08));
  train.data.reshape(1,3);
  DEBUG_VAR(main, train.data);
  DEBUG_VAR(main, o.classify(train.data, 0));
  intA classes;
  classes.append(o.classify(train.data, 0));
  train.classes = classes;

  char unused;
  guiData.sample = &train.data;

  gui.threadOpen();
  gui.threadLoop();
  //std::cin >> unused;

  ClassificatorV cl;
  if(gaussproc) 
    cl.classificator = new GaussianProcessAL(&sampler);
  else
    cl.classificator = new LogisticRegression(&sampler);
  cl.oracle = new InsideOracle();
  cl.tester = new Tester(5000, filename, 24, &sampler);

  DEBUG_VAR(classify, d.get_numOfWorkingJobs(NULL));
  DEBUG_VAR(classify, &d);

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
  alp.guiData = &guiData;
 
  alp.threadOpen();
  alp.threadStep();



  alp.threadStep(n_steps);


  alp.threadClose();
  gui.threadClose();
}


