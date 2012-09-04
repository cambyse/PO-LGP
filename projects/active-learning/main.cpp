#define LOG_STRING (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line <<  " | "  << name << " (" << level_str[0] << ") | " << current_time << " | " << msg << " ]" <<  std::endl; 

#include <JK/active_learning/al_logistic_regression.h>
#include <JK/active_learning/al_gaussian_process.h>
#include <JK/active_learning/al_grounded_symbol.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_gui.h>
#include <JK/active_learning/al_process.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_problem.h>

#include <JK/utils/oracle.h>
#include <JK/utils/sampler.h>
#include <JK/utils/featureGenerator.h>

#include <biros/logging.h>

#include <MT/opengl.h>
#include <MT/array_t.cxx>
#include <MT/ors.h>

#include <csignal>

SET_LOG(main, DEBUG);

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}
int main(int argc, char** argv) {
  ClassifyData d;
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  double seed = MT::getParameter<double>("seed", time(NULL));
  srand(seed);

  int n_steps = MT::getParameter<int>("steps", 20);
  MT::String filename =  MT::getParameter<MT::String>("dataFile", MT::String("classification.data"));
  bool gaussproc = MT::getParameter<bool>("gauss", true);
  bool pause = MT::getParameter<bool>("pause", false);
  MT::String problem_name = MT::getParameter<MT::String>("problem", MT::String("tray"));

  ActiveLearningProblem prob;
  if(problem_name == "tray") {
    prob.sampler = new TraySampler;
    prob.oracle  = new InsideOracle;
    prob.generator = new TrayFeatureGenerator;
    INFO(main, "Start tray problem.");
  }
  else if(problem_name == "tower") {
    prob.sampler = new BlocksWorldSampler;
    prob.oracle  = new OnOracle;
    prob.generator = new DistanceFeatureGenerator;
    INFO(main, "Start stack problem.");
  }
  else if(problem_name == "close") {
    prob.sampler = new BlocksWorldSampler;
    prob.oracle  = new CloseOracle;
    prob.generator = new DistanceFeatureGenerator;
    INFO(main, "Start distance problem.");
  }
  else if(problem_name == "outOfReach") {
    prob.sampler = new OutOfReachSampler;
    prob.oracle  = new OutOfReachOracle;
    prob.generator = new SimpleFeatureGenerator;
    INFO(main, "Start out of reach problem.");
  }
  

  Gui gui(MT::getParameter<MT::String>("orsFile", MT::String("schunk-armani.ors")));
  GuiDataV guiData;
  gui.guiData = &guiData;

  TrainingsDataV train;

  // VERSION 1: Read from file
  //MT::Array<arr> sample;
  //ifstream is("samples.data");
  //for (int i=0; i<MT::getParameter<int>("sample_number"); ++i) {
  //  is >> sample;  
  //}
  //train.data = sample;
  
  // VERSION 2: fixed sample
  //train.data.append(ARR(0.159201, -1.00565, 0.838));
  //train.data.append(ARR( 0.08)); 
  //train.data.append(ARR(0.157674, -1.00132, 0.73 ));
  //train.data.append(ARR(0.08));
  
  // VERSION 3: use sampler
  do {
    prob.sampler->sample(train.data);  
  } while (!prob.oracle->classify(train.data, 0)) ;

  uintA sample_size;
  prob.generator->getSize(sample_size);
  train.data.reshape(sample_size(0), sample_size(1));

  DEBUG_VAR(main, train.data);
  DEBUG_VAR(main, prob.oracle->classify(train.data, 0));

  intA classes;
  classes.append(prob.oracle->classify(train.data, 0));
  train.classes = classes;

  guiData.sample = &train.data;

  gui.threadOpen();
  gui.threadLoop();

  char unused;
  if (pause)
    std::cin >> unused;


  ClassificatorV cl;
  if(gaussproc) 
    cl.classificator = new GaussianProcessAL(prob);
  else
    cl.classificator = new LogisticRegression(prob);
  cl.tester = new Tester(5000, filename, 24, prob.sampler);

  DEBUG_VAR(classify, d.get_numOfWorkingJobs(NULL));
  DEBUG_VAR(classify, &d);

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
  alp.guiData = &guiData;
 
  alp.threadOpen();
  if (pause) {
    for (int s=0; s<n_steps; ++s) {
      alp.threadStep();
      std::cin >> unused;
    }
  }
  else alp.threadStep(n_steps);

  alp.threadClose();
  gui.threadClose();
}


