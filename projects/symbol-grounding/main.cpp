#include <MT/opengl.h>

#include <JK/active_learning/al_logistic_regression.h>
#include <JK/active_learning/al_gaussian_process.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_gui.h>
#include <JK/active_learning/al_process.h>
#include <JK/utils/oracle.h>
#include <JK/utils/sampler.h>

#include <relational/RobotManipulationSimulator.h>

#include <MT/array_t.cpp>
#include <MT/ors.h>

#include <csignal>

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}

void init_grounded_symbol(GroundedSymbolAL& gs, ActiveLearner& al) {
  bool gaussproc = MT::getParameter<bool>("gauss", true);
  int n_steps = MT::getParameter<int>("steps", 20);
	
	BlocksWorldSampler sampler;
  OnOracle o;

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
  cl.oracle = &o;

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
 
  alp.threadOpen();
  alp.threadSteps(n_steps);
  alp.threadClose();

	gs
}

int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  double seed = MT::getParameter<double>("seed", time(NULL));

	srand(seed);

  RobotManipulationSimulator sim;
  sim.shutdownAll();
  sim.loadConfiguration("situation.ors");
  sim.startOde();
  sim.startSwift();
  sim.simulate(50);




}


