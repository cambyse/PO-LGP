#define LOG_STRING (level == ERROR ? std::cerr : std::cout) << "[@" << file << ":" << line << " | " << name << " | " << msg << " ]" << std::endl; 
#include <MT/opengl.h>

#include <JK/active_learning/al_logistic_regression.h>
#include <JK/active_learning/al_gaussian_process.h>
#include <JK/active_learning/al_grounded_symbol.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_gui.h>
#include <JK/active_learning/al_process.h>
#include <JK/utils/oracle.h>
#include <JK/utils/sampler.h>
#include <JK/utils/featureGenerator.h>

#include <relational/robotManipulationSimulator.h>
#include <relational/robotManipulationSymbols.h>
#include <relational/robotManipulationInterface.h>
#include <relational/literals.h>

#include <MT/array_t.cxx>
#include <MT/kin.h>

#include <csignal>
#include <iostream>
SET_LOG(main, DEBUG);

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}

void init_grounded_symbol(AL_GroundedSymbol& gs) {
  srand(mlr::getParameter<int>("seed", time(NULL)));
  bool gaussproc = mlr::getParameter<bool>("gauss", true);
  int n_steps = mlr::getParameter<int>("steps", 20);

  ActiveLearningProblem problem;
	//problem.sampler   = new BlocksWorldSampler;
  //problem.oracle    = new OnOracle;
  //problem.generator = new DistanceFeatureGenerator;

  problem.sampler   = new TraySampler;
  problem.oracle    = new InsideOracle;
  problem.generator = new TrayFeatureGenerator;

  TrainingsDataV train;
  do {
    problem.sampler->sample(train.data);
  } while (!problem.oracle->classify(train.data, 0));
  intA classes;
  classes.append(problem.oracle->classify(train.data, 0));
  train.classes = classes;

  ClassificatorV cl;
  if(gaussproc) 
    cl.classificator = new GaussianProcessAL(problem);
  else
    cl.classificator = new LogisticRegression(problem);

  ActiveLearningP alp;
  alp.traindata = &train;
  alp.classificator = &cl;
 
  alp.threadOpen();
  alp.threadStep(n_steps);
  alp.threadClose();

  gs.classificator = cl.classificator;
}

int main(int argc, char** argv) {
  mlr::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  double seed = mlr::getParameter<double>("seed", time(NULL));
	srand(seed);
  
  RobotManipulationSimulator sim;
  sim.shutdownAll();
  sim.loadConfiguration(mlr::getParameter<mlr::String>("orsFile"));
  sim.startOde();
  sim.startSwift();
  sim.simulate(150);


  // ------------------------------------
  //  SET UP GROUNDED SYMBOL
  //  -----------------------------------

  mlr::String relation("inside");
  AL_GroundedSymbol gs(relation, 2, false);
  init_grounded_symbol(gs);
  mlr::Array<relational::GroundedSymbol*> sgs;
  sgs.append(&gs);
  relational::LitL lits;
  relational::calculateSymbols(lits, sgs, sim.C);
  DEBUG(main, "test");
  DEBUG_VAR(main, lits); 
  

	// -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
 
  mlr::String symbolsFile_name("symbols.dat");
	cout<<endl<<endl;
	cout<<"SYMBOLS:"<<endl;
	cout<<"Reading symbols from file \""<<symbolsFile_name<<"\"..."<<flush;
  uintA simObjects;
  sim.getObjects(simObjects);
  DEBUG_VAR(main, simObjects);
  relational::SymL symbols;
  relational::ArgumentTypeL types;
  relational::readSymbolsAndTypes(symbols, types, symbolsFile_name);
  relational::reason::setConstants(simObjects);
  cout<<"done!"<<endl;
  
  relational::writeSymbolsAndTypes("used_symbols.dat");
  


  // -------------------------------------
  //   STATE
  // -------------------------------------
  cout<<"STARTING STATE:"<<endl;
  relational::SymbolicState *s = RMSim::RobotManipulationInterface::calculateSymbolicState(&sim);
  s->lits.append(lits);
  relational::reason::derive(s);
  
  cout<<"State:"<<endl<<*s<<endl<<endl;
  PRINT(relational::reason::getConstants());

  relational::StateTransitionL transL;

  //RMSim::RobotManipulationInterface::generateSimulationSequence_realistic(os, sim, 10, 19);
  std::ofstream os("transition.dat");
  os << *s <<std::endl;
  relational::Symbol* p_GRAB = relational::Symbol::get(mlr::String("grab"));
  relational::Symbol* p_PUTON = relational::Symbol::get(mlr::String("puton"));

  relational::Symbol *ac;
  uintA args;
  args.resize(1);
  for(int iter = 0; iter < mlr::getParameter<int>("actions", 50); ++iter) {

    relational::StateTransition *trans = new relational::StateTransition;
    trans->pre = *s;

    //relational::Literal* action = RMSim::RobotManipulationInterface::generateAction_wellBiased(*s, 19);
    if(iter % 2 == 0) {
      ac = p_GRAB;
      args(0) = rand()%4 + 21;
    }
    else {
      ac = p_PUTON;
      if(rand() % 4 == 1)
        args(0) = 20;
      else
        args(0) = rand()%4 + 21;
    }

    relational::Literal *action = relational::Literal::get(ac, args, 1);
    //perform the action
    os << *action << std::endl;
    DEBUG_VAR(main, *action);
    RMSim::RobotManipulationInterface::performAction(action, &sim, 2);
    trans->action = action;


    // calculate grounded symbols
    relational::calculateSymbols(lits, sgs, sim.C);

    // calculate handcrafted symbols
    s = RMSim::RobotManipulationInterface::calculateSymbolicState(&sim);

    // append grounded symbols
    s->lits.append(lits);
    relational::reason::derive(s);
    trans->post = *s;

    os << *s << std::endl;

    transL.append(trans);
    //DEBUG_VAR(main, transL);

  }
  os.close();

}


