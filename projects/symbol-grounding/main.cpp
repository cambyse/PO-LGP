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
#include <relational/prada.h>

#include <MT/array_t.cxx>
#include <MT/ors.h>

#include <csignal>
SET_LOG(main, DEBUG);

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}

void init_grounded_symbol(AL_GroundedSymbol& gs) {
  bool gaussproc = MT::getParameter<bool>("gauss", true);
  int n_steps = MT::getParameter<int>("steps", 20);

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
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  double seed = MT::getParameter<double>("seed", time(NULL));
	srand(seed);
  
  
  double discountFactor;
  MT::getParameter(discountFactor, "discountFactor");
  PRINT(discountFactor);

  uint horizon;
  MT::getParameter(horizon, "PRADA_horizon");
  PRINT(horizon);
  
  uint PRADA_num_samples;
  MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
  PRINT(PRADA_num_samples);
  
  double PRADA_noise_softener;
  MT::getParameter(PRADA_noise_softener, "PRADA_noise_softener");
  PRINT(PRADA_noise_softener);
  
  MT::String rulesFile_name;
  MT::getParameter(rulesFile_name, "file_rules");
  
  MT::String stateFile_name;
  MT::getParameter(stateFile_name, "file_state");
  PRINT(stateFile_name);
  
  MT::String rewardFile_name;
  MT::getParameter(rewardFile_name, "file_reward");
  PRINT(rewardFile_name);
  
  MT::String symbolsFile_name;
  MT::getParameter(symbolsFile_name, "file_symbols");
  PRINT(symbolsFile_name);

  RobotManipulationSimulator sim;
  sim.shutdownAll();
  sim.loadConfiguration(MT::getParameter<MT::String>("orsFile"));
  sim.startOde();
  sim.startSwift();
  sim.simulate(150);


  // ------------------------------------
  //  SET UP GROUNDED SYMBOL
  //  -----------------------------------

  MT::String relation("inside");
  AL_GroundedSymbol gs(relation, 2, false);
  init_grounded_symbol(gs);
  MT::Array<relational::GroundedSymbol*> sgs;
  sgs.append(&gs);
  relational::LitL lits;
  relational::calculateSymbols(lits, sgs, sim.C);
  DEBUG(main, "test");
  DEBUG_VAR(main, lits); 
  

	// -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
  
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
  
  //cout<<endl<<endl;
  //cout<<"Reading state from file \""<<stateFile_name<<"\"... "<<flush;
  //relational::SymbolicState s;
  //ifstream in_state(stateFile_name);
  //s.read(in_state);
  //cout<<"done!"<<endl<<endl;
  cout<<"State:"<<endl<<*s<<endl<<endl;
  //relational::reason::setConstants(s.state_constants);
  //cout<<"CONSTANTS:"<<endl;  PRINT(relational::reason::getConstants());
  PRINT(relational::reason::getConstants());

  // ------------------------------------
  // REWARD
  // ------------------------------------

  relational::Reward* reward = NULL;
  reward = relational::RobotManipulationSymbols::RewardLibrary::cleanup(&sim);
  //reward = relational::RobotManipulationSymbols::RewardLibrary::stack();
	cout<<"REWARD: "<<endl;
	reward->write();

	
  // -------------------------------------
  // RULES
  // -------------------------------------
  
	cout<<endl<<endl;
	cout<<"RULES:"<<endl;
  relational::RuleSet rules;
  relational::RuleSet::read(rulesFile_name, rules);
  cout<<"Rules successfully read from file \""<<rulesFile_name<<"\"."<<endl;
  cout<<"Rules ("<<rules.num()<<"):"<<endl;   rules.write(cout);

  relational::RuleSet coveringGroundRules;
  relational::reason::calc_coveringRules(coveringGroundRules, rules, *s);
  cout<<endl<<endl<<"Ground actions with unique covering rules:"<<endl;
  uint i;
  FOR1D_(coveringGroundRules, i) {
    cout<<*coveringGroundRules.elem(i)->action<<" ";
  }
  cout<<endl;

  relational::RuleSet ground_rules;
  relational::RuleSet::ground_with_filtering(ground_rules, rules, relational::reason::getConstants(), *s);
  cout<<endl<<endl;
  cout<<"GROUND RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_rules.num()<<endl;

  cout<<endl<<endl;
  cout<<"PLANNER:"<<endl;
  relational::NID_Planner* planner = new relational::PRADA_Planner();
  ((relational::PRADA_Planner* ) planner)->setNumberOfSamples(PRADA_num_samples);
  ((relational::PRADA_Planner* ) planner)->setNoiseSoftener(PRADA_noise_softener);
  planner->setDiscount(discountFactor);
  planner->setHorizon(horizon);
  planner->setGroundRules(ground_rules);
  planner->setReward(reward);

  for(int iter = 0; iter < 10; ++iter) {
    // calculate grounded symbols
    relational::calculateSymbols(lits, sgs, sim.C);

    // calculate handcrafted symbols
    s = RMSim::RobotManipulationInterface::calculateSymbolicState(&sim);

    // append grounded symbols
    s->lits.append(lits);
    relational::reason::derive(s);

    // actual plan the next action
    relational::Literal* action = planner->plan_action(*s);

    //perform the action
    RMSim::RobotManipulationInterface::performAction(action, &sim, 8);
  }

}


