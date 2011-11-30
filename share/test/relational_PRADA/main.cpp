#include <relational/prada.h>


#define PLAN_TYPE__SST 1
#define PLAN_TYPE__UCT 2
#define PLAN_TYPE__PRADA 3
#define PLAN_TYPE__A_PRADA 4



void plan() {
  // -------------------------------------
  // READ CONFIG
  // -------------------------------------
  uint randSeed;
  MT::getParameter(randSeed, "randSeed");
  rnd.seed(randSeed);
  PRINT(randSeed);
  
  uint plan_type;
  MT::getParameter(plan_type, "plan_type");
  PRINT(plan_type);
  
  double discountFactor;
  MT::getParameter(discountFactor, "discountFactor");
  PRINT(discountFactor);
  
  
  double SST_noise_scaling_factor;
  MT::getParameter(SST_noise_scaling_factor, "noise_scaling_factor");
  PRINT(SST_noise_scaling_factor);
  
  uint SST_branch;
  MT::getParameter(SST_branch, "SST_branch");
  PRINT(SST_branch);
  
  uint SST_horizon;
  MT::getParameter(SST_horizon, "SST_horizon");
  PRINT(SST_horizon);
        
  
  uint PRADA_horizon;
  MT::getParameter(PRADA_horizon, "PRADA_horizon");
  PRINT(PRADA_horizon);
  
  uint PRADA_num_samples;
  MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
  PRINT(PRADA_num_samples);
  
  double PRADA_noise_softener;
  MT::getParameter(PRADA_noise_softener, "PRADA_noise_softener");
  PRINT(PRADA_noise_softener);
  
  
    
  uint UCT_horizon;
  MT::getParameter(UCT_horizon, "UCT_horizon");
  PRINT(UCT_horizon);
  
  uint UCT_c;
  MT::getParameter(UCT_c, "UCT_c");
  PRINT(UCT_c);
  
  uint UCT_numEpisodes;
  MT::getParameter(UCT_numEpisodes, "UCT_numEpisodes");
  PRINT(UCT_numEpisodes);
  
  uint horizon;
  if (plan_type == PLAN_TYPE__SST) horizon = SST_horizon;
  else if (plan_type == PLAN_TYPE__UCT) horizon = UCT_horizon;
  else horizon = PRADA_horizon;
  PRINT(horizon);
  
    
  MT::String rulesFile_name;
  MT::getParameter(rulesFile_name, "file_rules");
  PRINT(rulesFile_name);
  
  MT::String stateFile_name;
  MT::getParameter(stateFile_name, "file_state");
  PRINT(stateFile_name);
  
  MT::String rewardFile_name;
  MT::getParameter(rewardFile_name, "file_reward");
  PRINT(rewardFile_name);
  
  MT::String languageFile_name;
  MT::getParameter(languageFile_name, "file_language");
  PRINT(languageFile_name);

  
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
    
  TL::logicObjectManager::setPredicatesAndFunctions(languageFile_name);

  
  // -------------------------------------
  //   STATE
  // -------------------------------------
  
  cout<<"Reading state... "<<flush;
  TL::State* s;
  // Here, we use the readTrial-method to read
  // a single state (= trial with only one state).
  // Second argument "true": constants in state description
  // will be added to the list of the logicObjectManager.
  TL::Trial* trial = TL::logicObjectManager::readTrial_withConstants(stateFile_name, true);
  s = trial->states(0);
  cout<<"done!"<<endl;
  cout<<"STARTING STATE:"<<endl<<*s<<endl<<endl;
  PRINT(TL::logicObjectManager::constants);
    
  
  
  // -------------------------------------
  // REWARD
  // -------------------------------------
  
  cout<<"Reading reward... "<<flush;
  TL::Reward* reward = TL::readReward(rewardFile_name);
  cout<<"done!"<<endl;
  cout<<"REWARD: ";  reward->writeNice(); cout<<endl;cout<<endl;
  
  
  
  // -------------------------------------
  // RULES
  // -------------------------------------
  
  TL::RuleSet rules;
  TL::logicObjectManager::readRules("rules.dat", rules);
  cout<<"Rules successfully read."<<endl;
  cout<<"RULES ("<<rules.num()<<"):"<<endl;   rules.write(cout);
  
 
  
  // -------------------------------------
  // GROUND THE RULES
  // -------------------------------------
  
  TL::RuleSet ground_rules;
  TL::ruleReasoning::ground_with_filtering(ground_rules, rules, TL::logicObjectManager::constants, *s);
  cout<<"GROUND RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_rules.num()<<endl;
//   ground_rules.write();
  
  
  
  
  
  // -------------------------------------
  // PLANNER
  // -------------------------------------
  
  TL::NID_Planner* planner = NULL;
  if (plan_type == PLAN_TYPE__SST) {
    planner = new TL::NID_SST(SST_branch, SST_noise_scaling_factor);
    cout<<"Planner: SST"<<endl;
  }
  else if (plan_type == PLAN_TYPE__PRADA) {
    planner = new TL::PRADA();
    ((TL::PRADA* ) planner)->setNumberOfSamples(PRADA_num_samples);
    ((TL::PRADA* ) planner)->setNoiseSoftener(PRADA_noise_softener);
    cout<<"Planner: PRADA"<<endl;
  }
  else if (plan_type == PLAN_TYPE__A_PRADA) {
    planner = new TL::A_PRADA();
    ((TL::A_PRADA* ) planner)->setNumberOfSamples(PRADA_num_samples);
    ((TL::A_PRADA* ) planner)->setNoiseSoftener(PRADA_noise_softener);
    cout<<"Planner: A-PRADA"<<endl;
  }
  else if (plan_type == PLAN_TYPE__UCT) {
    planner = new TL::NID_UCT(SST_noise_scaling_factor);
    ((TL::NID_UCT*) planner)->setNumEpisodes(UCT_numEpisodes);
    ((TL::NID_UCT*) planner)->setC(UCT_c);
    cout<<"Planner: UCT"<<endl;
  }
  else {
    HALT("no planner defined");
  }
  planner->setDiscount(discountFactor);
  planner->setHorizon(horizon);
  planner->setGroundRules(ground_rules);
  planner->setReward(reward);
  cout<<"Planner has been fully set up."<<endl; cout<<endl;

  // -------------------------------------
  //    PLANNING
  // -------------------------------------
  
  TL::Atom* action;
  action = planner->generateAction(*s);
  
  cout<<"The planner would like to kindly recommend the following action to you:"<<endl<<*action<<endl;
  
  if (plan_type == PLAN_TYPE__PRADA || plan_type == PLAN_TYPE__A_PRADA) {
    AtomL plan;
    double value;
    ((TL::PRADA*) planner)->generatePlan(plan, value, *s);
    cout<<endl;
    cout<<"Also, it would like to kindly recommend the following plan with value "<<value<<" to you:"<<endl<<plan<<endl;
  }
  
  cout<<endl<<endl<<"ALL TESTS SUCCESSFUL!"<<endl<<"Adeus!"<<endl;
}




int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  cout.precision(3);
  MT::String config_file;
  MT::getParameter(config_file, MT::String("confFile"), MT::String("config"));
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
  plan();
  return 0;
}

