#include <Core/util.tpp>
#include <relational/prada.h>
#include <relational/robotManipulationSymbols.h>

#define PLAN_TYPE__PRADA 1
#define PLAN_TYPE__A_PRADA 2
#define PLAN_TYPE__UCT 3
#define PLAN_TYPE__SST 4



void concurrent_plan() {
  cout<<"********************************"<<endl;
  cout<<" libPRADA plan demo"<<endl;
  cout<<" Domain: Robot Manipulation"<<endl;
  cout<<" This demo shows how to set up a planning problem and solve it."<<endl
      <<" It uses the robot manipulation domain of the experiments in"<<endl
      <<" the paper Lang & Toussaint, JAIR (2010)."<<endl;
  cout<<"********************************"<<endl;
  
	
  // -------------------------------------
  // READ CONFIG
  // -------------------------------------
	
  cout<<endl<<"READ CONFIG:"<<endl;
	
  uint randSeed;
  mlr::getParameter(randSeed, "randSeed");
  rnd.seed(randSeed);
  PRINT(randSeed);
  
  uint plan_type;
  mlr::getParameter(plan_type, "plan_type");
  PRINT(plan_type);
  
  double discountFactor;
  mlr::getParameter(discountFactor, "discountFactor");
  PRINT(discountFactor);
  
  
  double SST_noise_scaling_factor;
  mlr::getParameter(SST_noise_scaling_factor, "noise_scaling_factor");
  PRINT(SST_noise_scaling_factor);
  
  
  uint UCT_horizon;
  mlr::getParameter(UCT_horizon, "UCT_horizon");
  PRINT(UCT_horizon);
  
  uint UCT_c;
  mlr::getParameter(UCT_c, "UCT_c");
  PRINT(UCT_c);
  
  uint UCT_numEpisodes;
  mlr::getParameter(UCT_numEpisodes, "UCT_numEpisodes");
  PRINT(UCT_numEpisodes);
  
  uint horizon;
  if (plan_type == PLAN_TYPE__UCT) horizon = UCT_horizon;
  else horizon = UCT_horizon;
  PRINT(horizon);
  
    
  mlr::String rulesFile_name;
  mlr::getParameter(rulesFile_name, "file_rules");
  PRINT(rulesFile_name);
  
  mlr::String stateFile_name;
  mlr::getParameter(stateFile_name, "file_state");
  PRINT(stateFile_name);

  mlr::String terminalFile_name;
  mlr::getParameter(terminalFile_name, "file_terminal");
  PRINT(terminalFile_name);
  
  mlr::String rewardFile_name;
  mlr::getParameter(rewardFile_name, "file_reward");
  PRINT(rewardFile_name);
  
  mlr::String symbolsFile_name;
  mlr::getParameter(symbolsFile_name, "file_symbols");
  PRINT(symbolsFile_name);
	

  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
  
  cout<<endl<<endl;
  cout<<"SYMBOLS:"<<endl;
  cout<<"Reading symbols from file \""<<symbolsFile_name<<"\"..."<<flush;
  relational::SymL symbols;
  relational::ArgumentTypeL types;
  relational::readSymbolsAndTypes(symbols, types, symbolsFile_name);
  cout<<"done!"<<endl;
  
  relational::writeSymbolsAndTypes("used_symbols.dat");

  cout<<"symbols " <<symbols<<endl;


  // -------------------------------------
  //   STATE
  // -------------------------------------
  
  cout<<endl<<endl;
  cout<<"STARTING STATE:"<<endl;
  cout<<"Reading state from file \""<<stateFile_name<<"\"... "<<flush;
  relational::SymbolicState s;
  ifstream in_state(stateFile_name);
  s.read(in_state);
  ////////////////////////////////////////////////////////////////////
  // augment with functions, e.g. pickup(X Y)=0 (where hand(X), object(Y)
  // objects ID: 67->75 (note: ground(71))
  // hands   ID: 76->78
  ///////////////////////////////////////////////////////////////////
  uint i,j;
/*/  for(i=76;i<79;i++){
      //pickup
      for(j=67;j<76;j++){
        relational::Literal* lit_pick = relational::Literal::get(relational::Symbol::get("pickup"), TUP(i, j), 0.);
        s.lits.append(lit_pick);
      }
      //positioning
      relational::Literal* lit_positioning = relational::Literal::get(relational::Symbol::get("positioning"), TUP(i), 0.);
      s.lits.append(lit_positioning);
      //release
      relational::Literal* lit_release = relational::Literal::get(relational::Symbol::get("release"), TUP(i), 0.);
      s.lits.append(lit_release);
      //screwing
      for(j=67;j<71;j++){
        relational::Literal* lit_screwing = relational::Literal::get(relational::Symbol::get("screwing"), TUP(i), 0.);
        s.lits.append(lit_screwing);
      }
  }
/*/
  for(i=76;i<79;i++){
      //pickup
      for(j=67;j<68;j++){
        relational::Literal* lit_pick = relational::Literal::get(relational::Symbol::get("pickup"), TUP(i, j), 0.);
        s.lits.append(lit_pick);
      }
      for(j=71;j<=72;j++){
        relational::Literal* lit_pick = relational::Literal::get(relational::Symbol::get("pickup"), TUP(i, j), 0.);
        s.lits.append(lit_pick);
      }
      //positioning
      relational::Literal* lit_positioning = relational::Literal::get(relational::Symbol::get("positioning"), TUP(i), 0.);
      s.lits.append(lit_positioning);
      //release
      relational::Literal* lit_release = relational::Literal::get(relational::Symbol::get("release"), TUP(i), 0.);
      s.lits.append(lit_release);
      //screwing
      for(j=67;j<68;j++){
        relational::Literal* lit_screwing = relational::Literal::get(relational::Symbol::get("screwing"), TUP(i), 0.);
        s.lits.append(lit_screwing);
      }
  }
  //////////////////////////////////////////////////////
  cout<<"done!"<<endl<<endl;
  cout<<"State:"<<endl<<s<<endl<<endl;
  relational::reason::setConstants(s.state_constants);
  cout<<"CONSTANTS:"<<endl;  PRINT(relational::reason::getConstants());
  


  cout<<"TERMINAL STATE:"<<endl;
  cout<<"Reading terminal state from file \""<<terminalFile_name<<"\"... "<<flush;
  relational::SymbolicState sTerminal;
  ifstream in_terminalstate(terminalFile_name);
  sTerminal.read(in_terminalstate);
  cout<<"done!"<<endl<<endl;
  cout<<"TERMINAL State:"<<endl<<sTerminal<<endl<<endl;
  
  
  // -------------------------------------
  // REWARD
  // -------------------------------------
  
  cout<<endl<<endl;
  cout<<"REWARD: "<<endl;
  relational::Reward* reward = NULL;
  // (1) SIMPLE REWARD FOR TOWER
  //     Create here by hand
  // (1a) LiteralReward
//   relational::Literal* lit = relational::Literal::get(relational::Symbol::get("on"), TUP(66, 67), 1.);
//   reward = new relational::LiteralReward(lit);
  // (1b) LiteralListReward
  //relational::LitL lits_reward;
  //relational::Literal::get(lits_reward, mlr::String("on(66 69) on(69 67)"));
  reward = new relational::LiteralListReward(sTerminal.lits);//lits_reward);

  // (2) STACKING REWARD 
  //     Use specification in robotManipulationInterface
//   reward = relational::RobotManipulationSymbols::RewardLibrary::stack();

  reward->write();
	
	
  // -------------------------------------
  // RULES
  // -------------------------------------
  
  cout<<endl<<endl;
  cout<<"RULES:"<<endl;
  relational::RuleSet rules;
  relational::RuleSet::read(rulesFile_name, rules);
  cout<<"Rules successfully read from file \""<<rulesFile_name<<"\"."<<endl;
  //cout<<"Rules ("<<rules.num()<<"):"<<endl;   rules.write(cout);
  


#if 1
  // Devide into two set: activate and terminate
  relational::RuleSet activate_rules;
  relational::RuleSet terminate_rules;
  terminate_rules.clear();

  //add default rule
  activate_rules.append(rules.elem(0));
  terminate_rules.append(rules.elem(0));

  FOR1D_(rules,i){
      if(i>0){//ignore default rule
          if(i%2==1)
            activate_rules.append(rules.elem(i));
          else
            terminate_rules.append(rules.elem(i));
      }

  }
  terminate_rules.sort();
  terminate_rules.sanityCheck();
  activate_rules.sort();
  activate_rules.sanityCheck();
  cout<<"ACTIVATE Rules ("<<activate_rules.num()<<"):"<<endl;     activate_rules.write(cout);
  cout<<"TERMINATE Rules ("<<terminate_rules.num()<<"):"<<endl;   terminate_rules.write(cout);
#endif

#if 1
  // Manually create "doNothing"-rule if desired
  relational::Rule* rule_doNothing = relational::Rule::getDoNothingRule();
  activate_rules.append(rule_doNothing);
  terminate_rules.append(rule_doNothing);
  rule_doNothing->write();
#endif
  
  // -------------------------------------
  // GROUND THE RULES
  // -------------------------------------
  
  relational::RuleSet ground_rules;
  relational::RuleSet ground_terminate_rules;
  relational::RuleSet ground_activate_rules;
  //relational::RuleSet::ground_with_filtering(ground_rules, rules, relational::reason::getConstants(), s);
  relational::RuleSet::ground_with_filtering(ground_activate_rules, activate_rules, relational::reason::getConstants(), s);
  relational::RuleSet::ground_with_filtering(ground_terminate_rules, terminate_rules, relational::reason::getConstants(), s);

  //CONCATENATE two sets

  ground_rules.clear();
  cout<<endl<<endl;
  cout<<"GROUND ACTIVATE RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_activate_rules.num()<<endl;
  ground_rules.append(ground_activate_rules.elem(1)); //default
  ground_rules.append(ground_activate_rules.elem(0)); //donothing

  FOR1D_(ground_activate_rules,i){
     if(i>1) ground_rules.append(ground_activate_rules.elem(i));
     cout<<*ground_activate_rules.elem(i)->action<<"  ";
  }

  cout<<endl<<endl;
  cout<<"GROUND TERMINATE RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_terminate_rules.num()<<endl;
  FOR1D_(ground_terminate_rules,i){
      if(i>1) ground_rules.append(ground_terminate_rules.elem(i)); //ignore default and dothing (as already added before)
      cout<<*ground_terminate_rules.elem(i)->action<<"  ";
  }

  cout<<endl<<endl;
  cout<<"GROUND RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_rules.num()<<endl;
  FOR1D_(ground_rules,i){
      cout<<*ground_rules.elem(i)->action<<"  ";
  }



  // -------------------------------------
  // PLANNERs
  // -------------------------------------

  cout<<endl<<endl;
  cout<<"PLANNER:"<<endl;
  relational::NID_Planner* planner = NULL;
 if (plan_type == PLAN_TYPE__UCT) {
    planner = new relational::NID_UCT_CON();
    ((relational::NID_UCT_CON*) planner)->setNumEpisodes(UCT_numEpisodes);
    ((relational::NID_UCT_CON*) planner)->setC(UCT_c);
    planner->setNoiseScalingFactor(SST_noise_scaling_factor);
    cout<<"Planner: UCT-CONCURRENT"<<endl;
  }
  else {
    HALT("no planner defined");
  }
  planner->setDiscount(discountFactor);
  planner->setHorizon(horizon);
  planner->setGroundRules(ground_rules);
  planner->setReward(reward);
  cout<<"Planner has been fully set up."<<endl;cout<<endl;
  PRINT(planner->ground_actions);
  
     
    // -------------------------------------
  //    PLANNING
  // -------------------------------------
  
  cout<<endl<<endl;
  cout<<"PLANNING:"<<endl;
  relational::Literal* action;
  cout<<endl<<endl<<"*** Planning for a single action."<<endl;
  action = planner->plan_action(s);
  
  if (action != NULL)
    cout<<"The planner would like to kindly recommend the following action to you:"<<endl<<*action<<endl;
  else
    cout<<"No action has been found."<<endl;

}






int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  cout.precision(3);
  mlr::String config_file;
  mlr::getParameter(config_file, mlr::String("confFile"), mlr::String("config"));
  cout << "Config-file: " << config_file << endl;
  mlr::openConfigFile(config_file);
  concurrent_plan();
  return 0;
}

