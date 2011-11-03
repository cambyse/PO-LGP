#include <TL/logicDefinitions.h>
#include <TL/ruleReasoning.h>
#include <TL/prada.h>


#define PLAN_TYPE__SST 1
#define PLAN_TYPE__UCT 2
#define PLAN_TYPE__PRADA 3
#define PLAN_TYPE__A_PRADA 4



void logicDemo() {
  cout<<"********************************"<<endl;
  cout<<" RELATIONAL LOGIC DEMO"<<endl;
  cout<<" This demo shows how to define a logic language and rules within the code."<<endl;
  cout<<"********************************"<<endl;
  
  // ----------------------------------
  // SOME CONCEPTS
  //  (Should be pointers. The logicObjectManager will take care for deletion.)
  TL::Predicate* p_incity = new TL::Predicate;
  p_incity->id = 1;
  p_incity->name = "p_incity";
  p_incity->d = 1;
  
  TL::Predicate* p_cruiseTo_action = new TL::Predicate;
  p_cruiseTo_action->id = 2;
  p_cruiseTo_action->name = "cruiseto";
  p_cruiseTo_action->d = 1;
  p_cruiseTo_action->type = TL::Predicate::predicate_action;
  
  TL::Function* f_gas = new TL::Function;
  f_gas->id = 11;
  f_gas->name = "f_gas";
  f_gas->d = 0;  
  
  cout<<"CONCEPTS:"<<endl;
  p_incity->writeNice(cout); cout<<endl;
  p_cruiseTo_action->writeNice(cout); cout<<endl;
  f_gas->writeNice(cout); cout<<endl;
  cout<<endl;
  
  
  // Provide the new predicates to the logicObjectManager
  PredL p_prim, p_derived, p_actions;
  FuncL f_prim, f_derived;
  
  p_prim.append(p_incity);
  p_actions.append(p_cruiseTo_action);
  f_prim.append(f_gas);
  
  TL::logicObjectManager::setPredicatesAndFunctions(p_prim, p_derived, f_prim, f_derived, p_actions);
  
  TL::logicObjectManager::writeLanguage("logicDemo__used_language.dat");
  
  
  // ----------------------------------
  // SOME CONSTANTS
  // (represented by large uints)
  uintA constants;
  constants.append(20); // e.g. city 1 which might be Berlin
  constants.append(21); // e.g. city 2 which might be Frankfurt
  constants.append(22); // e.g. a person which might be Angela Merkel
  
  
  // Provide the constants to the logicObjectManager
  TL::logicObjectManager::setConstants(constants);
    
  
  
  // ----------------------------------
  // LITERALS, FUNCTION-VALUES and ATOMS
  uintA arguments(1);
  
  arguments(0) = 0;   // Convention: small uints (<10) denote constants.
  TL::Literal* l1 = TL::logicObjectManager::getLiteral(p_incity, true, arguments);
  
  arguments(0) = 1;
  TL::Literal* l2 = TL::logicObjectManager::getLiteral(p_incity, false, arguments);
  
  arguments(0) = 21;  // Convention: larger uints (>=10) denote constants.
  TL::Literal* l3 = TL::logicObjectManager::getLiteral(p_incity, true, arguments);
  
  arguments(0) = 0;
  TL::Atom* a_action = TL::logicObjectManager::getAtom(p_cruiseTo_action, arguments);
  
  arguments.clear();  // arguments = {}
  TL::FunctionValue* fv1 = TL::logicObjectManager::getFV(f_gas, arguments, 3);
  TL::FunctionValue* fv2 = TL::logicObjectManager::getFV(f_gas, arguments, 0);
  
  TL::ComparisonLiteral* l_comp =  TL::logicObjectManager::getCompLiteral_constant(f_gas, TL::comparison_greater, 0., arguments);
  
  
  cout<<"LITERALS:"<<endl<<*l1<<endl<<*l2<<endl<<*l3<<endl<<*a_action<<endl<<*fv1<<endl<<*fv2<<endl<<*l_comp<<endl;
  
  
  // ----------------------------------
  // RULES
  
  // RULE 1: noisy default rule
  //  --> Always required!!
  double probability_noise_outcome = 0.7;
  TL::Rule* rule1 = TL::ruleReasoning::generateDefaultRule(probability_noise_outcome);
  rule1->noise_changes = 1.7;
  
  // RULE 2
  TL::Rule* rule2 = new TL::Rule;
  
  rule2->context.append(l1);
  rule2->context.append(l2);
  rule2->context.append(l_comp);
  
  rule2->action = a_action;
  
  // Outcome 1:  action succeeded: different city
  LitL outcome1;
  outcome1.append(TL::logicObjectManager::getLiteralNeg(l1));
  outcome1.append(TL::logicObjectManager::getLiteralNeg(l2));
  rule2->outcomes.append(outcome1);
  
  // Outcome 2:  action failed: still in same city
  //  (We've driving around, but could not find our way out of the city.)
  LitL outcome2;
  rule2->outcomes.append(outcome2);
  
  // Outcome 3:  noise outcome
  //  (E.g. we ended in a different city, or our car broke on the Autobahn or...)
  LitL outcome3; // empty dummy
  rule2->outcomes.append(outcome3);
  rule2->noise_changes = 2.4; // for PRADA's noise outcome heuristic
  
  arr outcome_probabilities(3);
  outcome_probabilities(0) = 0.7;
  outcome_probabilities(1) = 0.2;
  outcome_probabilities(2) = 0.1;
  rule2->probs = outcome_probabilities;
  
  
  cout<<"RULES:"<<endl<<*rule1<<endl<<*rule2<<endl;
  
  
  // We don't delete the rule pointers in this demo.
  // This is usually achieved by managing rules in 
  // the data structure RuleSet.
}








/*
  
*/
void planDemo() {
  cout<<"********************************"<<endl;
  cout<<" PLANNING DEMO"<<endl;
  cout<<" Domain: Robot Manipulation"<<endl;
  cout<<" This demo shows how to set up a planning problem and solve it."<<endl
      <<" It uses the robot manipulation domain of the experiments in"<<endl
      <<" the paper Lang & Toussaint, JAIR (2010)."<<endl;
  cout<<"********************************"<<endl;
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
    
   // Set up logic
  cout<<"Reading language file 'language.dat'."<<endl;
  TL::logicObjectManager::setPredicatesAndFunctions("language.dat");


  // -------------------------------------
  //   STATE
  // -------------------------------------
  
  cout<<"Reading state from 'state.dat'... "<<flush;
  TL::State* s;
  // Here, we use the readTrial-method to read
  // a single state (= trial with only one state).
  // Second argument "true": constants in first line of "state.dat"
  // will be added to the list of the logicObjectManager.
  TL::Trial* trial = TL::logicObjectManager::readTrial_withConstants("state.dat", true);
  s = trial->states(0);
  cout<<"done!"<<endl;
  cout<<"STARTING STATE:"<<endl<<*s<<endl<<endl;
  PRINT(TL::logicObjectManager::constants);
  
  
  
  // -------------------------------------
  // REWARD
  // -------------------------------------
  
  TL::Reward* reward = NULL;
  TL::Predicate* p_ON = TL::logicObjectManager::getPredicate(MT::String("on"));
  uintA args(2);  args(0)=66;  args(1)=70;
  TL::Literal* lit = TL::logicObjectManager::getLiteral(p_ON, true, args);
  reward = new TL::LiteralReward(lit);
  cout<<"REWARD: "<<endl;  reward->writeNice();  cout<<endl<<endl;
    
  
  // -------------------------------------
  // RULES
  // -------------------------------------
  
  TL::RuleSet rules;
  TL::logicObjectManager::readRules("rules.dat", rules);
  cout<<"Rules successfully read from 'rules.dat'."<<endl;
  cout<<"RULES ("<<rules.num()<<"):"<<endl;   rules.write(cout);
  
#if 1
  // Manually create "doNothing"-rule if desired
  TL::Rule* rule_doNothing = TL::ruleReasoning::getDoNothingRule();
  rules.append(rule_doNothing);
  rule_doNothing->write();
#endif

  
  
  // -------------------------------------
  // GROUND THE RULES
  // -------------------------------------
  
  TL::RuleSet ground_rules;
  TL::ruleReasoning::ground_with_filtering(ground_rules, rules, TL::logicObjectManager::constants, *s);
  cout<<"GROUND RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_rules.num()<<endl;
//   ground_rules.write();
  
  
  
  
  // -------------------------------------
  // PLANNERs
  // -------------------------------------
  
//   PLAN_TYPE__SST,   PLAN_TYPE__UCT,  PLAN_TYPE__PRADA,  PLAN_TYPE__A_PRADA
  cout<<"*** Set choice of planner (SST, UCT, PRADA, A-PRADA) in the code: "<<MT_HERE<<endl;
  uint plan_type = PLAN_TYPE__PRADA;
  
  // General parameters
  uint horizon = 2;
  double discountFactor = 0.95;
  
  // SST parameters
  double SST_noise_scaling_factor = 0.75;
  uint SST_branch = 2;
  
  // PRADA parameters
  uint PRADA_num_samples = 300;
  double PRADA_noise_softener = 0.2;
  
  // UCT parameters  
  uint UCT_c = 1.0;
  uint UCT_numEpisodes = 500;
  
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
  cout<<"Planner has been fully set up."<<endl;cout<<endl;
  
  
  
 
  
  
  // -------------------------------------
  //    PLANNING
  // -------------------------------------
  
  TL::Atom* action;
  cout<<endl<<endl<<"*** Planning for a single action."<<endl;
  action = planner->generateAction(*s);
  
  cout<<"The planner would like to kindly recommend the following action to you:"<<endl<<*action<<endl;
  
  if (plan_type == PLAN_TYPE__PRADA || plan_type == PLAN_TYPE__A_PRADA) {
    cout<<endl<<endl<<"*** Planning for a complete plan."<<endl;
    AtomL plan;
    double value;
    ((TL::PRADA*) planner)->generatePlan(plan, value, *s);
    cout<<endl;
    cout<<"Also, it would like to kindly recommend the following plan with value "<<value<<" to you:"<<endl<<plan<<endl;
  }
  
  cout<<endl<<endl<<"ALL TESTS SUCCESSFUL!\nAte mais!"<<endl;
}



int main(int argc, char** argv) {
  logicDemo();
  planDemo();
}
