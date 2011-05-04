/*  
    Copyright 2010   Marc Toussaint, Tobias Lang
    
    Homepage:  cs.tu-berlin.de/~lang/
    E-mail:    lang@cs.tu-berlin.de
    
    This file is part of libPRADA.

    libPRADA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libPRADA is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <MT/ors.h>
#include <TL/utilTL.h>
#include <TL/bwLanguage.h>
#include <TL/ors_actionInterface.h>
#include <TL/logic_world_interface.h>
#include <TL/ruleEngine.h>
#include <TL/prada.h>
#include <TL/plan.h>

#include "decisionMakingModule.h"

#define RULES_FILE "rules.dat"

ActionInterface AI;
TL::LogicEngine* le;
TL::RuleSet rules;
TL::RuleSet ground_rules;
TL::PRADA* prada;
TL::Reward* reward;

uint PRADA_horizon = 4;
uint PRADA_num_samples = 500;
double PRADA_noise_softener = 0.1;
double PRADA_threshold_goal_achieved = 0.1;
double discountFactor = 0.95;
  
  
DecisionMakingModule::DecisionMakingModule():Process("DecisionMaking") {
  le = NULL;
  prada = NULL;
  reward = NULL;
}
 
//initialize,load knowledge, etc
void DecisionMakingModule::open() {
  uint DEBUG = 2;
  // Create LogicEngine
  uintA empty;
  le = TL::bwLanguage::createLogicEngine(empty);
  le->writeLanguage("test");

  // Create RuleEngine and rules
  TL::RuleEngine::setLogicEngine(le);
  TL::RuleEngine::readRulesPlain(RULES_FILE, rules);
  TL::Rule* rule_doNothing = TL::RuleEngine::getDoNothingRule();
  rules.append(rule_doNothing);
  if (DEBUG>1) {rules.writeNice();}
  
  // Connect ActionInterface
  CHECK(ors,"please set the ors pointer before launching DecisionMakingModule!");
  AI.C = ors;
  AI.calcObjectNumber();
  
  // Set up PRADA
  prada = new TL::PRADA(le);
  prada->setNumberOfSamples(PRADA_num_samples);
  prada->setNoiseSoftener(PRADA_noise_softener);
  prada->setThresholdReward(PRADA_threshold_goal_achieved);
  prada->setDiscount(discountFactor);
  prada->setHorizon(PRADA_horizon);
  if (DEBUG>0) {cout<<"DecisionMakingModule has been successfully openend."<<endl;}

  actionIsReady=false;
}

void DecisionMakingModule::close() {
//   AI.shutdownAll();
  delete prada;
  delete reward;
  delete le;
}


// Planning
void DecisionMakingModule::step() {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"DecisionMakingModule::step() [START]"<<endl;}
  
  // Some administration (read out objects) if called for the first time
  if (le->constants.N == 0) {
    uintA objects;
    AI.getObjects(objects);
    if (DEBUG>0) {PRINT(objects);}
    
//     TermTypeA objects_types;
//     AI.getTypes(objects_types, objects, le->types);
//     le->setConstants(objects, objects_types);
    le->setConstants(objects);
    
    // set up reward
    uintA blocks;
    AI.getBlocks(blocks);
    CHECK(blocks.N > 1, "");
    reward = TL::bwLanguage::RewardLibrary::on(blocks(0), blocks(1), le);
    if (DEBUG>0) {cout<<"Reward:  "; reward->writeNice(); cout<<endl;}
    
    // ground rules
    TL::State* s_groundingHelper = TL::logic_world_interface::bw::observeLogic(&AI, le);
    if (DEBUG>0) {cout<<"s_groundingHelper:"<<endl;  s_groundingHelper->writeNice();  cout<<endl;}
    TL::RuleEngine::ground_with_filtering(ground_rules, rules, le->constants, *s_groundingHelper);
    delete s_groundingHelper;
    ground_rules.sort_using_args();
    TL::RuleEngine::removeDoublePredicateInstances(ground_rules);
    TL::RuleEngine::checkRules(ground_rules);
    if (DEBUG>0) {cout<<"Rules have been grounded to "<<ground_rules.num()<<" ground rules."<<endl;  /*ground_rules.writeNice();*/}
    
    // finish set up PRADA
    prada->setGroundRules(ground_rules);
    prada->setReward(reward);
  }
  
  
  // Read state
  TL::State* s = TL::logic_world_interface::bw::observeLogic(&AI, le);
  if (DEBUG>0) {cout<<"STATE:"<<endl;  s->writeNice();  cout<<endl;}
  
  // Check if finished
  if (reward->satisfied(*s)) {
    action = SYMBOLIC_ACTION__FINISHED;
    return;
  }
  
  // Plan
  TL::PredicateInstance* pi_action = prada->generateAction(*s, 1);
  if (DEBUG>0) {if (pi_action == NULL) cout<<"NO ACTION FOUND"<<endl; else cout<<"ACTION:   "<<*pi_action<<endl;}
  
  // Set fields
  if (pi_action == NULL) {
    action = SYMBOLIC_ACTION__NO_ACTION_FOUND;
  }
  else if (pi_action->pred->id == HAND_ID__GRAB) {
    action = SYMBOLIC_ACTION__GRAB;
    actionArgument = pi_action->args(0);
  }
  else if (pi_action->pred->id == HAND_ID__PUTON) {
    action = SYMBOLIC_ACTION__PUTON;
    actionArgument = pi_action->args(0);
  }
  else {
    action = 3; // unspecified
  }
  if (DEBUG>0) {if (pi_action != NULL) {PRINT(action);  PRINT(actionArgument)}}
  if (DEBUG>0) {cout<<"DecisionMakingModule::step() [END]"<<endl;}

  actionIsReady=true;
}
