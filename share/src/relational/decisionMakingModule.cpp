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
#include "relational/robotManipulationSymbols.h"
#include "relational/robotManipulationInterface.h"
#include "relational/reason.h"
#include "relational/prada.h"

#include "decisionMakingModule.h"

#define RULES_FILE "rules.dat"

RobotManipulationSimulator sim;
relational::RuleSet rules;
relational::RuleSet ground_rules;
relational::PRADA_Planner* prada;
relational::Reward* reward;

uint PRADA_horizon = 4;
uint PRADA_num_samples = 500;
double PRADA_noise_softener = 0.1;
double PRADA_threshold_goal_achieved = 0.1;
double discountFactor = 0.95;

using namespace relational;
  
  
DecisionMakingModule::DecisionMakingModule():Process("DecisionMaking") {
  prada = NULL;
  reward = NULL;
}
 
//initialize,load knowledge, etc
void DecisionMakingModule::open() {
  uint DEBUG = 2;
 
  // Connect ActionInterface
  CHECK(ors,"please set the ors pointer before launching DecisionMakingModule!");
  sim.C = ors;
  sim.calcObjectNumber();
  
  // Setup logic
  relational::RobotManipulationSymbols::initSymbols();
  
  // Create ruleReasoning and rules
  relational::RuleSet::read(RULES_FILE, rules);
  relational::Rule* rule_doNothing = relational::Rule::getDoNothingRule();
  rules.append(rule_doNothing);
  if (DEBUG>1) {rules.write();}
  
  // Set up PRADA
  prada = new relational::PRADA_Planner();
  prada->setNumberOfSamples(PRADA_num_samples);
  prada->setNoiseSoftener(PRADA_noise_softener);
  prada->setThresholdReward(PRADA_threshold_goal_achieved);
  prada->setDiscount(discountFactor);
  prada->setHorizon(PRADA_horizon);
  if (DEBUG>0) {cout<<"DecisionMakingModule has been successfully openend."<<endl;}

  actionIsReady=false;
}

void DecisionMakingModule::close() {
//   sim.shutdownAll();
  delete prada;
  delete reward;
}


// Planning
void DecisionMakingModule::step() {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"DecisionMakingModule::step() [START]"<<endl;}
  
  // Some administration (read out objects) if called for the first time
  if (relational::reason::getConstants().N == 0) {
    uintA objects;
    sim.getObjects(objects);
    if (DEBUG>0) {PRINT(objects);}
    
//     TermTypeA objects_types;
//     sim.getTypes(objects_types, objects, le->types);
//     le->setConstants(objects, objects_types);
    relational::reason::setConstants(objects);
    
    // set up reward
    uintA blocks;
    sim.getBlocks(blocks);
    CHECK(blocks.N > 1, "");
    reward = relational::RobotManipulationSymbols::RewardLibrary::on(blocks(0), blocks(1));
    if (DEBUG>0) {cout<<"Reward:  "; reward->write(); cout<<endl;}
    
    // ground rules
    relational::SymbolicState* s_groundingHelper = RMSim::RobotManipulationInterface::calculateSymbolicState(&sim);
    if (DEBUG>0) {cout<<"s_groundingHelper:"<<endl;  s_groundingHelper->write();  cout<<endl;}
    relational::RuleSet::ground_with_filtering(ground_rules, rules, relational::reason::getConstants(), *s_groundingHelper);
    delete s_groundingHelper;
    ground_rules.sort_using_args();
    ground_rules.sanityCheck();
    if (DEBUG>0) {cout<<"Rules have been grounded to "<<ground_rules.num()<<" ground rules."<<endl;  /*ground_rules.writeNice();*/}
    
    // finish set up PRADA
    prada->setGroundRules(ground_rules);
    prada->setReward(reward);
  }
  
  
  // Read state
  relational::SymbolicState* s = RMSim::RobotManipulationInterface::calculateSymbolicState(&sim);
  if (DEBUG>0) {cout<<"STATE:"<<*s<<endl;}
  
  // Check if finished
  if (reward->satisfied(*s)) {
    action = SYMBOLIC_ACTION__FINISHED;
    return;
  }
  
  // Plan
  relational::Literal* lit_action = prada->plan_action(*s, 1);
  if (DEBUG>0) {if (lit_action == NULL) cout<<"NO ACTION FOUND"<<endl; else cout<<"ACTION:   "<<*lit_action<<endl;}
  
  // Set fields
  if (lit_action == NULL) {
    action = SYMBOLIC_ACTION__NO_ACTION_FOUND;
  }
  else if (lit_action->s->name == "grab") {
    action = SYMBOLIC_ACTION__GRAB;
    actionArgument = lit_action->args(0);
  }
  else if (lit_action->s->name == "puton") {
    action = SYMBOLIC_ACTION__PUTON;
    actionArgument = lit_action->args(0);
  }
  else {
    action = 3; // unspecified
  }
  if (DEBUG>0) {if (lit_action != NULL) {PRINT(action);  PRINT(actionArgument)}}
  if (DEBUG>0) {cout<<"DecisionMakingModule::step() [END]"<<endl;}

  actionIsReady=true;
}
