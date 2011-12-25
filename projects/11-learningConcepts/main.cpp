#define MT_IMPLEMENT_TEMPLATES
#include <relational/robotManipulationDomain.h>
#include <relational/logicReasoning.h>
#include <relational/ruleLearner.h>
#include <relational/symbolGrounding.h>
#include <relational/robotManipulationDomain.h>
#include <relational/prada.h>



// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------



void learn_rules(TL::RuleSet& rules, ExperienceL& symbolic_experiences) {
  double alpha_PEN = 0.01;
  double p_min = 10e-8;
  double p_min_noisyDefaultRule = p_min;
  TL::RuleLearner learner(alpha_PEN, p_min, p_min_noisyDefaultRule);
  TL::RuleSetContainer rulesC;
  learner.learn_rules(rulesC, symbolic_experiences);
  cout<<"Learned rule-set:"<<endl;  rulesC.writeNice();
  rules = rulesC.rules;
}


void produce_rules(const char* file_rules, const char* file_data, const char* file_parameters, relational::SymbolGrounding::GroundingType grounding_type) {
  uint i;
  // (1) Read data
  MT::Array< relational::FullExperience* > experiences;
  relational::FullExperience::read_nikolayFormat(experiences, file_data);
  cout<<experiences.N<<" experiences have been read."<<endl;

  // (2) Read parameters
  MT::Array<relational::SymbolGrounding*> sgs;
  relational::read(sgs, file_parameters, grounding_type);
  cout<<sgs.N<<" SymbolGroundings have been read."<<endl;
  
  // (3) Calculate symbols
  FOR1D(experiences, i) {
    relational::calculateLiterals(sgs, *experiences(i));
  }
  cout<<"Symbols have been calculated in experiences."<<endl;
  
  cout<<"Experiences ["<<experiences.N<<"]:"<<endl;
//   relational::FullExperience::write_symbolic(experiences, cout);
  
  // (4) learn rules
  ExperienceL symbolic_experiences;
  FOR1D(experiences, i) {
//     if (i>30) break;
    symbolic_experiences.append(&experiences(i)->experience_symbolic);
  }
  TL::RuleSet rules;
  learn_rules(rules, symbolic_experiences);
  write(rules, file_rules);
  cout<<rules.num()<<" rules have been learned."<<endl;
  
  
  FOR1D(experiences, i) {delete experiences(i);}
  FOR1D(sgs, i) {delete sgs(i);}
}




void produce_rules() {
  uint grounding_type__uint;
  MT::getParameter(grounding_type__uint, "grounding_type");
  relational::SymbolGrounding::GroundingType grounding_type;
  if (grounding_type__uint == 0)
    grounding_type = relational::SymbolGrounding::NN;
  else
    grounding_type = relational::SymbolGrounding::RBF;
  PRINT_(grounding_type);
  
  MT::String dir;
  if (grounding_type == relational::SymbolGrounding::NN)
    dir = "parameters_NN/";
  else if (grounding_type == relational::SymbolGrounding::RBF)
    dir = "parameters_RBF/";
  
  
  // Set the objects
  uintA objects;
  uint i;
  uint OBJ_NUMBER = 5;  // set by hand
  for (i=0; i<OBJ_NUMBER; i++) {objects.append(relational::buildConstant_nikolayData(i));}
  TL::logicObjectManager::setConstants(objects);
  
  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);
  
  
#if 0
  MT::String file_rules;
  MT::getParameter(file_rules, "file_rules");
  PRINT_(file_rules);
  
  MT::String file_data;
  MT::getParameter(file_data, "file_data");
  PRINT_(file_data);
  
  MT::String file_prefix_parameters;
  MT::getParameter(file_prefix_parameters, "file_prefix_parameters");
  PRINT_(file_prefix_parameters);
  
  MT::String path_data;  path_data<<dir<<file_data;
  MT::String path_prefix_parameters;  path_prefix_parameters<<file_prefix_parameters;
   
  produce_rules(file_rules, path_data, path_prefix_parameters, grounding_type);
  HALT("raus");
#endif

  uint e, t;
  for (e=1; e<=5; e++) {
    for (t=1; t<=10; t++) {
      cout<<endl<<endl<<"==================================="<<endl;
      cout << "E" << e << "T" << t << endl<<endl;
      
      MT::String file_prefix_parameters;
      file_prefix_parameters << dir << "E" << e << "T" << t << "-";
      
      MT::String file_data;
      file_data << dir << "E" << e << "T" << t << ".txt";
      
      MT::String file_rules;
      file_rules << "rules/rules_E" << e << "T" << t << ".dat";
      
      produce_rules(file_rules, file_data, file_prefix_parameters, grounding_type);
    }
  }
}











// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------




RobotManipulationSimulator sim;


void initSimulator(const char* configurationFile, bool takeMovie) {
  sim.shutdownAll();
  sim.loadConfiguration(configurationFile);
#ifdef MT_FREEGLUT
  orsDrawProxies = false;
  orsDrawJoints = false;
#endif
  sim.startOde();
  if (takeMovie)
    sim.startRevel();
  sim.startSwift();
  sim.simulate(50);
}


#if 0
void experiment_simulator() {
  MT::String file_ors;
  MT::getParameter(file_ors, "file_ors");
  
  MT::String file_prefix_parameters;
  MT::getParameter(file_prefix_parameters, "file_prefix_parameters");
  
  uint grounding_type__uint;
  MT::getParameter(grounding_type__uint, "grounding_type");
  relational::SymbolGrounding::GroundingType grounding_type;
  if (grounding_type__uint == 0)
    grounding_type = relational::SymbolGrounding::NN;
  else
    grounding_type = relational::SymbolGrounding::RBF;
  PRINT_(grounding_type);
  
  
  uint i, k, t;
  
  // Set up simulator
  initSimulator(file_ors, false);
  sim.simulate(50);
//   sim.watch();
  
  // Set up logic
  TL::logicObjectManager::init("language.dat");
  TL::logicObjectManager::writeLanguage("used_language.dat");
  
  // Get objects and set in logic database
  uintA objs;
  sim.getObjects(objs);
  objs.append(sim.getHandID());
  TL::logicObjectManager::setConstants(objs);
  uintA movable_objs, balls, blocks;
  sim.getBalls(balls);
  sim.getBlocks(blocks);
  movable_objs.append(balls);  movable_objs.append(blocks);
  
  cout<<"Table-ID="<<sim.getTableID()<<endl;
  cout<<"Hand-ID="<<sim.getHandID()<<endl;
  
  // ------------------------------
  // SYMBOL GROUNDINGS
  // (i) Read parameters
  MT::Array<relational::SymbolGrounding*> sgs;
  relational::read(sgs, file_prefix_parameters, grounding_type);
  cout<<sgs.N<<" SymbolGroundings have been read."<<endl;
  
  
//   AtomL new_atoms;
//   uint i;
//   FOR1D(new_predicates, i) {
//     AtomL new_atoms_p;
//     TL::logicObjectManager::getAtoms(new_atoms_p, new_predicates(i),
// objs);
//     new_atoms.append(new_atoms_p);
//   }
//   cout<<"NEW ATOMS:  "<<new_atoms<<endl;
  
  
  AtomL actions;
  TL::logicObjectManager::getAtoms_actions(actions, objs); 
  PRINT(actions);
  
  
  // Action predicates
  TL::Predicate* p_grab =
    TL::logicObjectManager::getPredicate(MT::String("grab"));
  TL::Predicate* p_puton =
    TL::logicObjectManager::getPredicate(MT::String("puton"));

  for (t=0; t<10; t++) {
    cout<<"TIME-STEP t="<<t<<endl;
    // OBSERVE STATE
    TL::State* s = TL::RobotManipulationDomain::observeLogic(&sim);
    cout<<endl<<"OBSERVED STATE:"<<endl<<*s<<endl;
    
    // GROUNDED SYMBOLS
    LitL grounded_lits;
    relational::calculateLiterals(grounded_lits, sgs, sim.C);
    s->lits_prim.append(grounded_lits);
    
    TL::logicReasoning::derive(s);
    
    TL::Atom* action = actions(rnd.num(actions.N));
    cout<<"ACTION:  "<<*action<<endl;
    TL::RobotManipulationDomain::performAction(action, &sim, 100);
  }

}
#endif


void evaluate_rules_in_simulator(const char* file_rules, const char* file_ors, const SGL& sgl) {
  uint i, k, t;
  
  TL::Reward* reward = TL::RobotManipulationDomain::RewardLibrary::stack();

  TL::PRADA planner;
  planner.setReward(reward);
  planner.setHorizon(10);
  planner.setNumberOfSamples(1000);
  planner.setThresholdReward(0.1);
  planner.setNoiseSoftener(0.1);
  
  // Set up simulator
  initSimulator(file_ors, false);
  sim.simulate(50);
//   sim.watch();
  
  // Set up logic
//   logicObjectManager::shutdown();
  // Get objects and set in logic database
  uintA objs;
  sim.getObjects(objs);
  objs.append(sim.getHandID());
  TL::logicObjectManager::setConstants(objs);
  uintA movable_objs, balls, blocks;
  sim.getBalls(balls);
  sim.getBlocks(blocks);
  movable_objs.append(balls);  movable_objs.append(blocks);
  
  
  for (t=0; t<10; t++) {
    cout<<"TIME-STEP t="<<t<<endl;
    // OBSERVE STATE
    TL::State* state = TL::RobotManipulationDomain::observeLogic(&sim);
    cout<<endl<<"OBSERVED STATE:"<<endl<<*state<<endl;
    
    // GROUNDED SYMBOLS
    LitL grounded_lits;
    relational::calculateLiterals(grounded_lits, sgl, sim.C);
    state->lits_prim.append(grounded_lits);
    
    TL::logicReasoning::derive(state);
    
    TL::Atom* action = planner.generateAction(*state);
    cout<<"ACTION:  "<<*action<<endl;
    TL::RobotManipulationDomain::performAction(action, &sim, 100);
  }
  
}



void symbol_evaluation() {
  uint grounding_type__uint;
  MT::getParameter(grounding_type__uint, "grounding_type");
  relational::SymbolGrounding::GroundingType grounding_type;
  if (grounding_type__uint == 0)
    grounding_type = relational::SymbolGrounding::NN;
  else
    grounding_type = relational::SymbolGrounding::RBF;
  PRINT_(grounding_type);
  
  MT::String dir;
  if (grounding_type == relational::SymbolGrounding::NN)
    dir = "parameters_NN/";
  else if (grounding_type == relational::SymbolGrounding::RBF)
    dir = "parameters_RBF/";
  
  
  MT::String file_ors("situation.ors");
  
  // Set up logic
  TL::logicObjectManager::init("language.dat");
//   TL::logicObjectManager::writeLanguage("used_language.dat");
  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);

  uint e, t;
  for (e=1; e<=5; e++) {
    for (t=1; t<=10; t++) {
      cout<<endl<<endl<<"==================================="<<endl;
      cout << "E" << e << "T" << t << endl<<endl;
      
      // Symbol grounding
      MT::String file_prefix_parameters;
      file_prefix_parameters << dir << "E" << e << "T" << t << "-";
      SGL sgl;
      relational::read(sgl, file_prefix_parameters, grounding_type);
      cout<<sgl.N<<" SymbolGroundings have been read."<<endl;
      
      // Rule file
      MT::String file_rules;
      file_rules << "rules/" << "E" << e << "T" << t << "-";
      
      evaluate_rules_in_simulator(file_rules, file_ors, sgl);
      HALT("raus");
    }
  }
}

  

int main(int argc, char** argv){
  MT::String config_file("config");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
//   produce_rules();
  
  symbol_evaluation();
  
  return 0;
}
