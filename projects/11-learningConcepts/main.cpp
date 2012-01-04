#define MT_IMPLEMENT_TEMPLATES
#include <relational/robotManipulationDomain.h>
#include <relational/logicReasoning.h>
#include <relational/ruleLearner.h>
#include <relational/symbolGrounding.h>
#include <relational/robotManipulationSampling.h>
#include <relational/prada.h>



// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

void learn_rules(const char* file_rules, MT::Array< relational::FullExperience* >& experiences,
                   const MT::Array<relational::SymbolGrounding*> sgs) {
  uint i;
  // Calculate symbols
  FOR1D(experiences, i) {
    cout<<"##### Calculate symbols for experience i="<<i<<endl;
    relational::calculateLiterals(sgs, *experiences(i));
  }
  relational::FullExperience::write_symbolic(experiences, cout);
  cout<<"Symbols have been calculated in experiences."<<endl;
  relational::FullExperience::sanityCheck(experiences);
  cout<<"Sanity check successful."<<endl;
  ExperienceL symbolic_experiences;
  FOR1D(experiences, i) {
//     if (i>30) break;
    symbolic_experiences.append(&experiences(i)->experience_symbolic);
  }
//   double alpha_PEN = 0.01;
  double alpha_PEN = 10e-12;
  double p_min = 10e-8;
  double p_min_noisyDefaultRule = p_min;
  TL::RuleLearner learner(alpha_PEN, p_min, p_min_noisyDefaultRule);
  TL::RuleSetContainer rulesC;
  learner.learn_rules(rulesC, symbolic_experiences);
  cout<<"Learned rule-set:"<<endl;
  rulesC.writeNice();
//   rulesC.write_rulesWithExperiences();
  write(rulesC.rules, file_rules);
  cout<<rulesC.rules.num()<<" rules have been learned."<<endl;
//   HALT("stop");
}




void produce_rules() {
  bool read_nikolay;
  MT::getParameter(read_nikolay, "read_nikolay");
  PRINT(read_nikolay);
  
  uint grounding_type__uint;
  MT::getParameter(grounding_type__uint, "grounding_type");
  relational::SymbolGrounding::GroundingType grounding_type;
  grounding_type = relational::SymbolGrounding::GroundingType(grounding_type__uint);
  PRINT_(grounding_type);

  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);

  uint e, t, o;
  MT::Array< MT::Array < MT::String > > files_kombos;
  MT::String dir;
  if (grounding_type == relational::SymbolGrounding::NN)
    dir = "parameters_NN/";
  else if (grounding_type == relational::SymbolGrounding::RBF)
    dir = "parameters_RBF/";
  // (1) Nikolay's data
  if (read_nikolay) {
    for (t=1; t<=1; t++) {
      for (e=5; e<=5; e++) {
        MT::String file_prefix_parameters;
        file_prefix_parameters << dir << "E" << e << "T" << t << "-";
        MT::String file_data;
        file_data << dir << "E" << e << "T" << t << ".txt";
        MT::String file_rules;
        file_rules << "rules/rules_E" << e << "T" << t << ".dat";
        MT::Array< MT::String > kombo;
        kombo.append(file_prefix_parameters);
        kombo.append(file_data);
        kombo.append(file_rules);
        files_kombos.append(kombo);
      }
    }
  }
  // (2) My data
  else {
    MT::Array < MT::String > files_data;
    files_data.append(MT::String("experience_data/exp_o0.dat"));
    files_data.append(MT::String("experience_data/exp_o1.dat"));
    for (t=5; t<=10; t++) {
      for (e=5; e<=5; e++) {
        for (o=0; o<2; o++) {
          MT::String file_prefix_parameters;
          file_prefix_parameters << dir << "E" << e << "T" << t << "-";
          MT::String file_data;
          file_data << "experience_data/exp_o" << o << ".dat";
          MT::String file_rules;
          file_rules << "rules/rules_E" << e << "T" << t << "o" << o << ".dat";
          MT::Array< MT::String > kombo;
          kombo.append(file_prefix_parameters);
          kombo.append(file_data);
          kombo.append(file_rules);
          files_kombos.append(kombo);
        }
      }
    }
  }
  
  uint i, k;
//   FOR1D(files_kombos, i) {
//     PRINT(files_kombos(i));
//   }
  FOR1D(files_kombos, i) {
    cout<<endl<<endl<<"==================================="<<endl;
    cout<<"Kombo #"<<i<<" (out of "<<files_kombos.N<<"):  "<<files_kombos(i)<<endl;
    // SymbolGroundings
    MT::Array<relational::SymbolGrounding*> sgs;
    relational::read(sgs, files_kombos(i)(0), grounding_type);
    cout<<sgs.N<<" SymbolGroundings have been read."<<endl;
    // Experiences
    MT::Array< relational::FullExperience* > experiences;
    if (read_nikolay) {
      relational::FullExperience::read_continuous_nikolayFormat(experiences, files_kombos(i)(1));
    }
    else {
      relational::FullExperience::read_continuous(experiences, files_kombos(i)(1));
    }
    cout<<experiences.N<<" FullExperiences have been read."<<endl;
    if (i==0) {
      uint OBJ_NUMBER = experiences(0)->state_continuous_pre.N;
      uintA objects;
      for (k=0; k<OBJ_NUMBER; k++) {objects.append(relational::buildConstant(k));}
      TL::logicObjectManager::setConstants(objects);
    }
    PRINT(TL::logicObjectManager::constants);
    // Learn rules
    learn_rules(files_kombos(i)(2), experiences, sgs);
    FOR1D(experiences, k) {delete experiences(k);}
    FOR1D(sgs, k) {delete sgs(k);}
    HALT("schtop");
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


void run_simulator(const char* file_ors, const SGL& sgl, uint thinking_style = 1, MT::String file_rules = MT::String("")) {
  uint t;
  
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
//   TL::logicObjectManager::setConstants(objs);
  uintA movable_objs, balls, blocks;
  sim.getBalls(balls);
  sim.getBlocks(blocks);
  movable_objs.append(balls);  movable_objs.append(blocks);
  TL::logicObjectManager::setConstants(movable_objs);
  
  
  // rules
  TL::RuleSet rules;
  TL::RuleSet rules_grounded;
  TL::Reward* reward;
  TL::PRADA planner;
  if (thinking_style == 2) {
    // Rules
    TL::logicObjectManager::readRules(file_rules, rules);
    cout<<"Rules:"<<endl;  rules.write();
    TL::ruleReasoning::ground(rules_grounded, rules, movable_objs);
    cout<<"Grounded rules: "<<rules_grounded.num()<<endl;
    reward = TL::RobotManipulationDomain::RewardLibrary::stack();
    // geiler HACK
    TL::TransClosurePredicate* p_ABOVE1 = TL::RobotManipulationDomain::getPredicate_above();
    p_ABOVE1->basePred = TL::logicObjectManager::getPredicate(MT::String("b1")); // HACK da in regelfiles bis juni 2009 on andere id hat
    TL::logicObjectManager::writeLanguage("used_language.dat");
    // PRADA
    uint PRADA_num_samples;
    MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
    planner.setReward(reward);
    planner.setHorizon(4);
    planner.setNumberOfSamples(PRADA_num_samples);
    planner.setThresholdReward(0.1);
    planner.setNoiseSoftener(0.1);
    planner.setGroundRules(rules_grounded);
  }
  
  AtomL ground_actions;
  TL::logicObjectManager::getAtoms_actions(ground_actions, movable_objs);  // get all action atoms
  
  StateL seq_states;
  AtomL seq_actions;
  ExperienceL seq_exps;
  
  for (t=0; t<2; t++) {
    cout<<endl<<endl<<"++++++++++++++++++++++++++++++++++++++++++"<<endl;
    cout<<"TIME-STEP t="<<t<<endl;
    TL::State* state = TL::RobotManipulationDomain::observeLogic(&sim);
//     cout<<endl<<"OBSERVED STATE (unlearned):"<<endl<<*state<<endl;
    // Consider learned symbols
    LitL grounded_lits;
    relational::calculateLiterals(grounded_lits, sgl, sim.C);
    state->lits_prim.append(grounded_lits);
//     cout<<endl<<"OBSERVED STATE (unlearned):"<<endl<<*state<<endl;
    TL::logicReasoning::dederive(state);
    TL::logicReasoning::derive(state);
    seq_states.append(state);
    if (t > 0) {
      TL::Experience* ex = new TL::Experience(*seq_states(t-2), seq_actions(t-2), *seq_states(t-1));
      seq_exps.append(ex);
    }
    cout<<endl<<"OBSERVED STATE:"<<endl<<*state<<endl;
    
//     AtomL possible_actions;
//     FOR1D(as_actions, i) {
//       TL::RuleSet coveringGroundedRules;
//       TL::ruleReasoning::coveringGroundedRules_groundedAction(rules_grounded, *state, as_actions(i), coveringGroundedRules);
//       cout<<"Covering rules for "<<*as_actions(i)<<endl;
//       coveringGroundedRules.write();
//       possible_actions.append(as_actions(i));
//     }

    TL::Atom* action = NULL;
    if (thinking_style == 1) {
      action = ground_actions(rnd.num(ground_actions.N));
    }
    else if (thinking_style == 2) {
      action = planner.generateAction(*state);
    }
    else
      NIY;
    
    cout<<"ACTION:  "<<*action<<endl;
    seq_actions.append(action);
    TL::RobotManipulationDomain::performAction(action, &sim, 100);
    
    //   HALT("stop");
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
  
  
  MT::String file_ors("situation_big.ors");
  
  // Set up logic
  TL::logicObjectManager::init("language.dat");
//   TL::logicObjectManager::writeLanguage("used_language.dat");
  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);

  uint e, trial, run;
  //   for (t=1; t<=10; t++) {
    for (trial=5; trial<=10; trial++) {
//       for (e=2; e<=5; e++) {
        for (e=5; e<=5; e++) {
      for (run=0; run<50; run++) {
        cout<<endl<<endl<<"==================================="<<endl;
        cout << "Trial " << trial << " - episode " << e << endl<<endl;
        
        // Symbol grounding
        MT::String file_prefix_parameters;
        file_prefix_parameters << dir << "E" << e << "T" << trial << "-";
        SGL sgl;
        relational::read(sgl, file_prefix_parameters, grounding_type);
        cout<<sgl.N<<" SymbolGroundings have been read."<<endl;
        
        // Rule file
        MT::String file_rules;
        file_rules << "rules/rules_" << "E" << e << "T" << trial << ".dat";
        
        run_simulator(file_ors, sgl, 1, file_rules);
        HALT("raus");
      }
    }
  }
}



void collect_simulator_data() {
  MT::Array< MT::String > files_ors;
 // files_ors.append(MT::String("ors_situations/sit_10cubes_1.ors"));
 // files_ors.append(MT::String("ors_situations/sit_10cubes_2.ors"));
  files_ors.append(MT::String("situationNik.ors"));
  // Set up logic
  TL::logicObjectManager::init("language.dat");

  uint o, t, run, k;
  
  
  MT::Array<relational::SymbolGrounding*> sgs_E5T1;
  relational::read(sgs_E5T1, "parameters_RBF/E5T1-", relational::SymbolGrounding::RBF);
//   ((relational::RBF_Grounding*) sgs_E5T1(0))->w_c(0,1) -= 0.04;
  arr& w_c = ((relational::RBF_Grounding*) sgs_E5T1(0))->w_c;
//   PRINT(w_c);
  arr& w_sigma = ((relational::RBF_Grounding*) sgs_E5T1(0))->w_sigma;
  w_sigma(1) += 0.2;
  w_sigma(2) += 0.2;
//   PRINT(w_sigma);
  
  FOR1D(files_ors, o) {
    for (run=0; run<10; run++) {
      // Set up simulator
      initSimulator(files_ors(o), false);
      sim.simulate(50);
      //   sim.watch();
      // Get objects and set in logic database
      uintA objs;
      sim.getObjects(objs);
      PRINT(objs);
      TL::logicObjectManager::setConstants(objs);
      // Data collectors
      StateL seq_states;
      AtomL seq_actions;
      MT::Array< MT::Array< arr > > seq_states_cont;
      FullExperienceL seq_fex;
      StateL seq_states_learned;
      for (t=0; t<11; t++) {
        cout<<"***** TIME-STEP t="<<t<<"  (run="<<run<<", file="<<o<<") *****"<<endl;
        cerr<<"***** TIME-STEP t="<<t<<"  (run="<<run<<", file="<<o<<") *****"<<endl;
        TL::State* state = TL::RobotManipulationDomain::observeLogic(&sim);
        cout<<endl<<"OBSERVED STATE:"<<endl;  state->write(cout, true);  cout<<endl;
        seq_states.append(state);
        // Continuous data
        MT::Array< arr > objects_data;
        relational::getFeatureVectors(objects_data, *sim.C, objs);
        seq_states_cont.append(objects_data);
        // Nikolay's learned symbols
        LitL lits_learned;
        relational::SymbolGrounding::calculateLiterals(lits_learned, sgs_E5T1, objs, objects_data);
        TL::State* state_learned = new TL::State;
        state_learned->lits_prim = lits_learned;
        cout<<"LEARNED STATE DESCRIPTION (E5T5): "<<*state_learned<<endl;
        seq_states_learned.append(state_learned);
        // inhand object
        uint obj_inhand = sim.getInhand();
        if (obj_inhand != UINT_MAX) {
          arr f_obj_inhand;  relational::getFeatureVector(f_obj_inhand, *sim.C, obj_inhand);
          MT::String helfer;  helfer<<"u1("<<obj_inhand<<")";
          TL::Literal* lit_u1 = TL::logicObjectManager::getLiteral(helfer);
          bool u1_there = lits_learned.findValue(lit_u1)>=0;
          cout<<"inhand "<<obj_inhand<< " "<<f_obj_inhand<<"  --> "<<*lit_u1<<"="<<u1_there<<endl;
          cerr<<"inhand "<<obj_inhand<< " "<<f_obj_inhand<<"  --> "<<*lit_u1<<"="<<u1_there<<endl;
          PRINT(w_c);  PRINT2(w_c, cerr);
          arr diff = w_c - f_obj_inhand;
          PRINT(diff);  PRINT2(diff, cerr);
          PRINT(w_sigma);  PRINT2(w_sigma, cerr);
        }
        if (t > 0) {
          relational::FullExperience* fex = new relational::FullExperience;
          fex->state_continuous_pre = seq_states_cont(t-1);
          fex->state_continuous_post = seq_states_cont(t);
          fex->experience_symbolic.pre = *seq_states(t-1);
          fex->experience_symbolic.action = seq_actions(t-1);
          fex->experience_symbolic.post = *seq_states(t);
          fex->experience_symbolic.calcChanges();
          FOR1D(fex->experience_symbolic.action->args, k) {
            uint idx = objs.findValue(fex->experience_symbolic.action->args(k));
            CHECK(idx>=0, "arg not found");
            fex->action_args.append(idx);
          }
          if (seq_actions(t-1)->pred->id == HAND_ID__GRAB)
            fex->action_type = relational::FullExperience::grab;
          else
            fex->action_type = relational::FullExperience::puton;
          fex->reward = 0.;
          seq_fex.append(fex);
        }
        TL::Atom* action = TL::robotManipulationSampling::generateAction_wellBiased(*state, sim.getTableID());
        seq_actions.append(action);
        cout<<endl<<"ACTION:  "<<*action<<endl;
        TL::RobotManipulationDomain::performAction(action, &sim, 100);
      }
      MT::String file_exp;  file_exp<<"experience_data/exp_o"<<o<<"r"<<run<<".dat";
      ofstream out_file_exp(file_exp);
      relational::FullExperience::write_continuous(seq_fex, out_file_exp);
      
      MT::String file_exp_image;  file_exp_image<<"experience_data/img_exp_o"<<o<<"r"<<run<<".dat";
      ofstream out_file_exp_image(file_exp_image);
      FOR1D(seq_states, t) {
        out_file_exp_image << "***** t=" << t << " (o=" << o << ", r="<< run << ") *****"<<endl;
        TL::RobotManipulationDomain::writeStateInfo(*seq_states(t), out_file_exp_image);
        out_file_exp_image << endl << "Learned state: " << *seq_states_learned(t) << endl;
        if (t<seq_states.N-1) out_file_exp_image << endl << "Action:  " << *seq_actions(t) << endl << endl;
      }
    }
  }
}


/* TODO
 * (3) Regellernen mit meinen Daten zum Laufen bringen
 *     - SymbolGroundings finden fast keine u1-Praedikate
 * 
 *    + Tisch rausnehmen
 * 
 * HAUPTZIEL: gute Regeln lernen und die evaluieren
 */

#include "NikolayRoutines.h"

int main(int argc, char** argv){
  MT::String config_file("config");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);

  uint nik_data;
  MT::getParameter(nik_data, "nikolay_data");
  if (nik_data)
  NikGenerateData();


  uint rand_seed;
  MT::getParameter(rand_seed, "rand_seed");
  rnd.seed(rand_seed);
  PRINT_(rand_seed);
  
   collect_simulator_data();
  produce_rules();
//   symbol_evaluation();

//   FullExperienceL fel;
//   relational::FullExperience::read_continuous(fel, "exp_o0r0.dat");
  
  return 0;
}


