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
//     cout<<"##### Calculate symbols for experience i="<<i<<endl;
    relational::calculateLiterals(sgs, *experiences(i));
  }
  relational::FullExperience::write_symbolic(experiences, cout);
  cout<<"Symbols have been calculated in experiences."<<endl;
  relational::FullExperience::sanityCheck(experiences);
  cout<<"Sanity check successful."<<endl;
  SymbolicExperienceL symbolic_experiences;
  FOR1D(experiences, i) {
//     if (i>30) break;
    symbolic_experiences.append(&experiences(i)->experience_symbolic);
  }
  double alpha_PEN = 1.0;
//   double alpha_PEN = 10e-12;
  double p_min = 10e-7;
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

  uint e, trial, o;
  MT::Array< MT::Array < MT::String > > files_kombos;
  MT::String dir;
  if (grounding_type == relational::SymbolGrounding::NN)
    dir = "parameters_NN/";
  else if (grounding_type == relational::SymbolGrounding::RBF)
    dir = "parameters_RBF/";
  // (1) Nikolay's data
  if (read_nikolay) {
    for (trial=1; trial<=1; trial++) {
      for (e=5; e<=5; e++) {
        MT::String file_prefix_symbols;
        file_prefix_symbols << dir << "E" << e << "T" << trial << "-";
        MT::String file_data;
        file_data << dir << "E" << e << "T" << trial << ".txt";
        MT::String file_rules;
        file_rules << "rules/rules_E" << e << "T" << trial << ".dat";
        MT::Array< MT::String > kombo;
        kombo.append(file_prefix_symbols);
        kombo.append(file_data);
        kombo.append(file_rules);
        files_kombos.append(kombo);
      }
    }
  }
  // (2) My data
  else {
//     MT::Array < MT::String > files_data;
//     files_data.append(MT::String("data_transition_model_learning/exp_o0.dat"));
//     files_data.append(MT::String("data_transition_model_learning/exp_o1.dat"));
    for (trial=5; trial<=10; trial++) {
      for (e=5; e<=5; e++) {
        for (o=0; o<2; o++) {
          MT::String file_prefix_symbols;
          file_prefix_symbols << dir << "E" << e << "T" << trial << "-";
          MT::String file_data;
          file_data << "data_transition_model_learning/exp_o" << o << ".dat";
          MT::String file_rules;
          file_rules << "rules/rules_E" << e << "T" << trial << "_o" << o << ".dat";
          MT::Array< MT::String > kombo;
          kombo.append(file_prefix_symbols);
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
    
    // HACK
    arr& w_sigma = ((relational::RBF_Grounding*) sgs(0))->w_sigma;
    w_sigma(1) += 0.2;
    w_sigma(2) += 0.2;
    
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
      TL::logicObjectManager::setConstants(experiences(0)->state_continuous_pre.object_ids);
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



void run_simulator(const char* file_ors, const char* file_prefix_symbols, relational::SymbolGrounding::GroundingType grounding_type,
                   uint thinking_style = 1, MT::String file_rules = MT::String("")) {
  PRINT(thinking_style);
  cout<<"thinking_style=1 --> random,  thinking_style=2 --> planning"<<endl;
  
  // Set up simulator
  initSimulator(file_ors, false);
  sim.simulate(50);
//   sim.watch();
  
  // Set up logic
  //   logicObjectManager::shutdown();
  TL::logicObjectManager::init("language.dat");
//   TL::logicObjectManager::writeLanguage("used_language.dat");
  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);
  
  // Symbol Groundings
  SGL sgl;
  relational::read(sgl, file_prefix_symbols, grounding_type);
  cout<<sgl.N<<" SymbolGroundings have been read."<<endl;
  // HACK
  arr& w_sigma = ((relational::RBF_Grounding*) sgl(0))->w_sigma;
  w_sigma(1) += 0.2;
  w_sigma(2) += 0.2;
  
  // Set up logic
  // Get objects and set in logic database
//   uintA objs;
//   sim.getObjects(objs);
//   objs.append(sim.getHandID());
//   TL::logicObjectManager::setConstants(objs);
  uintA movable_objs, balls, blocks;
  sim.getBalls(balls);
  sim.getBlocks(blocks);
  movable_objs.append(balls);  movable_objs.append(blocks);
  TL::logicObjectManager::setConstants(movable_objs);
  
  // Rules
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
    cout<<"Reward:"; reward->writeNice();
    // PRADA
    uint PRADA_num_samples, PRADA_horizon;
    MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
    MT::getParameter(PRADA_horizon, "PRADA_horizon");
    planner.setReward(reward);
    planner.setHorizon(PRADA_horizon);
    planner.setNumberOfSamples(PRADA_num_samples);
    planner.setThresholdReward(0.1);
    planner.setNoiseSoftener(0.1);
    planner.setGroundRules(rules_grounded);
  }
  
  
  AtomL ground_actions;
  TL::logicObjectManager::getAtoms_actions(ground_actions, movable_objs);  // get all action atoms
  PRINT(ground_actions);
  
  StateL seq_states_unlearned;
  StateL seq_states_learned;
  AtomL seq_actions;
  SymbolicExperienceL seq_exps;
  
  uint run, t;

#define NUM_RUNS 1
#define NUM_TIMESTEPS 10
  
  for (run=0; run<NUM_RUNS; run++) {
    for (t=0; t<NUM_TIMESTEPS; t++) {
      cout<<endl<<endl<<"+++++ t="<<t<<" (run="<<run<<")  +++++"<<endl;
      cerr<<endl<<endl<<"+++++ t="<<t<<" (run="<<run<<")  +++++"<<endl;
      TL::State* state_unlearned = TL::RobotManipulationDomain::observeLogic(&sim);
      cout<<endl<<"OBSERVED STATE (unlearned):"<<endl;
      TL::RobotManipulationDomain::writeStateInfo(*state_unlearned);   TL::RobotManipulationDomain::writeStateInfo(*state_unlearned, cerr);
      seq_states_unlearned.append(state_unlearned);
      // Nikolay's learned symbols
      LitL lits_learned;
      relational::calculateLiterals(lits_learned, sgl, sim.C);
      TL::State* state_learned = new TL::State;
      state_learned->lits_prim = lits_learned;
      cout<<"LEARNED STATE DESCRIPTION: "<<*state_learned<<endl;
      seq_states_learned.append(state_learned);
      TL::logicReasoning::derive(state_learned);
      double reward_value = reward->evaluate(*state_learned);
      PRINT(reward_value);  PRINT2(reward_value, cerr);
      if (t > 0) {
        TL::SymbolicExperience* ex = new TL::SymbolicExperience(*seq_states_learned(t-1), seq_actions(t-1), *seq_states_learned(t));
        seq_exps.append(ex);
      }
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
        action = planner.generateAction(*state_learned);
      }
      else
        NIY;
      
      cout<<"ACTION:  "<<*action<<endl;
      cerr<<"ACTION:  "<<*action<<endl;
      seq_actions.append(action);
      TL::RobotManipulationDomain::performAction(action, &sim, 100);
      
      //   HALT("stop");
    }
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

  MT::Array< MT::String > files_ors;
  files_ors.append(MT::String("ors_situations/sit_10cubes_1.ors"));
  files_ors.append(MT::String("ors_situations/sit_10cubes_2.ors"));
//   MT::String file_ors("situation_big.ors");
  
  MT::Array < MT::Array < MT::String > > kombos;
  uint e, trial, i;
  for (trial=5; trial<=10; trial++) {
    for (e=5; e<=5; e++) {
      MT::String file_prefix_symbols;
      file_prefix_symbols << dir << "E" << e << "T" << trial << "-";
      MT::String file_rules;
      file_rules << "rules/rules_" << "E" << e << "T" << trial << "_o0.dat";     
      FOR1D(files_ors, i) {
        MT::Array< MT::String > kombo;
        kombo.append(files_ors(i));
        kombo.append(file_prefix_symbols);
        kombo.append(file_rules);
        kombos.append(kombo);
      }
    }
  }

  FOR1D(kombos, i) {
    cout<<"++++++++++ Kombo "<<i<<" ++++++++++"<<endl;
    cout<<kombos(i)<<endl;
    run_simulator(kombos(i)(0), kombos(i)(1), grounding_type, 2, kombos(i)(2));
    HALT("raus");
  }
}









// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------



void collect_simulator_data() {
  MT::Array< MT::String > files_ors;
  files_ors.append(MT::String("ors_situations/sit_10cubes_1.ors"));
  files_ors.append(MT::String("ors_situations/sit_10cubes_2.ors"));
  
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
  
  TL::Reward* reward = TL::RobotManipulationDomain::RewardLibrary::stack();
  // geiler HACK
  TL::TransClosurePredicate* p_ABOVE1 = TL::RobotManipulationDomain::getPredicate_above();
  p_ABOVE1->basePred = TL::logicObjectManager::getPredicate(MT::String("b1")); // HACK da in regelfiles bis juni 2009 on andere id hat
  cout<<"Reward:"; reward->writeNice();
  
#define COLLECT_DATA__NUM_RUNS 10
#define COLLECT_DATA__NUM_TIMESTEPS 10
  
  FOR1D(files_ors, o) {
    for (run=0; run<COLLECT_DATA__NUM_RUNS; run++) {
      // Set up simulator
      initSimulator(files_ors(o), false);
      sim.simulate(50);
      //   sim.watch();
      // Get objects and set in logic database
//       uintA objs;
//       sim.getObjects(objs);
//       PRINT(objs);
      uintA movable_objs, balls, blocks;
      sim.getBalls(balls);
      sim.getBlocks(blocks);
      movable_objs.append(balls);  movable_objs.append(blocks);
      TL::logicObjectManager::setConstants(movable_objs);
      // Data collectors
      StateL seq_states_unlearned;
      StateL seq_states_learned;
      AtomL seq_actions;
      MT::Array< relational::ContinuousState* > seq_states_cont;
      FullExperienceL seq_fex;
      arr seq_rewards;
      for (t=0; t<COLLECT_DATA__NUM_TIMESTEPS; t++) {
        cout<<"***** TIME-STEP t="<<t<<"  (run="<<run<<", file="<<o<<") *****"<<endl;
        cerr<<"***** TIME-STEP t="<<t<<"  (run="<<run<<", file="<<o<<") *****"<<endl;
        TL::State* state_unlearned = TL::RobotManipulationDomain::observeLogic(&sim);
        cout<<endl<<"OBSERVED STATE:"<<endl;  TL::RobotManipulationDomain::writeStateInfo(*state_unlearned);
        seq_states_unlearned.append(state_unlearned);
        // Continuous data
        relational::ContinuousState* cont_state = relational::getContinuousState(*sim.C, movable_objs);
        seq_states_cont.append(cont_state);
//         cout<<"CONT STATE:"<<endl;  cont_state->write(cout);
        // Nikolay's learned symbols
        LitL lits_learned;
        relational::SymbolGrounding::calculateLiterals(lits_learned, sgs_E5T1, *cont_state);
        TL::State* state_learned = new TL::State;
        state_learned->lits_prim = lits_learned;
        state_learned->state_objects = movable_objs;
        cout<<"LEARNED STATE DESCRIPTION (E5T5): "<<*state_learned<<endl;
        TL::logicReasoning::derive(state_learned);
        seq_states_learned.append(state_learned);
        // inhand object
#if 0
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
#endif
        double reward_value = reward->evaluate(*state_learned);
        PRINT(reward_value);  PRINT2(reward_value, cerr);
        seq_rewards.append(reward_value);
        if (t > 0) {
          relational::FullExperience* fex = new relational::FullExperience;
          fex->state_continuous_pre = *seq_states_cont(t-1);
          fex->state_continuous_post = *seq_states_cont(t);
          fex->experience_symbolic.pre = *seq_states_learned(t-1);
          fex->experience_symbolic.action = seq_actions(t-1);
          fex->experience_symbolic.post = *seq_states_learned(t);
          fex->experience_symbolic.calcChanges();
          fex->reward = seq_rewards(t-1);
          fex->action_args = seq_actions(t-1)->args;
//           FOR1D(fex->experience_symbolic.action->args, k) {
//             uint idx = movable_objs.findValue(fex->experience_symbolic.action->args(k));
//             CHECK(idx>=0, "arg not found");
//             fex->action_args.append(idx);
//           }
          if (seq_actions(t-1)->pred->id == HAND_ID__GRAB)
            fex->action_type = relational::FullExperience::grab;
          else
            fex->action_type = relational::FullExperience::puton;
          seq_fex.append(fex);
        }
        TL::Atom* action = TL::robotManipulationSampling::generateAction_wellBiased(*state_unlearned, sim.getTableID());
        seq_actions.append(action);
        cout<<endl<<"ACTION:  "<<*action<<endl;
        TL::RobotManipulationDomain::performAction(action, &sim, 100);
      }
      MT::String file_exp;  file_exp<<"data_generation_tobias/exp_o"<<o<<"r"<<run<<".dat";
      ofstream out_file_exp(file_exp);
      relational::FullExperience::write_continuous(seq_fex, out_file_exp);
      
      MT::String file_exp_image;  file_exp_image<<"data_generation_tobias/img_exp_o"<<o<<"r"<<run<<".dat";
      ofstream out_file_exp_image(file_exp_image);
      FOR1D(seq_states_unlearned, t) {
        out_file_exp_image << "***** t=" << t << " (o=" << o << ", r="<< run << ") *****"<<endl;
        TL::RobotManipulationDomain::writeStateInfo(*seq_states_unlearned(t), out_file_exp_image);
        out_file_exp_image << endl << "Learned state: " << *seq_states_learned(t) << endl;
        if (t<seq_states_unlearned.N-1) out_file_exp_image << endl << "Action:  " << *seq_actions(t) << endl << endl;
      }
    }
  }
}




void example_for_nikolay() {
  // Get continuous data
  MT::Array< relational::FullExperience* > experiences;
  relational::FullExperience::read_continuous(experiences, "data_generation_tobias/exp_o0.dat");
  
  // Logic (1):  Create symbolic actions
  TL::Predicate* p_GRAB = TL::RobotManipulationDomain::getPredicate_action_grab();
  TL::Predicate* p_PUTON = TL::RobotManipulationDomain::getPredicate_action_puton();
  PredL p_action;  p_action.append(p_GRAB);  p_action.append(p_PUTON);
  TL::logicObjectManager::addActionPredicates(p_action);
  // Logic (2):  Set constants
  TL::logicObjectManager::setConstants(experiences(0)->state_continuous_pre.object_ids);
  
  // Get symbol groundings
  MT::Array<relational::SymbolGrounding*> sgs;
  relational::read(sgs, "parameters_RBF/E5T1-", relational::SymbolGrounding::RBF);
  cout<<sgs.N<<" SymbolGroundings have been read."<<endl;
  
  // Calculate symbols in data
  uint i;
  FOR1D(experiences, i) {relational::calculateLiterals(sgs, *experiences(i));}
  
  // Some example manipulations
  
  // write symbolic experiences
  relational::FullExperience::write_symbolic(experiences, cout);  cout<<endl<<endl<<endl;
  
  // get symbols of first experience
  TL::State& state_first_pre = experiences(0)->experience_symbolic.pre;
  cout<<"Primitive learned literals in pre-state of first experience:"<<endl<<state_first_pre.lits_prim<<endl<<endl;
  TL::State& state_first_post = experiences(0)->experience_symbolic.post;
  cout<<"Primitive learned literals in post-state of first experience:"<<endl<<state_first_post.lits_prim<<endl<<endl;
  
  // get some literals
  TL::Predicate* p_U1 = TL::logicObjectManager::getPredicate(MT::String("u1"));
  TL::Predicate* p_B1 = TL::logicObjectManager::getPredicate(MT::String("b1"));
  cout<<"Logic constants:  "<<TL::logicObjectManager::constants<<endl;
  uintA args;  args.append(TL::logicObjectManager::constants(0));  args.append(TL::logicObjectManager::constants(1));
  TL::Literal* lit1 = TL::logicObjectManager::getLiteral(p_B1, true, args);
  cout<<"lit1: "<<*lit1<<endl;
  TL::Literal* lit2 = TL::logicObjectManager::getLiteral("-u1(70)");
  cout<<"lit2: "<<*lit2<<endl;
  
  // some logic reasoning
  bool holds1 = TL::logicReasoning::holds(state_first_pre, lit1);
  cout<<"Does "<<*lit1<<" hold in state"<<endl<<state_first_pre<<"?  --> "<<holds1<<endl<<endl;
  bool holds2 = TL::logicReasoning::holds(state_first_pre, lit2);
  cout<<"Does "<<*lit2<<" hold in state"<<endl<<state_first_pre<<"?  --> "<<holds2<<endl<<endl;
  
  // ...
}



/* TODO
 * 
 * HAUPTZIEL: gute Regeln lernen und die evaluieren
 */

#include "NikolayRoutines.h"

int main(int argc, char** argv){
  MT::String config_file("config");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);

  uint rand_seed;
  MT::getParameter(rand_seed, "rand_seed");
  rnd.seed(rand_seed);
  PRINT_(rand_seed);
  
  
  
//   uint nik_data;
//   MT::getParameter(nik_data, "nikolay_data");
//   if (nik_data)
//   NikGenerateData();
  example_for_nikolay();
  
//   collect_simulator_data();
//   produce_rules();
//   symbol_evaluation();
  
  return 0;
}


