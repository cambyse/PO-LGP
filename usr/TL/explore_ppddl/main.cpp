#include <iostream>
#include <map>
#include <ctime>
#include <stdlib.h>
#include <cstdlib>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/util.h>
#include <TL/logicDefinitions.h>
#include <TL/logicReasoning.h>
#include <TL/ruleReasoning.h>
#include <TL/plan.h>
#include <TL/ruleExplorer.h>
#include <TL/ppddl.h>
#include <TL/experiments_fixedContexts.h>

/*
#define REWARD_SUCCESS 1
#define REWARD_FAILED_INTERRUPT 2
#define REWARD_FAILED_TIMEOUT 3




*/





// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------







void experiment_exploration() {
  cout<<"***** experiment_exploration PPDDL *****"<<endl;
  
  // -------------------------------------
  // READ CONFIG
  // -------------------------------------
  
  uint randSeed;
  MT::getParameter(randSeed, "randSeed");
  rnd.seed(randSeed);
  PRINT(randSeed);

 
  uint representation_uint;
  MT::getParameter(representation_uint, "representation");
  PRINT_(representation_uint);
  TL::RuleExplorer::RepresentationType representation;
  switch(representation_uint) {
    case TL::RuleExplorer::relational: representation = TL::RuleExplorer::relational;  break;
    case TL::RuleExplorer::factored: representation = TL::RuleExplorer::factored;  break;
    case TL::RuleExplorer::flat: representation = TL::RuleExplorer::flat;  break;
    default: NIY;
  }
  PRINT(representation);
  
  uint behavior_type;
  MT::getParameter(behavior_type, "behavior_type");
  PRINT(behavior_type);
  
  bool use_known_state_partial;
  MT::getParameter(use_known_state_partial, "use_known_state_partial");
  PRINT(use_known_state_partial);
  
  
  
  uint PRADA_horizon;
  MT::getParameter(PRADA_horizon, "PRADA_horizon");
  PRINT(PRADA_horizon);
  
  uint PRADA_num_samples;
  MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
  PRINT(PRADA_num_samples);
  
  double PRADA_noise_softener;
  MT::getParameter(PRADA_noise_softener, "PRADA_noise_softener");
  PRINT(PRADA_noise_softener);
  
  double discountFactor;
  MT::getParameter(discountFactor, "discountFactor");
  PRINT(discountFactor);
  
    
  double rule_learning__alpha_coeff__abstract;
  MT::getParameter(rule_learning__alpha_coeff__abstract, "abstract__complexity_penalty_coeff");
  PRINT(rule_learning__alpha_coeff__abstract);
  
  double abstract__p_lower_bound__noise_outcome;
  MT::getParameter(abstract__p_lower_bound__noise_outcome, "abstract__p_lower_bound__noise_outcome");
  PRINT(abstract__p_lower_bound__noise_outcome);
  
  double abstract__p_lower_bound__noise_outcome_in_default_rule;
  MT::getParameter(abstract__p_lower_bound__noise_outcome_in_default_rule, "abstract__p_lower_bound__noise_outcome_in_default_rule");
  PRINT(abstract__p_lower_bound__noise_outcome_in_default_rule);
  
  double rule_learning__alpha_coeff__factored;
  MT::getParameter(rule_learning__alpha_coeff__factored, "factored__complexity_penalty_coeff");
  PRINT(rule_learning__alpha_coeff__factored);
  
  double factored__p_lower_bound__noise_outcome;
  MT::getParameter(factored__p_lower_bound__noise_outcome, "factored__p_lower_bound__noise_outcome");
  PRINT(factored__p_lower_bound__noise_outcome);
  
  double factored__p_lower_bound__noise_outcome_in_default_rule;
  MT::getParameter(factored__p_lower_bound__noise_outcome_in_default_rule, "factored__p_lower_bound__noise_outcome_in_default_rule");
  PRINT(factored__p_lower_bound__noise_outcome_in_default_rule);

  double rule_learning__alpha_coeff, p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule;
  if (representation == TL::RuleExplorer::relational) {
    rule_learning__alpha_coeff = rule_learning__alpha_coeff__abstract;
    p_lower_bound__noise_outcome = abstract__p_lower_bound__noise_outcome;
    p_lower_bound__noise_outcome_in_default_rule = abstract__p_lower_bound__noise_outcome_in_default_rule;
  }
  else if (representation == TL::RuleExplorer::factored) {
    rule_learning__alpha_coeff = rule_learning__alpha_coeff__factored;
    p_lower_bound__noise_outcome = factored__p_lower_bound__noise_outcome;
    p_lower_bound__noise_outcome_in_default_rule = factored__p_lower_bound__noise_outcome_in_default_rule;
  }
  else if (representation == TL::RuleExplorer::flat) {
    rule_learning__alpha_coeff = 0.;
    p_lower_bound__noise_outcome = 0.;
    p_lower_bound__noise_outcome_in_default_rule = 0.;
  }
  else
    NIY;
  PRINT(rule_learning__alpha_coeff);
  PRINT(p_lower_bound__noise_outcome);
  PRINT(p_lower_bound__noise_outcome_in_default_rule);
  
  
  bool watch;
  MT::getParameter(watch, "watch");
  PRINT(watch);
  
  uint num_trials;
  MT::getParameter(num_trials, "num_trials");
  PRINT(num_trials);
  
  int experience_id;
  MT::getParameter(experience_id, "experience_id");
  PRINT(experience_id);
  
  uint num_rounds;
  MT::getParameter(num_rounds, "num_rounds");
  PRINT(num_rounds);
  
  uint max_actions;
  MT::getParameter(max_actions, "max_actions");
  PRINT(max_actions);
  
  uint secs_wait;
  MT::getParameter(secs_wait, "secs_wait");
  PRINT(secs_wait);
  
  MT::String languageFile_name;
  MT::getParameter(languageFile_name, "file_language");
  PRINT(languageFile_name);
  
  
  
  uint num_constants;
  MT::getParameter(num_constants, "num_constants", (uint) 100);
  PRINT(num_constants);
  
  MT::String worldRulesFile_name;
  MT::getParameter(worldRulesFile_name, "file_world_rules");
  PRINT(worldRulesFile_name);
  
  MT::String name_file_ground_world_rules;
  MT::getParameter(name_file_ground_world_rules, "file_ground_world_rules");
  PRINT(name_file_ground_world_rules);
  
  
  MT::String filename_results;
  MT::getParameter(filename_results, "file_results");
  PRINT(filename_results);
   
  bool fixed_contexts;
  MT::getParameter(fixed_contexts, "fixed_contexts", false);
  PRINT(fixed_contexts);
  
  
  bool do_rounds_with_same_file;
  MT::getParameter(do_rounds_with_same_file, "do_rounds_with_same_file");
  PRINT(do_rounds_with_same_file);
  
  MT::String file_ppddl_problem;
  
  MT::Array< MT::String > files_ors;
  MT::String file_ppddl_problem_1, file_ppddl_problem_2, file_ppddl_problem_3, file_ppddl_problem_4, file_ppddl_problem_5,
             file_ppddl_problem_6, file_ppddl_problem_7, file_ppddl_problem_8, file_ppddl_problem_9, file_ppddl_problem_10;

  MT::Array< MT::String > files_reward;
  MT::String file_reward_1, file_reward_2, file_reward_3, file_reward_4, file_reward_5,
             file_reward_6, file_reward_7, file_reward_8, file_reward_9, file_reward_10;
  
  if (do_rounds_with_same_file) {
    MT::getParameter(file_ppddl_problem, "file_ppddl_problem");
    PRINT(file_ppddl_problem);
  }
  else {
    MT::getParameter(file_ppddl_problem_1, "file_ppddl_problem_1");
    PRINT(file_ppddl_problem_1);
    files_ors.append(file_ppddl_problem_1);
  
    MT::getParameter(file_ppddl_problem_2, "file_ppddl_problem_2");
    PRINT(file_ppddl_problem_2);
    files_ors.append(file_ppddl_problem_2);
  
    MT::getParameter(file_ppddl_problem_3, "file_ppddl_problem_3");
    PRINT(file_ppddl_problem_3);
    files_ors.append(file_ppddl_problem_3);
  
    MT::getParameter(file_ppddl_problem_4, "file_ppddl_problem_4");
    PRINT(file_ppddl_problem_4);
    files_ors.append(file_ppddl_problem_4);
  
    MT::getParameter(file_ppddl_problem_5, "file_ppddl_problem_5");
    PRINT(file_ppddl_problem_5);
    files_ors.append(file_ppddl_problem_5);
  
    MT::getParameter(file_ppddl_problem_6, "file_ppddl_problem_6");
    PRINT(file_ppddl_problem_6);
    files_ors.append(file_ppddl_problem_6);
  
    MT::getParameter(file_ppddl_problem_7, "file_ppddl_problem_7");
    PRINT(file_ppddl_problem_7);
    files_ors.append(file_ppddl_problem_7);
  
    MT::getParameter(file_ppddl_problem_8, "file_ppddl_problem_8");
    PRINT(file_ppddl_problem_8);
    files_ors.append(file_ppddl_problem_8);
  
    MT::getParameter(file_ppddl_problem_9, "file_ppddl_problem_9");
    PRINT(file_ppddl_problem_9);
    files_ors.append(file_ppddl_problem_9);
  
    MT::getParameter(file_ppddl_problem_10, "file_ppddl_problem_10");
    PRINT(file_ppddl_problem_10);
    files_ors.append(file_ppddl_problem_10);
  }
  
  
  
  
  
  
 
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
  
  // Create LogicEngine object
  TL::logicObjectManager::setPredicatesAndFunctions(languageFile_name, 2);
  TL::logicObjectManager::writeLanguage("used_language.dat");
  
  
  
  // -------------------------------------
  //  PROBLEM
  // -------------------------------------
  LitL dummy_lits;
  TL::LiteralListReward reward(dummy_lits);
//   TL::MaximizeFunctionReward reward;
  TL::State s0;
  TL::readPPDDLdomain(s0, reward, file_ppddl_problem);
  cout<<"STATE s0: "<<s0<<endl;
  cout<<"REWARD: "; reward.writeNice(); cout<<endl;
//   if (TL::LOGIC_ENGINE__MAX_ID_NUMBER <= le.constants.N + 20) {
//     cout<<"Too few constants: #constants = " << le.constants.N << endl;
//   }
  
  
  // -------------------------------------
  // WORLD RULES
  // -------------------------------------
  TL::RuleSet world_rules;
  TL::logicObjectManager::readRules(worldRulesFile_name, world_rules);
  cout<<"WORLD RULES:"<<endl;
  world_rules.write();
  
  
  // -------------------------------------
  // GROUND THE WORLD RULES
  // -------------------------------------
  TL::RuleSet ground_world_rules;
  if (name_file_ground_world_rules.N() > 39) {
    // boxworld
    // boxworld_ground_rules_2.dat
    // 19 bis vor "_ground"
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-27));
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-26));
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-25));
    // rectangle
    // rectangle_tireworld_ground_rules_2.dat
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-39));
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-38));
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-37));
    PRINT(name_file_ground_world_rules(name_file_ground_world_rules.N()-36));
  }
  if (   name_file_ground_world_rules.N() > 25
      && (   (   name_file_ground_world_rules(name_file_ground_world_rules.N()-27) == 'b'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-26) == 'o'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-25) == 'x')
              || (    name_file_ground_world_rules(name_file_ground_world_rules.N()-38) == 'r'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-37) == 'e'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-36) == 'c')
              || (    name_file_ground_world_rules(name_file_ground_world_rules.N()-39) == 'r'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-38) == 'e'
                  &&  name_file_ground_world_rules(name_file_ground_world_rules.N()-37) == 'c')
          )
     ) {
    cout<<"Reading ground rules from file."<<endl;
    TL::logicObjectManager::readRules(name_file_ground_world_rules, ground_world_rules);
  }
  else {
    cout<<"Building ground rules via ground_with_filtering."<<endl;
    TL::ruleReasoning::ground_with_filtering(ground_world_rules, world_rules, TL::logicObjectManager::constants, s0, true);
  }

  //   TL::ruleReasoning::removeNonChangingConcepts(ground_rules, rules);
  TL::ruleReasoning::removeDoubleLiterals(ground_world_rules);
  TL::ruleReasoning::checkRules(ground_world_rules);
  
  cout<<"GROUND RULES: (plenty!!)"<<endl;
  cout<<"# = "<<ground_world_rules.num()<<endl;
//   ground_world_rules.writeNice();
  TL::write(ground_world_rules, "ground_world_rules.dat");
//   exit(0);
  
  
  
  // -------------------------------------
  // EXPERIMENT
  // -------------------------------------
  
  double t_start, t_finish;
  
  FILE* f_results;
  f_results = fopen(filename_results, "w");
  fprintf(f_results, "#  Trial  Round  Success  Actions  Rules  Explores_Direct   Explores_Planned   Exploits   ExploitEnd   NumObjs\n");
  fflush(f_results);
  
 
  uint k, a, i_round, i_trial;
  uintA num_actions;
  uintA num_successes;
  
  
  
  for (i_trial=0; i_trial<num_trials; i_trial++) {
    cout<<endl<<endl<<endl<<endl<<"===== TRIAL "<<i_trial<<" ====="<<endl;
    cerr<<endl<<endl<<endl<<endl<<"===== TRIAL "<<i_trial<<" ====="<<endl;
    
    TL::RuleExplorer* explorer;
    if (representation == TL::RuleExplorer::relational) {
      TL::RuleSet fixed_rules;
      FOR1D_(world_rules, k) {
        if (world_rules.elem(k)->action->pred == TL::logicObjectManager::getPredicate(MT::String("end-mission"))) {
          fixed_rules.append(world_rules.elem(k));
        }
      }
      if (!fixed_contexts) {
        explorer = new TL::AbstractRuleExplorer(rule_learning__alpha_coeff__abstract,
                                              abstract__p_lower_bound__noise_outcome, abstract__p_lower_bound__noise_outcome_in_default_rule,
                                              fixed_rules, 0);
      }
      else {
        TL::RuleSet fixed_partial_rules;
        FOR1D_(world_rules, k) {
          if (!TL::ruleReasoning::isDefaultRule(world_rules.elem(k)))
            fixed_partial_rules.append(world_rules.elem(k));
        }
        explorer = new TL::AbstractRuleExplorer_FixedContexts(fixed_partial_rules, rule_learning__alpha_coeff__abstract,
                                              abstract__p_lower_bound__noise_outcome, abstract__p_lower_bound__noise_outcome_in_default_rule,
                                              fixed_rules, 0);
      }
    }
    else if (representation == TL::RuleExplorer::factored) {
      TL::RuleSet fixed_rules;
      FOR1D_(ground_world_rules, k) {
        if (ground_world_rules.elem(k)->action->pred == TL::logicObjectManager::getPredicate(MT::String("end-mission"))) {
          fixed_rules.append(ground_world_rules.elem(k));
        }
      }
      explorer = new TL::FactoredRuleExplorer(rule_learning__alpha_coeff__abstract, factored__p_lower_bound__noise_outcome,
                                              factored__p_lower_bound__noise_outcome_in_default_rule, fixed_rules);
    }
    else if (representation == TL::RuleExplorer::flat) {
      TL::RuleSet fixed_rules;
      FOR1D_(ground_world_rules, k) {
        if (ground_world_rules.elem(k)->action->pred == TL::logicObjectManager::getPredicate(MT::String("end-mission"))) {
          fixed_rules.append(ground_world_rules.elem(k));
        }
      }
      explorer = new TL::FlatExplorer(fixed_rules);
    }
    else
      NIY;
    if (experience_id >= 0) {
      MT::String data_file;
      data_file << "experiences_" << experience_id << ".dat";
      cout<<"Reading experience file " << data_file << endl;
      TL::Trial* read_trial = TL::logicObjectManager::readTrial_withConstants(data_file, false);
      for (k=1; k<read_trial->states.N; k++) {
        TL::logicObjectManager::makeOriginal(*read_trial->states(k-1));
        read_trial->actions(k-1) = TL::logicObjectManager::getAtomOrig(read_trial->actions(k-1));
        if (k==read_trial->states.N-1)
          TL::logicObjectManager::makeOriginal(*read_trial->states(k));
      }
      explorer->addObservations(*read_trial);
      delete read_trial;
    }
    
    TL::Trial trial;
    trial.constants = TL::logicObjectManager::constants;
    
    arr rewards_trial_states(num_rounds);
    arr rewards_trial_actions(num_rounds);
    
//     try {

    // -------------------------------------
    //   ROUND  [start]
    // -------------------------------------
    for (i_round= 0; i_round < num_rounds; i_round++) {
      if (num_rounds> 0) {
        cout<<endl<<endl<<endl<<endl<<"----- ROUND "<<i_round<<" -----"<<endl;
        cerr<<endl<<endl<<endl<<endl<<"----- ROUND "<<i_round<<" -----"<<endl;
      }
      
      if (representation == TL::RuleExplorer::factored  ||  representation == TL::RuleExplorer::flat) {
        if (i_round==0)
          explorer->updateLogicEngineConstants();
        else if (!do_rounds_with_same_file && i_round > 0)
          explorer->reset();
        else {
          // don't do anything
        }
      }

      cout<<"REWARD: ";  reward.writeNice(); cout<<endl;
      
    
      TL::Atom* action = NULL;
      TL::State* current_state = new TL::State;
      *current_state = s0;
      AtomL executed_actions;
      uintA exploits;
      uintA explores_planned;
      uintA explores_direct;
      
      ofstream actions_file("performed_actions.dat");

      
      uintA changes;
      arr rewards_round_action;
      
      // -------------------------------------
      //   ROUND-RUN  [start]
      // -------------------------------------
      
      bool failed = false;
      
      
      if (representation == TL::RuleExplorer::relational  &&  i_round > 0) {
        // TODO gilt das auch fuer die anderen IPPC-Domaenen?
        cerr<<"Setting  explorer->is_major_experience.last() = true;"<<endl;
        cout<<"Setting  explorer->is_major_experience.last() = true;"<<endl;
        explorer->is_major_experience.last() = true;  // als HACK: damit er potentiell exploiten in erwaegung zieht
      }
      
      AtomL good_old_plan;
      
      for (a=0; a<max_actions+1; a++) {
        // Observe current state and update explorer
        CHECK(current_state->lits_prim.N != 0, "current_state is not set");
        TL::State* old_state = current_state;
        if (a==0) {
          // TODO 
//           current_state = &s0;
        }
        else {
          // STATE TRANSITION
          uint dummy_flag;
          TL::State s_suc;
          double action_reward;
          action_reward = TL::ruleReasoning::calcSuccessorState(*current_state, ground_world_rules, action, dummy_flag, s_suc, true);
          if (TL::areEqual(action_reward, TL::TL_DOUBLE_NIL)) {
            cout<<"World model's contexts are not fulfilled."<<endl;
//             HALT("This is a completely idiot action: rule engine has no unique rule for that.");
//             break;
            // do nothing
            s_suc = *old_state;
            action_reward = 0.; // no reward specified then
          }
          else {
            changes.append(a-1);
            cout<<"STATE CHANGE!!!"<<endl;
            cerr<<"STATE CHANGE!!!"<<endl;
          }
          current_state = new TL::State;
          *current_state = s_suc;
          rewards_round_action.append(action_reward);
          cout<<"Current state:  "<<*current_state<<endl;
          if (!TL::isZero(action_reward)) {cout<<"Got a reward!"<<endl;}
          cout<<"ACTION REWARD: "<<action_reward<<"    (All collected rewards thus far: "<<sum(rewards_round_action)<<")"<<endl;
          cerr<<"ACTION REWARD: "<<action_reward<<"    (All collected rewards thus far: "<<sum(rewards_round_action)<<")"<<endl;
        }
        trial.states.append(current_state);
//         PRINT(current_state);
        
        cout<<endl<<endl<<endl<<"+++++ STATE "<<a<<" +++++   overall-state "<<explorer->all_experiences.N+1<<" (round "<<i_round<<")"<<endl;
        cerr<<endl<<endl<<endl<<"+++++ STATE "<<a<<" +++++   overall-state "<<explorer->all_experiences.N+1<<" (round "<<i_round<<")"<<endl;
        current_state->write(cout, true); cout<<endl;
        current_state->write(cerr, true); cerr<<endl;
        cout<<"(Reward:  "<<reward.lits<<")"<<endl;
        cerr<<"(Reward:  "<<reward.lits<<")"<<endl;
        PRINT(changes);  PRINT2(changes, cerr);
        if (current_state->lits_prim.N == 0) {HALT("current_state has not been correctly calculated (is empty)");}
                
        if (a>0) {
          explorer->addObservation(old_state, action, current_state);
          cout<<"DEL: "<<explorer->all_experiences.last()->del<<endl;
          cout<<"ADD: "<<explorer->all_experiences.last()->add<<endl;
          cerr<<"DEL: "<<explorer->all_experiences.last()->del<<endl;
          cerr<<"ADD: "<<explorer->all_experiences.last()->add<<endl;
          uint experience_changes = explorer->all_experiences.last()->add.N + explorer->all_experiences.last()->del.N;
          cout<<"--> Experience changes = "<<experience_changes<<endl;
          if (experience_changes>5) cout<<"Difficult experience (many changes)"<<endl;
        }
        
        // ++++++++++ Learn new rules ++++++++++
        if (a>0) {
          cout<<"Executed actions with effects thus far (" << (changes.N *1.0/executed_actions.N) << "%):  "<<changes<<"  ";
          cerr<<"Executed actions with effects thus far (" << (changes.N *1.0/executed_actions.N) << "%):  "<<changes<<"  ";
          FOR1D(changes, k) {
            cout<<"  ";  cerr<<"  ";
            if (k%1 == 0) {cout<<"["<<changes(k)<<"] ";  cerr<<"["<<changes(k)<<"] ";}
            executed_actions(changes(k))->write(cout);  executed_actions(changes(k))->write(cerr);
          }
          cout<<endl;  cerr<<endl;
          
          
          TL::Rule* old_covering_rule = NULL;
          if (representation == TL::RuleExplorer::relational) {
            old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(explorer->getRules(), *current_state, action));
          }
          else if (representation == TL::RuleExplorer::factored  ||  representation == TL::RuleExplorer::flat) {
            old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(explorer->getRules(), *current_state, action));
          }
          else
            NIY;
          // If old covering rule is persistence rule, then don't relearn!
          if (old_covering_rule->outcomes.N == 2  &&  old_covering_rule->outcomes(0).N == 0  &&  (changes.N == 0   ||  (changes.N > 0  &&  changes.last() != a-1))) {
            cout<<"No learning as no change in experience and already enough experiences"<<endl;
            cout<<"Old covering rule:"<<endl;  old_covering_rule->write();
            explorer->updateRules(false);
          }
          else {
            t_start = MT::cpuTime();
            explorer->updateRules();
            t_finish = MT::cpuTime();
            cout<<"Rule learning took " << (t_finish - t_start) << "s"<<endl;
            cerr<<"Rule learning took " << (t_finish - t_start) << "s"<<endl;
            if (explorer->is_major_experience.last()) {
              cout <<"MAJOR EXPERIENCE!" << endl << endl;
              cerr <<"MAJOR EXPERIENCE!" << endl << endl;
            }
            else
              cout <<"Boring experience" << endl << endl;
  //           explorer->rulesC.write_experiencesWithRules();
            // Display learned rule for latest experience on shell [START]
            #if 0
            const TL::RuleSet& rules = explorer->getRules();
            uintA nonDefaultRules_for_last_experience;
            explorer->get_nonDefaultRules_for_last_experience(nonDefaultRules_for_last_experience);
            cout<<endl<<nonDefaultRules_for_last_experience.N<<" uptodate rule(s) explaining last experience " << *action << ":   "<<nonDefaultRules_for_last_experience<<endl;
            cerr<<endl<<nonDefaultRules_for_last_experience.N<<" uptodate rule(s) explaining last experience: "<< *action << ":   "<<nonDefaultRules_for_last_experience<< endl;
            FOR1D(nonDefaultRules_for_last_experience, i) {
              rules.elem(nonDefaultRules_for_last_experience(i))->writeNice();
              rules.elem(nonDefaultRules_for_last_experience(i))->writeNice(cerr);
            }
            if (nonDefaultRules_for_last_experience.N != 1) {
              cout << " ---> No unique covering rule and thus default rule has to be used."<<endl;
              cerr << " ---> No unique covering rule and thus default rule has to be used."<<endl;
            }
            #endif
            // Display learned rule for latest experience on shell [END]
          }
          
          // For which actions do we have manipulating rules?
          cout<<endl<<endl;  cerr<<endl<<endl;
          cout<<">>>>>>>>>>"<<endl<<"Actions with manipulating rules:"<<endl;
          cerr<<">>>>>>>>>>"<<endl<<"Actions with manipulating rules:"<<endl;
          AtomL actions_with_manipulating_rules, actions_with_manipulating_rules__confident;
          FOR1D_(explorer->getRules(), k) {
            uint o;
            FOR1D(explorer->getRules().elem(k)->outcomes, o) {
              if (explorer->getRules().elem(k)->outcomes(o).N > 0) {
                break;
              }
            }
            if (o < explorer->getRules().elem(k)->outcomes.N)  {
              if (actions_with_manipulating_rules.N > 0  &&  actions_with_manipulating_rules.last() == explorer->getRules().elem(k)->action) {
                cout << ", #"<<k;
                cerr << ", #"<<k;
              }
              else {
                cout<<endl<<*explorer->getRules().elem(k)->action<<"  ->  rule #"<<k;
                cerr<<endl<<*explorer->getRules().elem(k)->action<<"  ->  rule #"<<k;
                actions_with_manipulating_rules.setAppend(explorer->getRules().elem(k)->action);
              }
              cout<<"("<<explorer->rules__confidences(k)<<")";  cerr<<"("<<explorer->rules__confidences(k)<<")";
              if (explorer->rules__confidences(k) >= RULE_CONFIDENCE_THRESHOLD) {
                actions_with_manipulating_rules__confident.setAppend(explorer->getRules().elem(k)->action);
                cout<<" C ";  cerr<<" C ";
              }
              else {
                cout<<" IN ";  cerr<<" IN ";
              }
            }
          }
          cout<<endl<<endl;  cerr<<endl<<endl;
          if (actions_with_manipulating_rules__confident.N == explorer->modeledActions.N-1) {
            cout<<endl<<endl<<endl<<endl<<endl
                <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                <<endl<<"RULES FOR ALL MODELED ACTIONS at t="<<a<<" in round "<<i_round<<"!!!"<<endl;
          }
          if (representation == TL::RuleExplorer::relational  &&
                  explorer->modeledActions.N-1 > actions_with_manipulating_rules__confident.N) { // -1 wegen doNothing-Aktion
            // TODO gilt das auch fuer die anderen IPPC-Domaenen?
            MT_MSG("special heuristic for exploitation:  setting flag that there has been no major experience / makes no sense to exploit if not for all actions a manipulating rule");
            cerr<<"Setting  explorer->is_major_experience.last() = false;"<<endl;
            cout<<"Setting  explorer->is_major_experience.last() = false;"<<endl;
            explorer->is_major_experience.last() = false;  // als HACK macht keinen sinn, damit zu exploiten
          }
          cout<<endl<<endl;  cerr<<endl<<endl;
        }
        
        if (reward.satisfied(*current_state)) {
          cerr<<"ULTRAKORREKT!!!  Reward has been achieved :D."<<endl;
          cout<<"Yesyes! Reward has been achieved."<<endl;
          cerr<<"#used actions = "<<a<<endl;
          cout<<"#used actions = "<<a<<endl;
          break;
        }
        else if (!reward.possible(*current_state)) {
          failed = true;
          cerr<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<"OH MY GOD!!!  Reward no more possible."<<endl;
          cout<<"OH MY GOD!!!  Reward no more possible."<<endl;
          cout<<"Executed actions thus far:  ";  cerr<<"Executed actions thus far:  ";
          FOR1D(executed_actions, k) {
            cout<<"  ";  cerr<<"  ";
            if (k%10 == 0) {cout<<"["<<k<<"] ";  cerr<<"["<<k<<"] ";}
            executed_actions(k)->write(cout);  executed_actions(k)->write(cerr);
            switch (explorer->moves(explorer->moves.N - executed_actions.N + k)) {
              case MOVE_TYPE__EXPLOIT: cout<<"$";  cerr<<"$";  break;
              case MOVE_TYPE__EXPLORE_PLANNED: cout<<"!";  cerr<<"!";  break;
              case MOVE_TYPE__EXPLORE_DIRECT: cout<<"?";  cerr<<"?";  break;
              default: NIY;
            }
          }
          cout<<endl;  cerr<<endl;
          cout<<endl;  cerr<<endl;
          break;
        }
        
        if (a==max_actions) {
          cout<<endl<<endl<<endl<<endl<<endl<<"Run out of time"<<endl;
          cerr<<endl<<endl<<endl<<endl<<endl<<"Run out of time"<<endl;
          break;
        }
        
        if (a%1 == 0) {
  //         MT::String data_file_name;
  //         data_file_name << "experiences_"<<a<<".dat";
  //         ofstream data_file(data_file_name);
  //         trial.write(data_file);
  //         data_file.close();
        }
        
        
        cout<<endl<<endl<<"+++++ ACTION "<<a<<" +++++"<<endl;
        cerr<<endl<<endl<<"+++++ ACTION "<<a<<" +++++"<<endl;
        cout<<exploits.N<<" exploits"<<endl;
        cout<<explores_planned.N<<" planned explores"<<endl;
        cout<<explores_direct.N<<" direct explores"<<endl;
        cerr<<exploits.N<<" exploits"<<endl;
        cerr<<explores_planned.N<<" planned explores"<<endl;
        cerr<<explores_direct.N<<" direct explores"<<endl;
        
    
        TL::A_PRADA planner;
        if (representation == TL::RuleExplorer::relational) {
          planner.setNumberOfSamples(PRADA_num_samples + 100 * explorer->rules__confidences.N);
        }
        else if (representation == TL::RuleExplorer::factored  || representation == TL::RuleExplorer::flat) {
          planner.setNumberOfSamples(PRADA_num_samples + 10 * explorer->rules__confidences.N);
        }
        else
          NIY;
        planner.setNoiseSoftener(PRADA_noise_softener);
        planner.setDiscount(discountFactor);
        planner.setReward(&reward);
        planner.setHorizon(PRADA_horizon);
        // TODO HACK setting down the horizon to avoid incredible long planning times while being close to the goal
        if (TL::logicObjectManager::getPredicate(MT::String("at")) != NULL) {
          if (changes.N > 10  && a<PRADA_horizon/2)
            planner.setHorizon(TL_MIN(PRADA_horizon-a, 5));
        }
        PRINT(good_old_plan);
        planner.good_old_plans.clear();
        if (good_old_plan.N > 0)
          planner.good_old_plans.append(good_old_plan);
        PRINT(planner.good_old_plans);
        if (planner.good_old_plans.N > 0)
          cout<<"PRADA has good old plans."<<endl;

        // +++++++++ THE DECISION ++++++++++
        action = explorer->decideAction(*current_state, (TL::NID_Planner*) &planner, behavior_type, use_known_state_partial);
                                       
        executed_actions.append(action);
        action->write(actions_file);  actions_file << endl;
        switch (explorer->moves.last()) {
          case MOVE_TYPE__EXPLOIT: exploits.append(a);  break;
          case MOVE_TYPE__EXPLORE_PLANNED: explores_planned.append(a); break;
          case MOVE_TYPE__EXPLORE_DIRECT: explores_direct.append(a); break;
          default: NIY;
        }
        
        // remember good old plan
        if (explorer->moves.last() == MOVE_TYPE__EXPLOIT) {
          good_old_plan.clear();
          good_old_plan = explorer->last_exploit_plan;
          PRINT(good_old_plan);
        }

//         explorer->message << "     (" << a << ")";
        
        cout<<endl<<"=====  Decided Action #" << a << ": "<<*action;
        cerr<<endl<<"=====  Decided Action #" << a << ": "<<*action;
        switch (explorer->moves.last()) {
            case MOVE_TYPE__EXPLOIT: cout<<"  exploit";  cerr<<"  exploit";  break;
            case MOVE_TYPE__EXPLORE_PLANNED: cout<<"  planned explore";  cerr<<"  planned explore";  break;
            case MOVE_TYPE__EXPLORE_DIRECT: cout<<"  direct explore";  cerr<<"  direct explore";  break;
            default: NIY;
        }
        cout<<"  ====="<<endl;  cerr<<"  ====="<<endl;
        
        cout<<"Executed actions thus far:  ";
        cerr<<"Executed actions thus far:  ";
        FOR1D(executed_actions, k) {
          cout<<"  ";  cerr<<"  ";
          if (k%10 == 0) {cout<<"["<<k<<"] ";  cerr<<"["<<k<<"] ";}
          executed_actions(k)->write(cout);  executed_actions(k)->write(cerr);
          switch (explorer->moves(explorer->moves.N - executed_actions.N + k)) {
            case MOVE_TYPE__EXPLOIT: cout<<"$";  cerr<<"$";  break;
            case MOVE_TYPE__EXPLORE_PLANNED: cout<<"!";  cerr<<"!";  break;
            case MOVE_TYPE__EXPLORE_DIRECT: cout<<"?";  cerr<<"?";  break;
            default: NIY;
          }
        }
        cout<<endl;  cerr<<endl;
        cout<<endl;  cerr<<endl;

        // Display old rule that predicts action [START]
        cout << "Current rule for prediction:"<<endl;
        TL::Rule* old_covering_rule = NULL;
        if (representation == TL::RuleExplorer::relational) {
          old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(explorer->getRules(), *current_state, action));
        }
        else if (representation == TL::RuleExplorer::factored  ||  representation == TL::RuleExplorer::flat) {
          old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(explorer->getRules(), *current_state, action));
        }
        else
          NIY;
        if (old_covering_rule == NULL  ||  TL::ruleReasoning::isDefaultRule(old_covering_rule)) {
          cout << " -- "<<endl;
        }
        else {
          old_covering_rule->write(cout, false);
//           explorer->rulesC.rules.writeNice();
        }
        // Display old rule that predicts action [END]
        
        
        cerr<<"Perform action... ";
                
        trial.actions.append(action);
      }
      // -------------------------------------
      //   ROUND-RUN  [end]
      // -------------------------------------
      
      num_actions.append(a);
      
      if (failed)
        num_successes.append(2);
      else if (num_actions.last() == max_actions)
        num_successes.append(0);
      else // SUCCESS
        num_successes.append(1);
      
      
      double state_reward = 0.0;
      if (num_successes.last() == 1) {
        cerr<<"--> STATE REWARD "<<state_reward<<endl;
        cout<<"--> STATE REWARD "<<state_reward<<endl;
      }
      rewards_trial_states(i_round) = state_reward;
      rewards_trial_actions(i_round) = sum(rewards_round_action);
      cerr<<"Des gibt e Belohnung:  "<<state_reward<<" + "<<sum(rewards_round_action)<<" = "<<(state_reward+sum(rewards_round_action)) << endl;
      cout<<"Des gibt e Belohnung:  "<<state_reward<<" + "<<sum(rewards_round_action)<<" = "<<(state_reward+sum(rewards_round_action)) << endl;
      
      cout<<"Executed action types:  ";   cerr<<"Executed action types:  ";
      for (k=explorer->moves.N-num_actions.last(); k<explorer->moves.N; k++) {
        cout<<"  ";  cerr<<"  ";
        switch (explorer->moves(k)) {
          case MOVE_TYPE__EXPLOIT: cout<<"$";  cerr<<"$";  break;
          case MOVE_TYPE__EXPLORE_PLANNED: cout<<"!";  cerr<<"!";  break;
          case MOVE_TYPE__EXPLORE_DIRECT: cout<<"?";  cerr<<"?";  break;
          default: NIY;
        }
      }

      
      cout<<endl;  cerr<<endl;
      
      if (exploits.findValue(num_actions.last()-1) >= 0) {
        cerr<<"Reward has been achieved with exploit."<<endl;
        cout<<"Reward has been achieved with exploit."<<endl;
      }
      
      fprintf(f_results, "%2i %i %i %2i %2i %2i %2i %2i     %2i   %2i     %.1f %.1f %.1f\n",
                i_trial, i_round, num_successes.last(), num_actions.last(), explorer->getRules().num(),
                explores_direct.N, explores_planned.N, exploits.N, exploits.findValue(num_actions.last()-1) >= 0, TL::logicObjectManager::constants.N,
              (rewards_trial_states(i_round) + rewards_trial_actions(i_round)), rewards_trial_states(i_round), rewards_trial_actions(i_round));
      fflush(f_results);
//       delete reward;
    }
    // -------------------------------------
    //   ROUND  [end]
    // -------------------------------------
//     }
//     catch(...) {
// //       cerr<<"TRIAL failed: error / exception caught for some reason."<<endl;
//       cout<<"TRIAL failed: error / exception caught for some reason."<<endl;
//     }
    
    // Epilogue of trial
    
    FOR1D(trial.states, k) {
      delete trial.states(k);
    }
//     cout<<"States deleted"<<endl;
    
    delete explorer;
//     cout<<"Explorer deleted"<<endl;
  }
  
  fclose(f_results);
}














int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  cout.precision(3);
  cerr.precision(3);
  MT::String config_file("config");
//   MT::String config_file("config_exp4");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
// 	showMovie("film_aktionen.dat");
//   showMovie("film_aktionen_ICML_exp3.dat");
//   showMovie("film_aktionen_ecml10papier.dat");
// 	exit(0);

  experiment_exploration();
  
	return 0;
}



