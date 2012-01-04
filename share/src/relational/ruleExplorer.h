#ifndef TL__RULE_EXPLORER
#define TL__RULE_EXPLORER

#include <relational/robotManipulationDomain.h>
#include <relational/ruleLearner.h>
#include <relational/ruleLearner_ground.h>
#include <relational/plan.h>
#include <relational/prada.h>


// ---------------------------------------------------
//       FLAGS

#define MOVE_TYPE__EXPLOIT 1
#define MOVE_TYPE__EXPLORE_PLANNED 2
#define MOVE_TYPE__EXPLORE_DIRECT 3



// ---------------------------------------------------
//       EXPLORATION - PARAMETERS

// General
#define RULE_CONFIDENCE_THRESHOLD 2.0
#define MIN_PATTERN_ACTION_REPEATS 2
#define BAD_PATTERN_MIN_LENGTH 2
#define ENTROPY_RELATED_OBJECTS_DEPTH 1   // 1 am besten bei box-clearance

// Planned explore
#define PLANNED_EXPLORE__NUMBER_PRADA_SAMPLES 300
#define PLANNED_EXPLORE__HORIZON 3
#define PLANNED_EXPLORE__THRESHOLD 0.2





extern RobotManipulationSimulator sim;

namespace TL {
  
  
struct RelationalStateGraph;
  


  
  
class RuleExplorer {
  public:
    
  enum BehaviorType {epsilon_greedy, ecube, rmax};
  enum DensityEstimationType {density_simple, density_entropy};
  enum RepresentationType {relational, factored, flat};
  
  
  // Greedy
  double greedy_epsilon_start;
  
    
  RepresentationType representation;
  
  // Rules
  arr rules__confidences; // offen fuer bearbeitung
  TL::RuleSet confident_ground_rules;
  
  // Rule learning
  double complexity_penalty_coeff;
  double p_lower_bound__noise_outcome;
  double p_lower_bound__noise_outcome_in_default_rule;
  
  // Actions
  AtomL modeledActions;
  AtomL fixedActions;
  boolA learners_uptodate;
  uintA actions__covering_rules;
  MT::Array< uintA > actions__covering_rules__all;
  arr actions__confidences;
  arr actions__dissimilarity_to_previous_experiences__local;  // TODO NEU berechnet Konfidenzen entsprechend der Zustaende in den Erfahrungen (ohne auf Regeln zu schauen)
  boolA action__has_interesting_argument;
  boolA action__is_known;
  
  AtomL possibleGroundActions;
  
  // State measures
  TL::State current_state;
  bool current_state_is_known;
  bool current_state_is_known_partially;
  
  // Experiences
  MT::Array< uintA > experiences_per_modeledAction;
  SymbolicExperienceL all_experiences;
  arr experience_weights;
  boolA is_major_experience;
  MT::Array<TL::State*> visited_pre_states;
  AtomL visited_actions;
  
  // Relevant objects
  uintA reward_constants;
  uintA reward_constants_neighbors;
  
  // Planned exploration
  TL::Reward* planned_explore_reward;
  uint planned_explore_reward__timestamp;
  LitL very_old_exploration_rewards;
  LitL very_very_old_exploration_rewards;
  
  // only for statistics
  uintA moves_explore_direct;
  uintA moves_explore_planned;
  uintA moves_exploit;
  uintA moves;
  
  // auxiliary structures
  AtomL last_exploit_plan;
  
  // only for presentation
  MT::String message;
  
  virtual uint action_to_learner_id(TL::Atom* action) = 0;
  
  
  
  // --------------------------------------------------------------------------------------------
  
  explicit RuleExplorer(RepresentationType representation,
                        double complexity_penalty_coeff, double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule);
  ~RuleExplorer();
    
  // Confidence calculations (--> true state novelty calculation / job is done here)
  double calcRuleConfidence(uint rule_id);
  void calcExploreWeights(arr& explore_weights, TL::Atom* taboo_action = NULL);
  
  // Propagate confidence calculations
  virtual void updateActionAndStateMeasures();
  
  // Access state informaction
  void getKnownStates(StateL& known_states); // aus rules, all_ground_actions und visited_states zusammengesetzt
  bool stateIsKnown(const TL::State& state, const AtomL& ground_actions);
  bool stateIsKnown_partially(const TL::State& state, const AtomL& ground_actions);
  virtual bool actionIsKnown(const TL::State& state, TL::Atom* action);
  virtual uint getNumberOfCoveredExperiences(uint rule_id) = 0;
  
  // Decision-making
  TL::Atom* getExploitAction(AtomL& exploit_plan, TL::PRADA* prada, const TL::State& state);
  TL::Atom* getExploreAction_planned__contexts(AtomL& explore_plan, TL::PRADA* prada, const TL::State& state);
  TL::Atom* getExploreAction_planned(AtomL& explore_plan, TL::PRADA* prada, const TL::State& state, uint type = 1);
  TL::Atom* getExploreAction_direct(TL::PRADA* prada, const TL::State& state, TL::Atom* taboo_action = NULL);
  
  // Methods for EXTERNAL use
  virtual TL::Atom* decideAction(const TL::State& state, TL::NID_Planner* planner, uint behavior_type, bool use_known_state_partial);
  void addObservation__helper(TL::State* state_pre, TL::Atom* action, TL::State* state_post);
  virtual void addObservation(TL::State* state_pre, TL::Atom* action, TL::State* state_post) = 0;
  void addObservations(const TL::Trial& trial);
  
  virtual void updateRules(bool always_re_learning = true) = 0;
  virtual void updateLogicEngineConstants();  // das hier vielleicht doch nicht, oder?
  virtual void reset();
  virtual const TL::RuleSet& getRules() = 0;
  virtual void get_nonDefaultRules_for_last_experience(uintA& rule_ids) = 0;
};
  




class AbstractRuleExplorer : public RuleExplorer {
public:
  TL::RuleSetContainer rulesC;
  MT::Array<RuleLearner*> learners;
  
  arr rule_experiences_entropies;
  
  // For more sophisticated density estimators
  RelationalStateGraph* current_graph;
  MT::Array< RelationalStateGraph* > graphs;
  
  uint density_estimation_type;
  
  public:
    AbstractRuleExplorer(double complexity_penalty_coeff, double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                          TL::RuleSet& fixed_rules_for_fixed_actions, uint density_estimation_type);
    ~AbstractRuleExplorer();
    virtual void init_rules();
    uint action_to_learner_id(TL::Atom* action);
    void updateRules(bool always_re_learning = true);
    
    void set_p_lower_bounds(double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule);
    
    // Gehoert eigentlich in die Superklasse:  Problem rulesC muesste vereinheitlicht werden...
    const TL::RuleSet& getRules() {return rulesC.rules;}
    void get_nonDefaultRules_for_last_experience(uintA& rule_ids) {rule_ids = rulesC.nonDefaultRules_per_experience.last();}
    uint getNumberOfCoveredExperiences(uint rule_id) {return rulesC.experiences_per_rule(rule_id).N;}
    void addObservation(TL::State* state_pre, TL::Atom* action, TL::State* state_post);
    void setFixedRulesForAction(TL::RuleSet& rules_for_action);
    double calcRuleConfidence(uint rule_id);
    void updateActionAndStateMeasures();
};


class FactoredRuleExplorer : public RuleExplorer {
  TL::RuleSetContainer_ground rulesC;
  MT::Array<RuleLearner_ground*> learners;
  
  public:
    FactoredRuleExplorer(double complexity_penalty_coeff, double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                         TL::RuleSet& fixed_rules_for_fixed_actions);
    ~FactoredRuleExplorer();
    virtual void init_rules();
    uint action_to_learner_id(TL::Atom* action);
    void updateRules(bool always_re_learning = true);
    void updateLogicEngineConstants();
    void reset();
    
    // Gehoert eigentlich in die Superklasse:  Problem rulesC muesste vereinheitlicht werden...Z
    const TL::RuleSet& getRules() {return rulesC.rules;}
    void get_nonDefaultRules_for_last_experience(uintA& rule_ids) {rule_ids = rulesC.nonDefaultRules_per_experience.last();}
    uint getNumberOfCoveredExperiences(uint rule_id) {return rulesC.experiences_per_rule(rule_id).N;}
    void addObservation(TL::State* state_pre, TL::Atom* action, TL::State* state_post);
};



class FlatExplorer : public RuleExplorer {
  MT::Array< TL::RuleSet > rules_hierarchy;  // dim 0: actions;  dim 1: states
  MT::Array< MT::Array< uintA > > experiences_per_rule__hierarchy;
  MT::Array< uintA > rule_confidences__hierarchy;
  // --> default rule:  hier ganz hinten
  
  // redundant rule-container
  TL::RuleSet rules_flat;
  MT::Array< uintA > experiences_per_rule__flat;
  // --> default rule:  hier ganz vorne
  
  TL::RuleSet fixed_rules_memory;
  
  public:
    FlatExplorer(TL::RuleSet& fixed_rules_for_fixed_actions);
    ~FlatExplorer();
    void init_rules();
    uint action_to_learner_id(TL::Atom* action);
    int action_state_to_flat_id(TL::Atom* action, TL::State* state);
    void updateRules(bool always_re_learning = true);
    void updateLogicEngineConstants();
    void reset();
    void addObservation(TL::State* state_pre, TL::Atom* action, TL::State* state_post);
    
    // Gehoert eigentlich in die Superklasse:  Problem rulesC muesste vereinheitlicht werden...Z
    const TL::RuleSet& getRules() {return rules_flat;}
    void get_nonDefaultRules_for_last_experience(uintA& rule_ids);
    uint getNumberOfCoveredExperiences(uint rule_id) {return experiences_per_rule__flat(rule_id).N;}
};




#if 1
struct RelationalStateGraph {
  
  const TL::State state;
  uintA constants;
  
  LitL lits_zeroary;
  MT::Array< LitL > lits_unary;
  MT::Array< LitL > lits_binary; // 2-dim
  boolA lits_binary_matrix;  // adjacency_matrix(a,b)  iff  a-->b  iff predicate(a,b)
  FuncVL fvs_zeroary;
  MT::Array< FuncVL > fvs_unary;
  
  RelationalStateGraph(const TL::State& state);
  ~RelationalStateGraph();
  
  void getRelatedConstants(uintA& related_constants, uint obj, uint depth) const;
  void getRelatedConstants(uintA& related_constants, const uintA& objects, uint depth) const;
  void getDirectNeighbors(uintA& neighbors, uint obj) const;
  
  RelationalStateGraph* getSubgraph(const uintA& objects) const;
  
  void writeNice(ostream& os = cout) const;
  
  
  
  // distance in [0,1]
  static double distance(const RelationalStateGraph& g1, const TL::Atom& a1,
                         const RelationalStateGraph& g2, const TL::Atom& a2);
  
  static double entropy(const AtomL& actions, const MT::Array< RelationalStateGraph* >& graphs);
  
//   RelationalStateGraph* createGraph(const TL::Atom& action, const TL::State& state, const TL::Rule& rule, uint depth);
  static RelationalStateGraph* createSubgraph(const TL::Atom& action, const TL::RelationalStateGraph& full_graph, const TL::Rule& rule, uint depth);
  
  static double getMinDistance(const RelationalStateGraph& graph, const TL::Atom& action,
                               const MT::Array< RelationalStateGraph* > other_graphs, const AtomL& other_actions);
  
};
#endif


}

#endif // TL__RULE_EXPLORER
