#ifndef TL__RULE_LEARNER
#define TL__RULE_LEARNER

#include <relational/logicObjectManager.h>


/*

HEURISTIC PARAMETERS / DECISIONS in the learning algorithm:

- Alpha:  regularization penalty
- Noise outcome probabilities:  p_min, p_min_noisyDefaultRule
- Weights of search operators (--> how often each operator is tried)
- Consideration horizon of previous successful search operator applications:
    #define SEARCH_OP_CHOICE__PAST_HORIZON 20
    #define SEARCH_OP_CHOICE__PAST_WEIGHT 0.5
- Penalties for parameter estimation (probabilities of outcomes) ("learnParameters")
    -- pen_sum: sum to 1
    -- pen_pos: probs non-negative


- Heuristics for setting alpha:

  --  For non-noise outcomes: for alpha<0.1 it pays off to use 2 rules with p=1.0 each instead of
      1 rule with two outcomes with p=0.5 each (i.e., not to merge several experiences).
      (assumption: the complexity of rules is not more than about 10 literals)


- Noise-outcomes:

  -- For strange experiences, which would require a separate rule with very, very many literals,
     it does *not* pay off to include this separate rule. Rather, this should be modeled by the noise
     outcome of a different rule (potentially the default rule).

  -- Given alpha=1.0 and p_min < 10e-5, it pays off to include a separate rule for 10 rule literals.
     --> Thus, it almost always pays off to include a separate rule for "normal" 
         experiences (i.e., which require a normal-sized rule)
*/



#define DEFAULT_LOGFILE "ruleLearner.log"


// Search operator weights determine how often each operator is tried.
// (The larger the weight, the more often.)
#define SO_WEIGHT__EXPLAIN_EXPERIENCES 4.0
#define SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM 4.0
// incl. dynamic bound cpts
#define SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM_AND_COMPARING 0.0  // 4.0

#define SO_WEIGHT__DROP_CONTEXT_LITERALS 0.0
#define SO_WEIGHT__DROP_CONTEXT_LITERALS_APPROX 100.0  // 48.0 vorher
#define APPROXIMATOR__RULES_PER_ROUND 5
#define SO_WEIGHT__DROP_REFS 2.0

#define SO_WEIGHT__DROP_RULES 3.0

#define SO_WEIGHT__SPLIT_ON_LITS 3.0
#define SO_WEIGHT__ADD_LITS 2.0
#define SO_WEIGHT__ADD_REFS 2.0

// constant bounds
#define SO_WEIGHT__SPLIT_ON_EQS 2.0
#define SO_WEIGHT__CHANGE_RANGE 3.0
#define SO_WEIGHT__MAKE_INTVL 3.0
// dynamic bounds
#define SO_WEIGHT__COMPARE_FUNCTIONVALUES 0.0
#define SO_WEIGHT__SPLIT_ON_COMPARE_FUNCTIONVALUES 0.0
// both bounds
#define SO_WEIGHT__GENERALIZE_EQS 5.0


#define SEARCH_OP_CHOICE__PAST_HORIZON 20
#define SEARCH_OP_CHOICE__PAST_WEIGHT 0.5


#define RULE_LEARNER__OP_CHOICE__LINEAR 1
#define RULE_LEARNER__OP_CHOICE__RANDOM 2





namespace TL {
  
// Efficiency wrapper for rule-sets
// --> stores the coverage of experiences
class RuleSetContainer {
  private:
    
  public:
  // First rule = default rule
  TL::RuleSet rules;
  const SymbolicExperienceL* p_experiences;
  
  // redundant memories
  MT::Array< uintA > nonDefaultRules_per_experience;  // only non-default rules!
  MT::Array< uintA > experiences_per_rule;
  
  MT::Array< MT::Array < uintA > > experiences_per_ruleOutcome;
  
  RuleSetContainer(const SymbolicExperienceL* _p_experiences);
  RuleSetContainer(); // use init() later when using this constructor
  
  void init(const SymbolicExperienceL* _p_experiences);
  void append(TL::Rule* rule, uintA& experiences_of_this_rule, MT::Array< uintA >& experiences_per_outcome_of_this_rule);
  void remove(uint id);
  void clear();
  void recomputeDefaultRule();
  void sort();
  
  void getResponsibilities(arr& responsibilities, MT::Array< uintA >& covered_experiences, uintA& covered_experiences_num) const;
  void getPartitionsForAction(MT::Array< uintA >& partititions, TL::Atom* action) const;
  
  void writeNice(ostream& out = std::cout, bool only_action = false) const;
  void write_experiencesWithRules(ostream& os = std::cout) const;
  void write_rulesWithExperiences(ostream& os = std::cout) const;
  
  void sanityCheck(bool ignore_default_rule = false) const;  // EXPENSIVE!!!
};





namespace CostFunction {
  void setNoiseStateProbability(double p_min);
  void setRuleCoveredExperiences(const SymbolicExperienceL& coveredEx);
  void setOutcomesCoverage(const boolA& coverage);
  void setPenaltySum(double pen_sum);
  void setPenaltyPos(double pen_pos);
  double loglikelihood(const arr& probs);
  
  // C = -LOG_LIK + PENALTIES
  // --> to minimize
  // need knowledge of covered experiences and corresponding covered outcomes
  double calc(const arr& in);	
  void calc_grad(arr& out, const arr& in);
};



void calcCoverage(SymbolicExperienceL& covered_experiences, uintA& covered_experiences_ids, const TL::Rule* r, const SymbolicExperienceL& experiences);




class SearchOperator {
  
public:
  enum ProbabilityOptimizationType {grad_desc, rprop}; // Others? newton, conjgrad, cond_grad
  
  
protected:

  ProbabilityOptimizationType param_opt_type;
  double pen_sum;
  double pen_pos;

  double alpha_PEN;
  double p_min;
  MT::String name;
  bool approximative;
        
  // takes the rulelist rules2add and integrates it into existing ruleset
  void integrateNewRules(const TL::RuleSetContainer& rulesC_old, const TL::RuleSetContainer& rules_2add,
                          const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_new);
  
  // Creates possible new rules for the given rule-set.
  // "rules_2add" are potential additional rules which are all supposed to become part of the SAME rule-set!
  // I.e., they will be integrated into the old rule-set.
  virtual void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add) = 0;
        
public:
  SearchOperator(double alpha_PEN, double p_min, ProbabilityOptimizationType param_opt_type);
  virtual ~SearchOperator() {}

  void setProbabilitiesLearningPenalty_sum(double pen_sum);
  void setProbabilitiesLearningPenalty_pos(double pen_pos);
  void set_p_min(double p_min);
  
  // resets search operator for next application round
  virtual void reset() = 0;

  // central method which is called by the RuleLearner
  virtual void createRuleSets(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, 
                  MT::Array< TL::RuleSetContainer >& sets_of_new_rules);
  
  const char* getName();
  bool isApproximator() {return approximative;}
  virtual void reset_total_approximator() {}
    
    
public:
  // Static Methods for estimation of outcomes and probabilities
  // Outcomes
  static void calcCoverage_outcomes(const MT::Array< LitL >& outcomes, const SymbolicExperienceL& experiences, const TL::Rule* rule, boolA& coverage);
  static void calcSubsumption(boolA& subsumes, const boolA& coverage);
  // remove outcomes that (i) do not cover any example and (ii) have zero-probability  and (iii) sets coverage for cost function
  static void produceTrimmedOutcomes(MT::Array< LitL >& outcomes, arr& probs, boolA& coverage, const SymbolicExperienceL& coveredExperiences,
                                      const TL::Rule& rule, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type);
              
  // Parameter learning
  static double learnParameters_constrainedCostfunction(const MT::Array< LitL >& outcomes, doubleA& probs, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type);
  static double learnParameters(const MT::Array< LitL >& outcomes, doubleA& probs, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type);
  
  static void induceOutcomes(TL::Rule* rule, MT::Array< uintA >& coveredExperiences_per_outcome, const SymbolicExperienceL& covered_experiences, const uintA& covered_experiences_ids,
                              double alpha_PEN, double p_min, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type);
};





class ExplainExperiences : public SearchOperator {
  uint nextPotentialExperience;
  bool slimContext;
  bool comparingValues;
  public:
    ExplainExperiences(bool slimContext, bool comparingValues, double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      if (!slimContext) {
        if (!comparingValues)
          name = "ExplainExperiences";
        else
          name = "ExplainExperiences_comparingValues";
      }
      else {
        if (!comparingValues)
          name = "ExplainExperiences_slim";
        else
          name = "ExplainExperiences_slim_comparingValues";
      }
			nextPotentialExperience = 0;
      this->slimContext = slimContext;
      this->comparingValues = comparingValues;
    }

    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    TL::Rule* explainExperience(TL::SymbolicExperience* ex);
    TL::Rule* explainExperience_straightforward(SymbolicExperience* ex);
    TL::Rule* explainExperience_deictic(SymbolicExperience* ex);
    TL::Rule* explainExperience_deictic_ALL_DRs(SymbolicExperience* ex);
    
    // resets field "nextPotentialExperience
    void reset();
};


class DropContextLiterals : public SearchOperator {
  uint nextRule;
  uint nextContextLiteral;
  public:
    DropContextLiterals(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
			name = "DropContextLiterals";
			nextRule = 0;
			nextContextLiteral = 0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    
    void reset();
};




class DropContextLiterals_approximativeVersion : public SearchOperator {
  uintA usableContextLiterals;
  const static uint DROP_NEGATIVE_BIAS = 3;
  bool prepareTotalNewSearch;
//     uint producedRules;
  public:
    DropContextLiterals_approximativeVersion(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
        name = "DropContextLiterals_approximativeVersion";
        approximative = true;
        prepareTotalNewSearch = false;
            }
            void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
            
            void reset();
    void reset_total_approximator();
};




class DropReferences : public SearchOperator {
  uint nextRule;
  uint nextReference;
  public:
    DropReferences(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "DropReferences";
      nextReference = 0;
      nextRule=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    
    void reset();
};


class DropRules : public SearchOperator {
  public:
    DropRules(double alpha, double p_min, ProbabilityOptimizationType param_opt_type, double pen_sum = 100.0, double pen_pos = 1.0) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "DropRules";
    }
    
    void createRuleSets(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, 
              MT::Array< TL::RuleSetContainer >& sets_of_new_rules);
    
    // the following two rules are empty
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);   
    void reset();
};



// considers only primitives and complex (not equality literals)
class SplitOnLiterals : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  LitL absentLiterals;
  public:
        SplitOnLiterals(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "SplitOnLiterals";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    
    void reset();
};



class AddLiterals : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  LitL absentLiterals;
  public:
        AddLiterals(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "AddLiterals";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    
    void reset();
};


class AddReferences : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  LitL restrictionLiterals;
  public:
    AddReferences(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "AddReferences";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
    
    void reset();
};



// SOs on ComparisonPredicates

class GeneralizeEquality : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  bool doneLess;

  public:
    GeneralizeEquality(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
        name = "GeneralizeEquality";
        nextRule=1; // ignore default rule
        nextLiteral=0;
        doneLess=false;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);

    void reset();
};



// for each variable v for each function f for which a comparison predicate with v is not
// used yet, we introduce equality literals for all existing values for v.
class SplitOnEqualities : public SearchOperator {
  uint nextRule;
  uint nextVar;
  uint nextFunc;
  uintA vars;
  FuncL usedFunctions;
  
  public:
      SplitOnEqualities(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
          name = "SplitOnEqualities";
          nextRule=1; // ignore default rule
          nextVar=0;
          nextFunc=0;
          usedFunctions.append(logicObjectManager::f_prim);
          // Achtung DON'T used derived: usedFunctions.append(logicObjectManager::f_derived);!!!!
      }
      void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
              
      void reset();
};



// changes ranges of all comp preds
class ChangeRange : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  uint nextPossibleValue;
  arr possibleValues;

  public:
    ChangeRange(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
        name = "ChangeRange";
        nextRule=1; // ignore default rule
        nextLiteral=0;
        nextPossibleValue=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
            
    void reset();
};


// for all less/greater and less/greater-equal compPreds, we introduce
// less/greater-equal compPreds for all possible values
// only defined for constant comparisons!
class MakeInterval : public SearchOperator {
  uint nextRule;
  uint nextLiteral;
  uint nextPossibleValue;
  arr possibleValues;

  public:
    MakeInterval(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
        name = "MakeInterval";
        nextRule=1; // ignore default rule
        nextLiteral=0;
        nextPossibleValue=0;
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
            
    void reset();
};



// Compares values of the same function, applied to all possible term combinations.
// (1) for all functions (2) for all term combinations (3) for all comparison types
// example: (1) size(.),  (2) size(x1)Xsize(x2), size(x1)Xsize(x3)...,
//   (3) size(x1)<size(x2), size(x1)<=size(x2), size(x1)==size(x2)...
// [CPT]: ComparisonPredicates dynamic
// 
// Be aware of the following subtle behavior: cover(CPT with ==) and cover(CPT with <)
// does not necessarily imply cover(CPT with <=). Explanation: Rule with (CPT==) may cover with sub1
// and rule with (CPT<) may cover with sub2 (=/= sub1), so that rule with (CPT<=) could cover with
// sub1 and sub2. But since we are using deictic referencing, the rule with (CPT<=) 
// does not apply in case in that case since we have more than one possible substitution for the rule.
class CompareFunctionValues : public SearchOperator {
  uint nextRule;
  uint nextFunction;
  uint nextComparisonType;
  uint nextTermCombination;
  
  MT::Array< ComparisonType > comparisonTypes;
  MT::Array<uintA> termCombos;
  FuncL usedFunctions;
  
  MT::Array<uintA> coveredExIDsPerComparisonType; // only for debugging

  public:
    CompareFunctionValues(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
        name = "CompareFunctionValues";
        nextRule=1; // ignore default rule
        nextFunction=0;
        nextComparisonType=0;
        nextTermCombination=0;
        usedFunctions.append(logicObjectManager::f_prim);
        usedFunctions.append(logicObjectManager::f_derived);
        // collect comparison types SAVE THE ORDER!
        comparisonTypes.append(comparison_equal);comparisonTypes.append(comparison_less);comparisonTypes.append(comparison_lessEqual);
        comparisonTypes.append(comparison_greater);comparisonTypes.append(comparison_greaterEqual);
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);
            
    void reset();
};




// Introduces a comparison between two functions values and defines 3 corresponding rules,
// namely with <,==,>, that are introduced to a single new rule-set.
//
// (1) for all functions (2) for all term combinations
// example: (1) size(.),  (2) size(x1)Xsize(x2), size(x1)Xsize(x3)...,
// [CPT]: ComparisonPredicates dynamic
// 
// Be aware of the following subtle behavior: cover(CPT with ==) and cover(CPT with <)
// does not necessarily imply cover(CPT with <=). Explanation: Rule with (CPT==) may cover with sub1
// and rule with (CPT<) may cover with sub2 (=/= sub1), so that rule with (CPT<=) could cover with
// sub1 and sub2. But since we are using deictic referencing, the rule with (CPT<=) 
// does not apply in case in that case since we have more than one possible substitution for the rule.
class SplitOnCompareFunctionValues : public SearchOperator {
  uint nextRule;
  uint nextFunction;
  uint nextTermCombination;
  MT::Array< ComparisonType > comparisonTypes;
  MT::Array<uintA> termCombos;
  FuncL usedFunctions;
  
  MT::Array<uintA> coveredExIDsPerComparisonType; // only for debugging

  public:
    SplitOnCompareFunctionValues(double alpha, double p_min, ProbabilityOptimizationType param_opt_type) : SearchOperator(alpha, p_min, param_opt_type) {
      name = "SplitOnCompareFunctionValues";
      nextRule=1; // ignore default rule
      nextFunction=0;
      nextTermCombination=0;
      usedFunctions.append(logicObjectManager::f_prim);
      usedFunctions.append(logicObjectManager::f_derived);
      // collect comparison types SAFE THE ORDER!
      comparisonTypes.append(comparison_equal);comparisonTypes.append(comparison_less);comparisonTypes.append(comparison_greater);
    }
    void findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rules_2add);

    void reset();
};














class RuleLearner {

protected:
  MT::Array< SearchOperator* > searchOperators;
  double alpha_PEN;
  double p_min;
  double p_min_noisyDefaultRule;

public: // TODO entfernen
    
  // statistics
  uintA num_so_improvements;
  uintA num_so_applied;
  arr so_improvements;
  uintA so_successfulUsageHistory;
  uintA so_UsageHistory;
  arr scores;
    
  
  // choosing the next operator
  int chooseNextOperator(boolA& op_applicable);
  uint so_choice_type;
  int so_current;
  arr so_priorWeights;
      
  public:
    RuleLearner(double alpha_PEN, double p_min, double p_min_noisyDefaultRule, uint so_choice_type = 2);
    ~RuleLearner();
  
    void setAlphaPEN(double alpha_PEN);
    void set_p_mins(double p_min, double p_min_noisyDefaultRule);
    
    double score(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, double cutting_threshold);
    double score(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, double cutting_threshold, arr& experience_weights);
    virtual void learn_rules(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, const char* logfile = DEFAULT_LOGFILE); 
    virtual void learn_rules(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, arr& experience_weights, const char* logfile = DEFAULT_LOGFILE); 
};

}

#endif // TL__RULE_LEARNER
