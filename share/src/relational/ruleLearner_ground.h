#ifndef TL__RULE_LEARNER_GROUND
#define TL__RULE_LEARNER_GROUND

/*

HEURISTISCHE WERTE:

- Gewichte der Suchoperatoren
- Beruecksichtigung vergangener erfolgreicher Suchoperatorenverwendungen:
    #define SEARCH_OP_CHOICE__PAST_HORIZON 20
    #define SEARCH_OP_CHOICE__PAST_WEIGHT 0.5
- Penalties for parameter estimation (probabilities of outcomes) ("learnParameters")
    -- pen_sum: sum to 1
    -- pen_pos: probs non-negative
- Alpha: regularization penalty
- Noise outcome probabilities: p_min, p_min_noisyDefaultRule

*/




#include <relational/ruleLearner.h>

// ANDERSDA ++++++++++++++
#define SO_WEIGHT__EXPLAIN_EXPERIENCE 4.0
#define SO_WEIGHT__EXPLAIN_EXPERIENCE_SLIM 4.0
// incl. dynamic bound cpts
#define SO_WEIGHT__EXPLAIN_EXPERIENCE_SLIM_AND_COMPARING 0.0  // 4.0

#define SO_WEIGHT__DROP_PRECONDS 0.0
#define SO_WEIGHT__DROP_PRECONDS_APPROX 48.0
// #define SO_WEIGHT__DROP_PRECONDS_APPROX 18.0 wert bei allem anderen scheiss
#define APPROXIMATOR__RULES_PER_ROUND 5

// ANDERSDA ++++++++++++++
#define SO_WEIGHT__DROP_REFS 2.0

#define SO_WEIGHT__DROP_RULES 3.0

#define SO_WEIGHT__SPLIT_ON_LITS 3.0
#define SO_WEIGHT__ADD_LITS 2.0



// ANDERSDA ++++++++++++++ sowie alle anderen folgenden
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
class RuleSetContainer_ground {
  private:
    
  public:
  
  TL::RuleSet rules;
  const ExperienceA* p_examples;  // TODO den hier noch const machen
  
  // redundant memories
  MT::Array< uintA > nonDefaultRules_per_experience;  // only non-default rules!
  MT::Array< uintA > experiences_per_rule;
  
  MT::Array< MT::Array < uintA > > experiences_per_ruleOutcome;
  
  RuleSetContainer_ground(const ExperienceA* _p_examples);
  RuleSetContainer_ground(); // sollte nicht verwendet werden
  
  void init(const ExperienceA* _p_examples);
  void append(TL::Rule* rule, uintA& examples_of_this_rule, MT::Array< uintA >& examples_per_outcome_of_this_rule);
  void remove(uint id);
  void clear();
  void recomputeDefaultRule();
  void sort();
  
  void getResponsibilities(arr& responsibilities, MT::Array< uintA >& covered_examples, uintA& covered_examples_num) const;
  
  void writeNice(ostream& out = std::cout, bool only_action = false) const;
  void write_experiencesWithRules(ostream& os = std::cout) const;
  void write_rulesWithExperiences(ostream& os = std::cout) const;
  
  void sanityCheck() const;  // EXPENSIVE!!!
};





namespace CostFunction_ground {

  void setNoiseStateProbability(double p_min);
  void setRuleCoveredExamples(const ExperienceA& coveredEx);
  void setOutcomesCoverage(const boolA& coverage);
  void setPenaltySum(double pen_sum);
  void setPenaltyPos(double pen_pos);
  double loglikelihood(const arr& probs);
	
  // C = -LOG_LIK + PENALTIES
  // --> to minimize
  // need knowledge of covered examples and corresponding covered outcomes
  double calc(const arr& in);	
  void calc_grad(arr& out, const arr& in);
};



void calcCoverage_ground(ExperienceA& covered_examples, uintA& covered_examples_ids, const TL::Rule* ground_r, const ExperienceA& examples);




class SearchOperator_ground {
  const static uint PARAM_OPT__GRAD_DESC;
  const static uint PARAM_OPT__NEWTON;
  const static uint PARAM_OPT__RPROP;
  const static uint PARAM_OPT__CONJGRAD;
  const static uint PARAM_OPT__COND_GRAD;
  
  uint param_opt_type;
  double pen_sum;
  double pen_pos;

  protected:
    double alpha_PEN;
    double p_min;
    MT::String name;
    bool approximative;
	
    // Outcomes
    void calcCoverage_outcomes(const MT::Array< LitL >& outcomes, const ExperienceA& examples, const TL::Rule* rule, boolA& coverage);
    void calcSubsumption(boolA& subsumes, const boolA& coverage);
    // remove outcomes that (i) do not cover any example and (ii) have zero-probability  and (iii) sets coverage for cost function
    void produceTrimmedOutcomes(MT::Array< LitL >& outcomes, arr& probs, boolA& coverage, const ExperienceA& coveredExamples, const TL::Rule& rule);
		void induceOutcomes(TL::Rule* rule, MT::Array< uintA >& coveredExamples_per_outcome, const ExperienceA& covered_examples, const uintA& covered_examples_ids);
		
        // Parameter learning
		double learnParameters_constrainedCostfunction(const MT::Array< LitL >& outcomes, doubleA& probs);
		double learnParameters(const MT::Array< LitL >& outcomes, doubleA& probs);
        
    // takes the rulelist rules2add and integrates it into existing ruleset
    void integrateNewRules(const TL::RuleSetContainer_ground& rulesC_old, const TL::RuleSetContainer_ground& rules_2add,
                            const ExperienceA& examples, TL::RuleSetContainer_ground& rules_new);
    
    // Creates possible new rules for the given rule-set.
    // "rules_2add" are potential additional rules which are all supposed to become part of the SAME rule-set!
    // I.e., they will be integrated into the old rule-set.
    virtual void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add) = 0;
        
        
	public:
    SearchOperator_ground(double alpha_PEN, double p_min, uint PARAM_OPT_TYPE);
    virtual ~SearchOperator_ground() {}

    void setProbabilitiesLearningPenalty_sum(double pen_sum);
    void setProbabilitiesLearningPenalty_pos(double pen_pos);
		
    // resets search operator for next application round
    virtual void reset() = 0;

    // central method which is called by the RuleLearner
    virtual void createRuleSets(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, 
                    MT::Array< TL::RuleSetContainer_ground >& sets_of_new_rules);
    
    const char* getName();
    bool isApproximator() {return approximative;}
    virtual void reset_total_approximator() {}
};





class ExplainExperiences_ground : public SearchOperator_ground {
	uint nextPotentialExperience;
  bool slimPreconditions;
  bool comparingValues;
	public:
    ExplainExperiences_ground(bool slimPreconditions, bool comparingValues, double alpha, double p_min, uint PARAM_OPT_TYPE, double pen_sum = 100.0, double pen_pos = 1.0) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      if (!slimPreconditions) {
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
      this->slimPreconditions = slimPreconditions;
      this->comparingValues = comparingValues;
		}

		void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		TL::Rule* explainExperience_ground(Experience* ex);
		TL::Rule* explainExperience_straightforward_ground(Experience* ex);
		TL::Rule* explainExperience_deictic_ground(Experience* ex);
		
		// resets field "nextPotentialExperience
		void reset();
};


class DropPreconditions_ground : public SearchOperator_ground {
	uint nextRule;
	uint nextPrecondition;
	public:
    DropPreconditions_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
			name = "DropPreconditions";
			nextRule = 0;
			nextPrecondition = 0;
		}
		void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
		void reset();
};




class DropPreconditions_approximativeVersion_ground : public SearchOperator_ground {
	uintA usablePreconds;
    const static uint DROP_NEGATIVE_BIAS = 3;
    bool prepareTotalNewSearch;
//     uint producedRules;
	public:
        DropPreconditions_approximativeVersion_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "DropPreconditions_approximativeVersion";
            approximative = true;
            prepareTotalNewSearch = false;
		}
		void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
		void reset();
        void reset_total_approximator();
};




class DropReferences_ground : public SearchOperator_ground {
  uint nextRule;
  uint nextReference;
  public:
    DropReferences_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      name = "DropReferences";
      nextReference = 0;
      nextRule=0;
    }
    void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
    
    void reset();
};


class DropRules_ground : public SearchOperator_ground {
  public:
    DropRules_ground(double alpha, double p_min, uint PARAM_OPT_TYPE, double pen_sum = 100.0, double pen_pos = 1.0) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      name = "DropRules";
    }
    
    void createRuleSets(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, 
              MT::Array< TL::RuleSetContainer_ground >& sets_of_new_rules);
    
    // the following two rules are empty
    void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);   
    void reset();
};



// considers only primitives and complex (not equality literals)
class SplitOnLiterals_ground : public SearchOperator_ground {
  uint nextRule;
  uint nextLiteral;
  LitL absentLiterals;
  public:
        SplitOnLiterals_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      name = "SplitOnLiterals";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
    
    void reset();
};



class AddLiterals_ground : public SearchOperator_ground {
  uint nextRule;
  uint nextLiteral;
  LitL absentLiterals;
  public:
    AddLiterals_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      name = "AddLiterals";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
    
    void reset();
};

/*
class AddReferences_ground : public SearchOperator_ground {
  uint nextRule;
  uint nextLiteral;
  LitL restrictionLiterals;
  public:
        AddReferences_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
      name = "AddReferences";
      nextRule=1; // ignore default rule
      nextLiteral=0;
    }
    void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
    
    void reset();
};



// SOs on ComparisonPredicates

class GeneralizeEquality_ground : public SearchOperator_ground {
    uint nextRule;
    uint nextLiteral;
    bool doneLess;

    public:
        GeneralizeEquality_ground(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "GeneralizeEquality";
            nextRule=1; // ignore default rule
            nextLiteral=0;
            doneLess=false;
        }
        void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
    
        void reset();
};



// for each variable v for each function f for which a comparison predicate with v is not
// used yet, we introduce equality literals for all existing values for v.
class SplitOnEqualities : publc SearchOperator_ground {
  
  unaktuelle version...
  
    uint nextRule;
    uint nextVar;
    uint nextFunc;
    uintA vars;
    FuncL usedFunctions;
    
    public:
        SplitOnEqualities(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "SplitOnEqualities";
            nextRule=1; // ignore default rule
            nextVar=0;
            nextFunc=0;
            usedFunctions.append(le->f_prim);
            usedFunctions.append(le->f_derived);
        }
        void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
        void reset();
};



// changes ranges of all comp preds
class ChangeRange : public SearchOperator_ground {
    uint nextRule;
    uint nextLiteral;
    uint nextPossibleValue;
    arr possibleValues;

    public:
        ChangeRange(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "ChangeRange";
            nextRule=1; // ignore default rule
            nextLiteral=0;
            nextPossibleValue=0;
        }
        void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
        void reset();
};


// for all less/greater and less/greater-equal compPreds, we introduce
// less/greater-equal compPreds for all possible values
// only defined for constant comparisons!
class MakeInterval : public SearchOperator_ground {
    uint nextRule;
    uint nextLiteral;
    uint nextPossibleValue;
    arr possibleValues;

    public:
        MakeInterval(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "MakeInterval";
            nextRule=1; // ignore default rule
            nextLiteral=0;
            nextPossibleValue=0;
        }
        void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
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
class CompareFunctionValues : public SearchOperator_ground {
    uint nextRule;
    uint nextFunction;
    uint nextComparisonType;
    uint nextTermCombination;
    
    uintA comparisonTypes;
    MT::Array<uintA> termCombos;
    FuncL usedFunctions;
    
    MT::Array<uintA> coveredExIDsPerComparisonType; // only for debugging

    public:
        CompareFunctionValues(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
            name = "CompareFunctionValues";
            nextRule=1; // ignore default rule
            nextFunction=0;
            nextComparisonType=0;
            nextTermCombination=0;
            usedFunctions.append(le->f_prim);
            usedFunctions.append(le->f_derived);
            // collect comparison types SAFE THE ORDER!
            comparisonTypes.append(comparison_equal);comparisonTypes.append(comparison_less);comparisonTypes.append(comparison_lessEqual);comparisonTypes.append(comparison_greater);comparisonTypes.append(comparison_greaterEqual);
        }
        void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
		
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
class SplitOnCompareFunctionValues : public SearchOperator_ground {
    uint nextRule;
    uint nextFunction;
    uint nextTermCombination;
    uintA comparisonTypes;
    MT::Array<uintA> termCombos;
    FuncL usedFunctions;
    
    MT::Array<uintA> coveredExIDsPerComparisonType; // only for debugging

    public:
      SplitOnCompareFunctionValues(double alpha, double p_min, uint PARAM_OPT_TYPE) : SearchOperator_ground(alpha, p_min, PARAM_OPT_TYPE) {
        name = "SplitOnCompareFunctionValues";
        nextRule=1; // ignore default rule
        nextFunction=0;
        nextTermCombination=0;
        usedFunctions.append(le->f_prim);
        usedFunctions.append(le->f_derived);
        // collect comparison types SAFE THE ORDER!
        comparisonTypes.append(comparison_equal);comparisonTypes.append(comparison_less);comparisonTypes.append(comparison_greater);
      }
      void findRules(const TL::RuleSetContainer_ground& rulesC_old, const ExperienceA& examples, TL::RuleSetContainer_ground& rules_2add);
  
      void reset();
};*/














class RuleLearner_ground {

  MT::Array< SearchOperator_ground* > searchOperators;
  double alpha_PEN;
  double p_min;
  double p_min_noisyDefaultRule;
//   double probabilitiesLearning__pen_pos;
//   double probabilitiesLearning__pen_sum;

  // Default rule: {} --[]--> p1: noChange, p2: noise
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
    RuleLearner_ground(double alpha_PEN, double p_min, double p_min_noisyDefaultRule, uint param_opt_type, uint so_choice_type = 2);
    ~RuleLearner_ground();
  
    void setAlphaPEN(double alpha_PEN);
    
    double score(TL::RuleSetContainer_ground& rulesC, ExperienceA& experiences, double cutting_threshold);
    double score(TL::RuleSetContainer_ground& rulesC, ExperienceA& experiences, double cutting_threshold, arr& experience_weights);
    void learn_rules(TL::RuleSetContainer_ground& rulesC, ExperienceA& experiences, const char* logfile = "ruleLearner_ground.log"); 
    void learn_rules(TL::RuleSetContainer_ground& rulesC, ExperienceA& experiences, arr& experience_weights, const char* logfile = "ruleLearner_ground.log"); 
};

}

#endif // TL__RULE_LEARNER_GROUND
