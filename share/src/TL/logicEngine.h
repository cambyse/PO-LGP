/*  
    Copyright 2009   Tobias Lang
    
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


#ifndef TL__LOGIC_ENGINE
#define TL__LOGIC_ENGINE

#include <TL/relationalLogic.h>



namespace TL {
    
/****************************************
     MANAGING CONCEPT-INSTANCE OBJECTS
 ***************************************/
  
extern uint LOGIC_ENGINE__MAX_ID_NUMBER;

// one per <predicate,positive> combination, e.g. for "-on(.,.)"
struct PredicateInstanceList {
  bool positive;
  Predicate* pred;
  PredIA mem;

  PredicateInstanceList();
  PredicateInstanceList(Predicate* pred, bool positive);
  ~PredicateInstanceList();
  void init(Predicate* pred, bool positive);

  PredicateInstance* get(uintA& args);
};

// one per <function,compType> combination, e.g. for "size(.)<="
struct ComparisonPredicateInstanceList {
  ComparisonPredicate* pred;
  Function* f;
  uint compType;
  MT::Array< CompPredIA > mem;

  ComparisonPredicateInstanceList();
  ComparisonPredicateInstanceList(ComparisonPredicate* pred, Function* f, uint compType);
  ~ComparisonPredicateInstanceList();
  void init(ComparisonPredicate* pred, Function* f, uint compType);

  ComparisonPredicateInstance* get_constant(uintA& args, double bound);
  ComparisonPredicateInstance* get_dynamic(uintA& args);
};
    
struct FunctionValueList {
  Function* f;
  MT::Array< FuncVA > mem;
  
  FunctionValueList();
  FunctionValueList(Function* f);
  ~FunctionValueList();
  void init(Function* f);
  
  FunctionValue* get(uintA& args, double value);
};
    
struct FunctionInstanceList {
  Function* f;
  FuncIA mem;
    
  FunctionInstanceList();
  FunctionInstanceList(Function* f);
  ~FunctionInstanceList();
  void init(Function* f);
    
  TL::FunctionInstance* get(uintA& args);
};

struct LogicObjectManager {
  PredA p_comp; // needs to be saved in order to be able to add functions later

  MT::Array< PredicateInstanceList > lists_pi;  // 1D
  MT::Array< ComparisonPredicateInstanceList > lists_comp; //  3D -- dim 1 predicate-id,  dim 2 comparison type,  dim 3 constant-bound
  MT::Array< FunctionValueList > lists_fv;  // 1D
  MT::Array< FunctionInstanceList > lists_fi;  // 1D

  LogicObjectManager(PredA& predicates, FuncA& functions);	
  ~LogicObjectManager();

  void setPredicates(PredA& preds);
  void addPredicate(Predicate* pred);
  void setFunctions(FuncA& preds);
  void addFunction(Function* f);
  
  PredicateInstance* get(Predicate* p, bool positive, uintA& args);
  inline uint calcID_pred(uint pred_id, bool positive);
  
  ComparisonPredicateInstance* getComp_constant(TL::Function* f, uint compType, double bound, uintA& args);
  ComparisonPredicateInstance* getComp_dynamic(TL::Function* f, uint compType, uintA& args);
  
  FunctionValue* get(Function* f, uintA& args, double value);
  FunctionInstance* get(Function* f, uintA& args);
};





/****************************************
     DEPENDENCY GRAPH
     (for derived concepts)
 ***************************************/

class ConceptDependencyGraph {
  boolA edges;
  uint derivedStartID;
  boolA id2pred;
  std::map<uint, uint> pred2graph;
  std::map<uint, uint> func2graph;
  std::map<uint, uint> graph2pred;
  std::map<uint, uint> graph2func;
  PredA predicates;
  FuncA functions;
  
  uintA order, order__derOnly;
  boolA order_isPredicate, order_isPredicate__derOnly;
  void calcWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly);
  
  public:
    TL::Predicate* getPredicate(uint id) const;
    TL::Function* getFunction(uint id) const;
    
    void resetDefault();
    void setNodes(PredA& predicates, FuncA& functions);

    bool isAcyclic();

    void getAllPrecessors(const TL::Predicate& p, PredA& pre_preds, FuncA& pre_funcs);
    void getAllPrecessors(const TL::Function& f, PredA& pre_preds, FuncA& pre_funcs);
    void getAllSuccessors(const TL::Predicate& p, PredA& suc_preds, FuncA& suc_funcs);
    void getAllSuccessors(const TL::Function& f, PredA& suc_preds, FuncA& suc_funcs);
    
    // order on functions and predicates according to dependencies
    // (richtige bezeichnung dafuer grad meinem gehirn entfallen)
    void getWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly);
    
    void writeNice(ostream& os = std::cout);
};






/****************************************
     LOGIC ENGINE
 ***************************************/


// Why use a LogicEngine OBJECT (and not just static methods) ?
// --> maintains list of derived concepts
// --> list of constants: can distinguish between variables and constants
// --> manages concept-instance objects

struct LogicEngine {
	uintA constants;
  TermTypeA constants_types;
  
  PredA actions;
	PredA p_prim;
	PredA p_derived;
  PredA p_comp;
  FuncA f_prim;
  FuncA f_derived;
  TermTypeA types;
  
  ConceptDependencyGraph dependencyGraph;

  LogicEngine(uintA& constants, PredA& p_prim, PredA& p_derived, PredA& p_comp, FuncA& f_prim, FuncA& f_derived, PredA& actions);
  LogicEngine(PredA& p_prim, PredA& p_derived, PredA& p_comp, FuncA& f_prim, FuncA& f_derived, PredA& actions);
  LogicEngine(const char* language_file, uint fileType = 1);
  
  ~LogicEngine();
  
  void initLOM(uintA& constants); // LogicObjectManager
	
	
	/****************************************
	       LANGUAGE HANDLING
	 ***************************************/
  // use with care...
  void initLanguage(PredA& p_prim, PredA& p_derived, PredA& p_comp, FuncA& f_prim, FuncA& f_derived, PredA& actions);
  
  void setConstants(uintA& constants);
  void setConstants(uintA& constants, const TermTypeA& constants_types);
  
  void addPredicates(PredA& preds);
  void addActions(PredA& actions);
  void addFunctions(FuncA& funcs);
    
  TL::Predicate* getPredicate(const MT::String& name) const;
  TL::Function* getFunction(const MT::String& name) const;
  TL::Predicate* getPredicate(uint id) const;
  TL::Function* getFunction(uint id) const;
  
  static void orderPredicates(PredA& primitives, PredA& p_derived, PredA& p_comp, const PredA& preds);
  static void orderFunctions(FuncA& primitives, FuncA& f_derived, const FuncA& funcs);
  
  TL::Predicate* create_doNothing_actionAndPI();
  TL::PredicateInstance* getPI_doNothing();
  
  TL::TermType* getTermTypeOfObject(uint object_id);
  
  
  
  /****************************************
      BASIC HELPERS
   ***************************************/
  
  bool isConstant(uint id);
    
  uint calcNumTerms(const TL::PredicateInstance& pt);
  static uint calcTerms(const TL::PredicateInstance& pt, uintA& terms);
  static uint calcTerms(const PredIA& pis, uintA& terms);
  void calcFreeVars(const TL::ConjunctionPredicate& scp, uintA& freeVars_pos, uintA& freeVars_neg);
  void calcUnconstrainedNegatedArguments(TL::ConjunctionPredicate* cp, uintA& vars);
  static bool containsNegativeBasePredicate(TL::Predicate* cp);
  bool isEqualityLiteral(TL::PredicateInstance* pt);
  static uint numberLiterals(const MT::Array< PredIA >& PredIAs);
  
  // lists
  static void getConstants(const FuncVA& fvs, uintA& constants);
  static void getConstants(const PredIA& pts, uintA& constants);
  static bool containsLiteral(const PredIA& p, const TL::PredicateInstance& literal);
  static bool containsNegativePredicateInstances(const PredIA& PredIAs);
  void filterPurelyAbstract(const PredIA& allPreds, PredIA& filteredPreds);
  void negate(const PredIA& predTs, PredIA& predTs_negated);
  static int findPattern(const PredIA& actions, uint minRepeats);
  
  // order lists
  static void order(PredIA& p); // sort
  void orderNegativesUngroundedLast(PredIA& p);
  static bool negativesLast(const PredIA& predTs);
  bool negativesUngroundedLast(const PredIA& predTs);
  
  
  /****************************************
     HELPERS FOR STATE
   ***************************************/
  static void getConstants(const TL::State& s, uintA& constants);
  static uint getArgument(const TL::State& s, const TL::Predicate& pred);
  static void getArguments(uintA& args, const TL::State& s, const TL::Predicate& pred);
  static void getValues(arr& values, const TL::State& s, const TL::Function& f, const uintA& objs);
  void usedValues(const TL::Function& f, const TL::State& s, arr& values);
  static bool containsNegativePredicateInstances(const TL::State& s);
  static void filterState_full(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly = true);  // only "filter_objects" as args
  static void filterState_atleastOne(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly = true); // at least one in "filter_objects" as arg
  static void changes(const TL::State& pre, const TL::State& post, uintA& changedConstants, PredIA& holdOnlyPre, PredIA& holdOnlyPost);
  // pred(X,Y), pred(X,Z) --> {Y,Z}
  static void getRelatedObjects(uintA& objs_related, uint id, bool id_covers_first, const TL::Predicate& pred, const TL::State& s);
  // TODO "X" am Ende des Methodennamens wieder loeschen
  // TODO hieran weiter arbeiten fuer den Graphen
  static void getGeneralRelatedObjectsX(uintA& objs_related, uint id, const TL::State& s);
  double getValue(uint id, const MT::String& function_name, const TL::State& s);
  static double getValue(TL::Function* f, const TL::State& s);
  static double getValue(uint id, TL::Function* f, const TL::State& s);
  static double getValue(const uintA& args, TL::Function* f, const TL::State& s);
  
  
  /****************************************
     GENERATING PREDICATE TUPLES
  ***************************************/
  
	// creating pts / all possible groundings
  void generateAllPossiblePredicateInstances(PredIA& pts, TL::Predicate* pred, const uintA& arguments, bool positiveOnly = false);
  void generateAllPossiblePredicateInstances(PredIA& pts, TL::Function* func, const uintA& arguments);
  void generateAllPossiblePredicateInstances(PredIA& pts, const uintA& arguments, bool positiveOnly = false);
  void generateAllPossiblePredicateInstances(PredIA& pts, const uintA& arguments, const uintA& arguments_mustBeContained, bool positiveOnly = false);
  void generateAllPossiblePredicateInstances_actions(PredIA& action_pts, const uintA& arguments);
  void generateAllPossibleFunctionInstances(FuncIA& fis, TL::Function* func, const uintA& arguments);
  void generateAllPossibleFunctionInstances(FuncIA& fis, const uintA& arguments);
  // what = 0 --> equality
  // what = 1 --> less than[?]
  // what = 2 --> greater than[?]
  // what = 3 --> all, even compare two objects
  // all p_comp to constants!
  void generateComparisonPredicateInstances(PredIA& pis, const uintA& arguments, const TL::State& s, uint what);
  // in case of free vars, instantiates all possible guys!
  void generateBaseInstantiations(PredIA& pts_base, FuncVA& fvs_base, TL::PredicateInstance* pt);
  void generateBaseInstantiations(PredIA& pts_base, FuncVA& fvs_base, TL::PredicateInstance* pt, uintA& free_constants);
  
  void generateComparisonPredicateInstances_dynamic(PredIA& pts, const uintA& arguments, const TL::State& s, uint what);
  
  void createAllPossibleSubstitutions(const uintA& vars, TL::SubstitutionSet& subs);
    
  // dealing with derived pi's and fv's
  TL::PredicateInstance* generateConjunctionPredicateInstance(TL::ConjunctionPredicate* cp, TL::Substitution* sub);
  TL::PredicateInstance* generateTransClosurePredicateInstance(TL::TransClosurePredicate* p, uintA& args);
  TL::PredicateInstance* generateCountPredicateInstance(TL::CountPredicate* p, uintA& args);
//     NOT TO EXIST:  TL::PredicateInstance* generatePredicateInstance(TL::Predicate* p, uintA& args);
  void generatePredicateInstances(TL::Predicate* p, const uintA& arguments, PredIA& pts);
  
  TL::FunctionValue* generateFunctionValue(TL::Function* f, uintA& args, double value);
    
	
	
  /****************************************
    ABSTRACTING AND GROUNDING
  ***************************************/
  
  bool isGrounded(TL::PredicateInstance* predT);
  bool isGrounded(TL::FunctionValue* funcV);
  bool isAbstract(TL::PredicateInstance* predT);
	
	// abstracting
  void createInverseSubstitution(const TL::PredicateInstance& predT, TL::Substitution& sub);
	

    
    
  /****************************************
      MISC BASIC LOGIC
  ***************************************/
    
  static bool nonContradicting(const PredIA& p1, const PredIA& p2);
  void removeRedundant(PredIA& p);
   
  
  
  /****************************************
    MISC ADVANCED LOGIC
  ***************************************/
    
  void determineStupidDerivedConcepts(MT::Array< TL::Trial* >& data, PredA& stupidPredicates, FuncA& stupidFunctions, bool deleteFromLogicEngine);
    
//     delete sons of each ConjunctionPredicateInstance without free variables
  void killBaseConcepts(PredIA& pts);
  
  
    
    
  /****************************************
      HOLDS (--> for grounded)
  ***************************************/

  // GOAL: Coverage for grounded concept instances (-> simple existence check)
  
  // Allows for positive and negative predicate instances.
  // In contrast to "cover(.)" which covers abstract / ungrounded predicates (see below)
	// Assumptions:
	// (1) Everything is grounded. (States, PredicateInstances)
	// (2) Knowledge base "groundedPredTs" contains only positive literals!!
  static bool holds_positive(const PredIA& positive_groundedPredTs, TL::PredicateInstance* pt);
  static bool holds(const PredIA& positive_groundedPredTs, TL::PredicateInstance* pt);
  // special function for ComparisonPredicateInstances
  static bool holds(const FuncVA& fvs, TL::ComparisonPredicateInstance* pt);
	// Additional assumptions for State variants:
	// (1) All complex predicate tuples have been derived in the states.
	// --> simple search
  static bool holds(const TL::State& s, TL::PredicateInstance* grounded_pi);
  static bool holds(const TL::State& s, const PredIA& grounded_pis);
	// Attention: we look only at the primitive predicates thus far!!
  static bool holds(const TL::State& s1, const TL::State& s2);
    
  // "holds_straight(.)": fast variant of "holds(TL::State* s, TL::PredicateInstance* grounded_pi)"
  bool holds_straight(uint id, const MT::String& predicate_name, const TL::State& s) const;
	
  
	/****************************************
	       UNIFY
	 ***************************************/
  
  // "unify" unifies literals.
  // "cover" uses "unify" to check coverage of literals in states.
    
  bool unify_basic(uintA& grounded_args, uintA& other_args, TL::Substitution& sub);
	bool unify(TL::PredicateInstance* grounded_pt, TL::PredicateInstance* other_pt, TL::Substitution& sub);
  // used for constant-bound cpt
	bool unify(TL::FunctionValue* grounded_fv, TL::ComparisonPredicateInstance* cpt_constant, TL::Substitution& sub);
  // used for dynamic-bound cpt
  bool unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonPredicateInstance* cpt_dynamic, TL::Substitution& sub);

  
  /****************************************
      COVER (--> for abstract)
   ***************************************/
  
  // GOAL: Coverage for abstract concept instance arrays
  
  // SEMANTICS
  // Free variables:
  // In "cover", free variables are dealt with depending upon the literal they are involved in:
  // (i)  Positive literals: existential quantification, i.e., we need to find a single substitution
  //      for the literal to hold
  // (ii) Negative literals: there are two possibilities specified by the "freeNegVarsAllQuantified"-flag.
  //      In either case, free variables in negative literals lead to a substantial computational burden
  //      and should be avoided. Therefore, we usually order PredicateInstance arrays such that
  //      the positive literals appear first (which bind many variables).
  //      (A) All-quantification:
  //          A predicate tuple -p(X1,...,Xn) is only true if p does not hold for any
  //          substitution of the free variables.
  //      (B) Existential-quantification:
  //          A predicate tuple -p(X1,...,Xn) is true with a substition S={X1->c1,...,Xn-->cn}
  //          such that p(c1,...,cn) does not hold.
  
  // USAGE
  // Substitution object instances:
  // - All substitutions in "subs" arrays found by "cover" are newly created Substition instances. It is
  //   the responsibility of the calling methods to delete them!
  // - If a cover method returns false, the "subs" array is supposed to be empty. (So, the caller does
  //   not have to care about deleting object instances.)
    
  // BASIC FUNCTION 1
	// Calculates substitutions "subs" (extensions of "initSub") which
	// substitute the arguments in "predT" such that the resulting
	// grounded predicate is satisfied in state "s".
  bool cover(const TL::State& s, TL::PredicateInstance* pt, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub = NULL);
  // BASIC FUNCTION 2
	// Calculates substitutions "subs" (extensions of "initSub") which
	// substitute the arguments in the predicate list "predTs" such that the resulting
	// grounded predicates are satisfied in state "s".
  bool cover(const TL::State& s, const PredIA& predTs, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub = NULL);
    
    
    
	
  /****************************************
            STATE UNIFICATION 
   ***************************************/
  void calcDifferences(PredIA& pi_diff_1to2, FuncVA& fv_diff_1to2, PredIA& pi_diff_2to1, FuncVA& fv_diff_2to1, const TL::State state1, const TL::State state2);
  // returns min-difference in number of literals
  uint unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub = NULL);
  bool unify(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub = NULL);
  bool unifiable(const TL::State& state1, const TL::State& state2);
  
  uint unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::PredicateInstance& action1, const TL::State& state2, const TL::PredicateInstance& action2);

	
	/****************************************
            DERIVING COMPLEX
	 ***************************************/
	
  // specialized methods
  bool derivePredicateInstances_conjunction(TL::ConjunctionPredicate& p, TL::State& s);
  bool derivePredicateInstances_transClosure(TL::TransClosurePredicate& p, TL::State& s);
  bool derivePredicateInstances_count(TL::CountPredicate& p, TL::State& s);
  bool deriveFunctionValues_count(TL::CountFunction& f, TL::State& s);
  bool deriveFunctionValues_avg(TL::AverageFunction& f, TL::State& s);
  bool deriveFunctionValues_max(TL::MaxFunction& f, TL::State& s);
  bool deriveFunctionValues_sum(TL::SumFunction& f, TL::State& s);
  bool deriveFunctionValues_reward(TL::RewardFunction& f, TL::State& s);
    
  void derive(const PredIA& pi_prim, const FuncVA& fv_prim, PredIA& pi_derived, FuncVA& fv_derived);
  void derive(TL::State* s);
  
  static void dederive(TL::State* s);

  
  /****************************************
      INTERFACE TO DEPENDENCY GRAPH
  ***************************************/
  void getAllPrecessors(const TL::Predicate& p, PredA& pre_preds, FuncA& pre_funcs);
  void getAllPrecessors(const TL::Function& f, PredA& pre_preds, FuncA& pre_funcs);


	/****************************************
	       READ & WRITE
	 ***************************************/

  //   void readLanguage(const char* filename);
  void writeLanguage(const char* filename);
  // takeConstants --> use constants in state as constants of LogicEngine
  TL::State* readState(const char* filename, bool takeConstants);
  
  
	
	/****************************************
			MANAGING OBJECT INSTANCES
	***************************************/
	TL::LogicObjectManager* lom;
	
  TL::PredicateInstance* getPI(TL::Predicate* pred, bool positive, uintA& args);
  TL::PredicateInstance* getPI(const char* text);
  void getPIs(PredIA& pis, const char* text);
  TL::ComparisonPredicateInstance* getCompPT_constant(TL::Function* f, uint compType, double bound, uintA& args);
  TL::ComparisonPredicateInstance* getCompPT_dynamic(TL::Function* f, uint compType, uintA& args);
  TL::FunctionValue* getFV(TL::Function* f, uintA& args, double value);
  TL::FunctionInstance* getFI(TL::Function* f, uintA& args);
    
	// to retrieve the original
  TL::PredicateInstance* getPIorig(TL::PredicateInstance* pt_copy);
  TL::FunctionInstance* getFIorig(TL::FunctionInstance* fi_copy);
  TL::FunctionValue* getFVorig(TL::FunctionValue* fv_copy);
  TL::PredicateInstance* getPIneg(TL::PredicateInstance* pt);
	void makeOriginal(TL::State& s);
	void makeOriginal(TL::Trial& t);
  
	
  // (muessen Member-Funktionen sein wegen PredicateInstance-Erzeugung)
	TL::PredicateInstance* applyOriginalSub(TL::Substitution& sub, TL::PredicateInstance* pt);
	void applyOriginalSub(TL::Substitution& sub, const PredIA& unsubPreds, PredIA& subPreds);
};


}



#endif // TL__LOGIC_ENGINE
