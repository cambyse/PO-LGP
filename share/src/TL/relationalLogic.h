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

#ifndef TL__RELATIONAL_LOGIC
#define TL__RELATIONAL_LOGIC

#include <MT/util.h>
#include <TL/utilTL.h>
#include <map>

#define TL_TERM_TYPE_SIMPLE 0
#define TL_TERM_TYPE_DISJUNCTION 1

#define TL_PRIMITIVE 0
#define TL_DERIVED 1

#define TL_PRED_ACTION 0
#define TL_PRED_SIMPLE 1
#define TL_PRED_COMPARISON 2
#define TL_PRED_CONJUNCTION 3
#define TL_PRED_TRANS_CLOSURE 4
#define TL_PRED_COUNT 5

#define TL_FUNC_SIMPLE 0
#define TL_FUNC_COUNT 1
#define TL_FUNC_AVG 2
#define TL_FUNC_MAX 3
#define TL_FUNC_CHANGE 4
#define TL_FUNC_SUM 5
#define TL_FUNC_REWARD 6

#define TL_COMPARISON_EQUAL 1
#define TL_COMPARISON_LESS 2
#define TL_COMPARISON_LESS_EQUAL 3
#define TL_COMPARISON_GREATER 4
#define TL_COMPARISON_GREATER_EQUAL 5

// "Action" of default rule: "null" with reserved ID
#define TL_DEFAULT_ACTION_PRED__ID 44444

#define TL_DEFAULT_FUNCTION_VALUE -1000

#define FOR1D_(x,i)   for(i=0;i<x.num();i++)
#define FOR1D_DOWN_(x,i)  for(i=x.num();i--;)


namespace TL {

  
  
/****************************************
     TYPES
 ***************************************/

class TermType {
  protected:
    uint typeI;
  public:
    const static uint ANY_id = 0;
    
    uint type_id;
    MT::String name;
    
    virtual bool operator==(const TermType& t) const;
    virtual bool operator!=(const TermType& t) const {
      return !(*this == t);
    }
    virtual bool subsumes(const TermType& t) const;
    
    TermType() {typeI = TL_TERM_TYPE_SIMPLE;}
    ~TermType() {}
    
    virtual void writeNice(ostream& os = cout) const;
    virtual void write(ostream& os) const;
};


struct DisjunctionTermType : public TermType {
  public:
    MT::Array<TermType*> base_types;
  
    bool subsumes(const TermType& t) const;
    
    DisjunctionTermType() {typeI = TL_TERM_TYPE_DISJUNCTION;}
    ~DisjunctionTermType() {}
    
    void writeNice(ostream& os = cout) const;
    void write(ostream& os) const;
};



/****************************************
     CONCEPTS
 ***************************************/


// ---------------------------------------
//  Primitives

class Predicate {
  public:
    uint id; // each predicate has individual ID  (0 is reserved for NULL-action of default rule)
    MT::String name;
    uint d; // arity
    uint type; // simple, comparison, conjunction...
    uint category; // primitive or p_derived
    MT::Array< TermType* > arg_types;
  
    Predicate() {
      type = TL_PRED_SIMPLE;
      category = TL_PRIMITIVE;
    }
    virtual ~Predicate() {}

    virtual bool operator==(const Predicate& p) const;  // not via id!!
    virtual bool operator!=(const Predicate& p) const {
      return !(*this == p);
    }
  
    virtual void writeNice(ostream& os = cout) const;
    virtual void write(ostream& os) const;
};


class Function {
	public:
		uint id;
    MT::String name;
		uint d; // arity
    uint type; // simple, count, ...
    uint category; // primitive or derived
    uintA range;
    bool real_range; // range in R (in this case, we don't need uintA range)
  
    Function() {
        type = TL_FUNC_SIMPLE;
        category = TL_PRIMITIVE;
        real_range = false;
    }
    
    virtual ~Function() {}
    
    // not via id!
    virtual bool operator==(const Function& f) const;
    
    virtual bool operator!=(const Function& f) const {
        return !(*this == f);
    }
        
		virtual void writeNice(ostream& os = cout) const;
        
    virtual void write(ostream& os) const;
};



class ComparisonPredicate : public Predicate {
  public:
    bool constantBound;
    ComparisonPredicate() {
      constantBound = true;
      type = TL_PRED_COMPARISON;
    }
    void write(ostream& os) const;
};




// ---------------------------------------
//  Derived functions

class CountFunction : public Function {
  public:
    CountFunction() {
        type = TL_FUNC_COUNT;
        category = TL_DERIVED;
        countedPred = NULL;
        max_value = -1;
    }
    
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
    
    Predicate* countedPred;
    uintA countedPred_mapVars2derived;
    int max_value;
    double reward_for_max_value;
};



class AverageFunction : public Function {
  public:
    AverageFunction() {
      type = TL_FUNC_AVG;
      category = TL_DERIVED;
      real_range = true;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};


class SumFunction : public Function {
  public:
    SumFunction() {
      type = TL_FUNC_SUM;
      category = TL_DERIVED;
      real_range = false;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};



class MaxFunction : public Function {
  public:
    MaxFunction() {
      type = TL_FUNC_MAX;
      category = TL_DERIVED;
      real_range = false;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};


class ChangeFunction : public Function {
  public:
    ChangeFunction() {
      type = TL_FUNC_CHANGE;
      category = TL_DERIVED;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;

    Function* f_base;
};


class PredicateInstance;
class RewardFunction : public Function {
  public:
    RewardFunction() {
      type = TL_FUNC_REWARD;
      category = TL_DERIVED;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;

    double reward_value;
    MT::Array< TL::PredicateInstance* > grounded_pis;
};







// ---------------------------------------
//  Derived predicates


class DerivedPredicate : public Predicate {
  public:
    DerivedPredicate() {
          category = TL_DERIVED;
      }
      virtual ~DerivedPredicate() {}
      
      virtual DerivedPredicate* clone() = 0;
};


class ConjunctionPredicate : public DerivedPredicate {
  public:
    ConjunctionPredicate () {
      type = TL_PRED_CONJUNCTION;
    }
    ~ConjunctionPredicate () {
    }
    MT::Array< TL::Predicate* > basePreds;
    boolA basePreds_positive;
    bool freeVarsAllQuantified;
		// domain: {0, ..., total #slots in basePreds - 1}
		// range: N --> {0,...,DerivedPredicate.arity-1} for bounded vars, rest for free vars
		// example: clear(V1) = not on(V2, V1);
		// basePreds_mapVars2conjunction(0) = 1
		// basePreds_mapVars2conjunction(1) = 0
    uintA basePreds_mapVars2conjunction;

    void writeNice(ostream& os = cout) const;
    void write(ostream& os) const;
    void getFreeVars(uintA& freeVars) const;
    
    ConjunctionPredicate* clone();
};



// assume d=2 thus far!
class TransClosurePredicate : public DerivedPredicate {
  public:
    TL::Predicate* basePred;
    
    TransClosurePredicate () {
        type = TL_PRED_TRANS_CLOSURE;
    }
    ~TransClosurePredicate () {
    }
    void writeNice(ostream& os = cout) const;
    void write(ostream& os) const;
    
    TransClosurePredicate* clone();
};


class CountPredicate : public DerivedPredicate {
  public:
    CountPredicate () {
        type = TL_PRED_COUNT;
        countedPred_isComparison = false;
    }
    ~CountPredicate () {
    }
    void writeNice(ostream& os) const;
    void write(ostream& os) const;
    CountPredicate* clone();
    
    TL::Predicate* countedPred;
    uintA countedPred_mapVars2derived; // d0 = countedPred.d !!
    uint bound;
    uint compType;
    
    // special fields in case of using comparison predicate
    bool countedPred_isComparison;
    Function* comparison_f;
    uint comparison_compType;
    double comparison_bound;
};










/****************************************
     CONCEPT INSTANCES
 ***************************************/


struct FunctionInstance {
  TL::Function* f;
  uintA args;
  void write(ostream& os) const;
  void writeNice(ostream& os = cout) const;
};



class FunctionValue {
	public:
		uintA args;
		Function* f;
		double value;
		bool operator==(FunctionValue& pt) const;
		bool operator!=(FunctionValue& pt) const;
		
    FunctionValue* clone();
        
		void write(ostream& os) const;
		void writeNice(ostream& os = cout) const;
};



class PredicateInstance {
	public:
		Predicate* pred;
		uintA args;
		bool positive;
		bool operator==(PredicateInstance& pt) const;
		bool operator!=(PredicateInstance& pt) const;
		virtual ~PredicateInstance() {}
        
    void name(MT::String& name) const;
        
    virtual PredicateInstance* clone();
		virtual void write(ostream& os) const;
		virtual void writeNice(ostream& os = cout, bool withTypes = false) const;
		void read(istream& is);
};



class ComparisonPredicateInstance : public PredicateInstance {
	public:
    // slot assignments are handled by PredicateInstance
		Function* f;
		double bound;
    uint comparisonType;
    bool operator==(PredicateInstance& pt) const;
		
    ComparisonPredicateInstance* clone();
    bool compare(double value);
    bool compare(double value_left, double value_right);
		void write(ostream& os) const;
		void writeNice(ostream& os = cout, bool withTypes = false) const;
        
    inline bool hasConstantBound() const {
//         ComparisonPredicate* cp = dynamic_cast<ComparisonPredicate*>(this->pred);
//         CHECK(cp!=NULL,"Cast failed")
      return ((ComparisonPredicate*)this->pred)->constantBound;
    }
};





/****************************************
     RULE
 ***************************************/

struct Rule {
	MT::Array< TL::PredicateInstance* > context;
	TL::PredicateInstance* action;
	MT::Array< MT::Array< TL::PredicateInstance* > > outcomes;
	doubleA probs;
	double noise_changes; // PRADA's noise outcome heuristic: average number of state properties that have changed in case of the noise outcome
  arr outcome_rewards; // optional
  
	int numRefs; // used for object management by RuleSet container class
	
	static uint globalRuleCounter;
	
	Rule() {
		numRefs=0;
		globalRuleCounter++;
    noise_changes = 0.0;
//     cout<<"new rule - now "<<globalRuleCounter<<endl;
	}
	
	~Rule() {
// 		cout << "Ich zaehl derb runter: "; PRINT(globalRuleCounter)
		globalRuleCounter--;
//     cout<<"destroyed rule - now "<<globalRuleCounter<<endl;
	}
  
  void copyBody(const TL::Rule& other);
  void involvedConcepts(MT::Array< TL::Predicate* >& preds, MT::Array< TL::Function* >& funcs);

	void write(ostream& os) const;
  void writeNice(ostream& os = std::cout, bool withAction = true) const;
  
  private:
    // cannot copy because of numRefs
    DISALLOW_COPY_AND_ASSIGN(Rule);
};



class RuleSet {
  MT::Array< TL::Rule* > ra;
  
  public:
  RuleSet();
  ~RuleSet();
  
  void append(TL::Rule* r);
  void remove(uint i);
  TL::Rule* elem(uint i) const;
  int findValue(TL::Rule* r);
  void clear();
  uint num() const;
  void overwrite(uint i, TL::Rule* r);
  RuleSet& operator=(const RuleSet& rs);
  void sort(); // sorts rule-set by actions
  void sort_using_args();
  
  void writeNice(ostream& os = std::cout);
};



class State;


/****************************************
     SUBSTITUTION
 ***************************************/

class Substitution {
	uintA ins;
	uintA outs;
	
	public:
		int numRefs;
		
    static int globalCounter_Substitution;
        
		Substitution() {
			numRefs=0;
      globalCounter_Substitution++;
		}
        
    Substitution(const Substitution& s) {
        *this = s;
        numRefs = 0;
        globalCounter_Substitution++;
    }
		
    ~Substitution() {
        globalCounter_Substitution--;
    }
        
		void apply(const MT::Array< TL::PredicateInstance* >& unsubPreds, MT::Array< TL::PredicateInstance* >& subPreds);
    void apply(const MT::Array< TL::FunctionValue* >& unsubFVs, MT::Array< TL::FunctionValue* >& subFVs);
		PredicateInstance* apply(PredicateInstance* unsubPred);
    FunctionValue* apply(FunctionValue* unsubFV);
    TL::State* apply(const TL::State& state);
		uint getSubs(uint in);
		void getIns(uintA& ids) const;
		void getOuts(uintA& ids) const;
		void addSubs2Variable(uint in);
		void addSubs(uint in, uint out);
		bool hasSubs(uint in);
		bool mapsToDistinct();
    bool empty();
    uint num() {
        return this->ins.N;
    }
    void getInverse(Substitution& invSub);
		
    Substitution& operator=(const Substitution& s);
        
		void write(ostream& os) const;
		void writeNice(ostream& os = cout);
        
    static Substitution* combine(Substitution& sub1, Substitution& sub2);
};


class SubstitutionSet {
  MT::Array< TL::Substitution* > sa;

  public:
  SubstitutionSet();
  ~SubstitutionSet();

  void append(TL::Substitution* s);
  void append(TL::SubstitutionSet& ss);
  void remove(uint i);
  int findValue(TL::Substitution* s);
  void clear();
  uint num() const;
  TL::Substitution* elem(uint i) const;
  TL::Substitution* last() const;
  void overwrite(uint i, TL::Substitution* s);
  
  SubstitutionSet& operator=(const SubstitutionSet& ss);
};




/****************************************
     STATE & TRIAL & EXPERIENCE
 ***************************************/

struct State {
	MT::Array<PredicateInstance*> pi_prim;
	MT::Array<PredicateInstance*> pi_derived;
  MT::Array<PredicateInstance*> pi_comp;
	MT::Array<FunctionValue*> fv_prim;
  MT::Array<FunctionValue*> fv_derived;
	bool derivedDerived; // Have derived concepts been calculated?
    
	State() {
    derivedDerived = false;
	}
  ~State();
	
	bool operator==(const State& pt) const;
  bool operator!=(const State& pt) const;
	
	void write(ostream& os) const;
  void writeNice(ostream& os = std::cout, bool breaks = false, bool primOnly = false) const;
};

struct Trial {
	uintA constants;
	MT::Array<State*> states;
	MT::Array<PredicateInstance*> actions;
  
  // potential non-logical additional info [start]
  MT::Array< arr > positions; // index=time, inner array has 2 dims: 0=constant, 1=positions
  MT::Array< arr > angles; // index=time, inner array has 2 dims: 0=constant, 1=angles
  // potential non-logical additional info [end]
	
  ~Trial();
    
  void write(ostream& os, bool writeContinuousFeatures = false) const;
	void writeNice(ostream& os) const;
	void read(istream& is);
};



struct Experience {
  TL::State* pre, *post;
  TL::PredicateInstance* action;
  
  MT::Array< TL::PredicateInstance* > del, add;
  uintA changedConstants;
  
  Experience(TL::State* pre, TL::PredicateInstance* action, TL::State* post);
  ~Experience();
  
  bool noChange();
  
  void writeNice(ostream& os = cout);
};

typedef MT::Array< Experience* > ExperienceA;




}


typedef MT::Array< TL::Predicate* > PredA;
typedef MT::Array< TL::Function* > FuncA;
typedef MT::Array< TL::FunctionValue* > FuncVA;
typedef MT::Array< TL::FunctionInstance* > FuncIA;
typedef MT::Array< TL::PredicateInstance* > PredIA;
typedef MT::Array< TL::ComparisonPredicateInstance* > CompPredIA;
typedef MT::Array< TL::State* > StateA;
typedef MT::Array< TL::TermType* > TermTypeA;
typedef MT::Array< TL::Experience* > ExperienceA;




namespace TL {

// ---------------------------------------------------------------
//    HELPER FUNCTIONS
// ---------------------------------------------------------------

void sort(PredA& preds);
void sort(FuncA& funcs);
bool equivalent(const PredIA& p1, const PredIA& p2);
bool equivalent(const FuncVA& fv1, const FuncVA& fv2);
inline bool compare(int a, int b, uint compType) {
  bool holds;
  switch(compType) {
    case TL_COMPARISON_EQUAL: holds = a==b; break;
    case TL_COMPARISON_LESS: holds = a<b; break;
    case TL_COMPARISON_LESS_EQUAL: holds = a<=b; break;
    case TL_COMPARISON_GREATER: holds = a>b; break;
    case TL_COMPARISON_GREATER_EQUAL: holds = a>=b; break;
    default: HALT("Undefined comparison")
  }
  return holds;
}
inline bool compare(double a, double b, uint compType) {
  bool holds;
  switch(compType) {
    case TL_COMPARISON_EQUAL: holds = areEqual(a,b); break;
    case TL_COMPARISON_LESS: holds = a<b; break;
    case TL_COMPARISON_LESS_EQUAL: holds = a<=b; break;
    case TL_COMPARISON_GREATER: holds = a>b; break;
    case TL_COMPARISON_GREATER_EQUAL: holds = a>=b; break;
    default: HALT("Undefined comparison")
  }
  return holds;
}
void writeComparison(uint comparisonType, std::ostream& os);
// calcs concepts on which p_derived concepts are based
void baseConcepts(TL::Predicate* p_p_derived, MT::Array< TL::Predicate* >& p_base, MT::Array< TL::Function* >& f_base);
void baseConcepts(TL::Function* f_p_derived, MT::Array< TL::Predicate* >& p_base, MT::Array< TL::Function* >& f_base);
  
// Rewards
double calcActionRewards(const PredIA& actions);
double calcActionRewards(const PredIA& actions, const arr& discount_pow);


// ---------------------------------------------------------------
//    WRITING   OF LISTS
// ---------------------------------------------------------------

void writeNice(const PredA&, ostream& os = cout);
void writeNice(const FuncA&, ostream& os = cout);
void writeNice(const PredIA& pis, ostream& os = cout);
void writeNice(const FuncVA& fvs, ostream& os = cout);
void writeNice(const FuncIA& fis, ostream& os = cout);
void writeNice(const MT::Array< PredIA >& outcomes, ostream& os = cout);


// ---------------------------------------------------------------
//    READING  &  WRITING   OF COMPLEX OBJECTS
// ---------------------------------------------------------------

// Trials
// read
TL::Trial* readTrial(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, bool readContinuousFeatures);
// helpers
TL::State* readState(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const FuncA& f_prim, const FuncA& f_derived);
// eats whole line
TL::FunctionValue* readFunctionValue(ifstream& in, const FuncA& f_prim, const FuncA& f_derived);
// eats whole line
TL::FunctionInstance* readFunctionInstance(ifstream& in, const FuncA& f_prim, const FuncA& f_derived);
// eats whole line
TL::PredicateInstance* readPredicateInstance(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const FuncA& f_prim, const FuncA& f_derived);
// eats whole line
TL::PredicateInstance* readAction(ifstream& in, const PredA& actions);
    
// RULES
// write
void writeRules(const RuleSet& rules, const char* filename);
// read
void readRules(const char* filename, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules);
void readRules(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules);
TL::Rule* readRule(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived);
// read plain format
void readRules_plainFormat(const char* filename, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules);
void readRules_plainFormat(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived, RuleSet& rules);
TL::Rule* readRule_plainFormat(ifstream& in, const PredA& p_prim, const PredA& p_derived, const PredA& p_comp, const PredA& actions, const FuncA& f_prim, const FuncA& f_derived);

// LANGUAGE
// read
void readLanguage(const char *filename, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types);
void readLanguage(ifstream& in, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types);
void readLanguageSimpleFormat(const char *filename, PredA& p_prim, PredA& p_derived, PredA& p_comp, PredA& actions, FuncA& f_prim, FuncA& f_derived, TermTypeA& types);
// write
void writeLanguage(const PredA& preds, const FuncA& funcs, const PredA& actions, TermTypeA& types, const char* filename);
void writeLanguage(const PredA& preds, const FuncA& funcs, const PredA& actions, TermTypeA& types, ostream& out);
// helpers
// eats whole line
TL::Predicate* readPredicate(ifstream& in, uintA& baseIds);
// eats whole line
TL::Function* readFunction(ifstream& in, uintA& baseIds);
// eats whole line
TL::TermType* readTermType(ifstream& in, const TermTypeA& existing_types);


} // TL namespace



std::ostream& operator<<(std::ostream& os, const TL::Predicate& p);
std::ostream& operator<<(std::ostream& os, const TL::Function& f);
std::ostream& operator<<(std::ostream& os, const TL::State& s);
std::ostream& operator<<(std::ostream& os, const TL::Trial& s);
std::ostream& operator<<(std::ostream& os, const TL::PredicateInstance& s);
std::ostream& operator<<(std::ostream& os, const TL::FunctionValue& s);
std::ostream& operator<<(std::ostream& os, const TL::Rule& s);

#endif // TL__RELATIONAL_LOGIC
