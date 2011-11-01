/*  
    Copyright 2011   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
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

#ifndef TL__LOGIC_DEFINITIONS
#define TL__LOGIC_DEFINITIONS

#include <map>
#include <MT/util.h>
#include <TL/utilTL.h>

#define FOR1D_(x,i)   for(i=0;i<x.num();i++)
#define FOR1D_DOWN_(x,i)  for(i=x.num();i--;)



namespace TL {

// "Action" of default rule: "null" with reserved ID
const uint DEFAULT_ACTION_PRED__ID = 0;
const double DEFAULT_FUNCTION_VALUE = -1000.;

  
enum ComparisonType { comparison_equal, comparison_less, comparison_lessEqual, comparison_greater, comparison_greaterEqual };
enum ConceptCategory { category_primitive, category_derived }; 

  
/****************************************
     TYPES
 ***************************************/

class TermType {
  protected:
    uint typeI;
  public:
    enum TermType_Type { term_type_simple, term_type_disjunction }; 
    
    const static uint ANY_id = 0;
    
    uint type_id;
    MT::String name;
    
    virtual bool operator==(const TermType& t) const;
    virtual bool operator!=(const TermType& t) const {
      return !(*this == t);
    }
    virtual bool subsumes(const TermType& t) const;
    
    TermType() {typeI = term_type_simple;}
    ~TermType() {}
    
    virtual void writeNice(ostream& os = cout) const;
    virtual void write_deprecated(ostream& os) const;
};


struct DisjunctionTermType : public TermType {
  public:
    MT::Array<TermType*> base_types;
  
    bool subsumes(const TermType& t) const;
    
    DisjunctionTermType() {typeI = term_type_disjunction;}
    ~DisjunctionTermType() {}
    
    void writeNice(ostream& os = cout) const;
    void write_deprecated(ostream& os) const;
};





/****************************************
     PREDICATES
 ***************************************/

class Function;

// ---------------------------------------
//  Primitives

class Predicate {
  public:
    enum PredicateType { predicate_action, predicate_simple, predicate_comparison, predicate_conjunction, predicate_transClosure, predicate_count };

    uint id; // each predicate has individual ID  (0 is reserved for NULL-action of default rule)
    MT::String name;
    uint d; // arity
    PredicateType type; // simple, comparison, conjunction...
    uint category; // primitive or p_derived
    MT::Array< TermType* > arg_types;
  
    Predicate() {
      type = predicate_simple;
      category = category_primitive;
    }
    virtual ~Predicate() {}

    virtual bool operator==(const Predicate& p) const;  // not via id!!
    virtual bool operator!=(const Predicate& p) const {
      return !(*this == p);
    }
  
    virtual void writeNice(ostream& os = cout) const;
    virtual void write(ostream& os) const;
};




class ComparisonPredicate : public Predicate {
  public:
    bool constantBound;
    ComparisonPredicate() {
      constantBound = true;
      type = predicate_comparison;
    }
    void write(ostream& os) const;
};



// ---------------------------------------
//  Derived predicates

class DerivedPredicate : public Predicate {
  public:
    DerivedPredicate() {
          category = category_derived;
      }
      virtual ~DerivedPredicate() {}
      
      virtual DerivedPredicate* newClone() = 0;
};


class ConjunctionPredicate : public DerivedPredicate {
  public:
    ConjunctionPredicate () {
      type = predicate_conjunction;
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
    
    ConjunctionPredicate* newClone();
};



// assume d=2 thus far!
class TransClosurePredicate : public DerivedPredicate {
  public:
    TL::Predicate* basePred;
    
    TransClosurePredicate () {
        type = predicate_transClosure;
    }
    ~TransClosurePredicate () {
    }
    void writeNice(ostream& os = cout) const;
    void write(ostream& os) const;
    
    TransClosurePredicate* newClone();
};


class CountPredicate : public DerivedPredicate {
  public:
    CountPredicate () {
        type = predicate_count;
        countedPred_isComparison = false;
    }
    ~CountPredicate () {
    }
    void writeNice(ostream& os) const;
    void write(ostream& os) const;
    CountPredicate* newClone();
    
    TL::Predicate* countedPred;
    uintA countedPred_mapVars2derived; // d0 = countedPred.d !!
    uint bound;
    ComparisonType compType;
    
    // special fields in case of using comparison predicate
    bool countedPred_isComparison;
    Function* comparison_f;
    ComparisonType comparison_compType;
    double comparison_bound;
};



// --------------------------------------------
//  Predicates with arguments and truth values

class Atom {
  public:
    Predicate* pred;
    uintA args;
    bool operator==(Atom& a) const;
    bool operator!=(Atom& a) const;
    virtual ~Atom() {}
        
    void name(MT::String& name) const;
        
    virtual void write_deprecated(ostream& os) const;
    virtual void write(ostream& os = cout, bool withTypes = false) const;
    void read(istream& is);
};


class FunctionAtom;
class ComparisonAtom : public Atom {
  public:
    // slot assignments are handled by Literal
    FunctionAtom* fa1;
    FunctionAtom* fa2;
    double bound;
    ComparisonType comparisonType;
    bool operator==(Atom& a) const;
    
    ComparisonAtom() {
      fa1 = NULL;
      fa2 = NULL;
    }
    
    void write_deprecated(ostream& os) const;
    void write(ostream& os = cout, bool withTypes = false) const;
        
    inline bool hasConstantBound() const {
      return ((ComparisonPredicate*)this->pred)->constantBound;
    }
};



class Literal {
  public:
    Atom* atom;
    bool positive;
    
    virtual ~Literal() {}
    bool operator==(Literal& lit) const;
    bool operator!=(Literal& lit) const;
        
    void name(MT::String& name) const;
        
    virtual void write_deprecated(ostream& os) const;
    virtual void write(ostream& os = cout, bool withTypes = false) const;
    void read(istream& is);
};


class ComparisonLiteral : public Literal {
  public:
    inline bool hasConstantBound() {return ((ComparisonAtom*) atom)->hasConstantBound();}
};






/****************************************
     FUNCTIONS
 ***************************************/


class Function {
  public:
    enum FunctionType { function_simple, function_count, function_avg, function_max, function_change, function_sum, function_reward };
    
    uint id;
    MT::String name;
    uint d; // arity
    FunctionType type; // simple, count, ...
    ConceptCategory category; // primitive or derived
    uintA range;
    bool real_range; // range in R (in this case, we don't need uintA range)
  
    Function() {
      type = function_simple;
      category = category_primitive;
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


// ---------------------------------------
//  Derived functions

class CountFunction : public Function {
  public:
    CountFunction() {
        type = function_count;
        category = category_derived;
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
      type = function_avg;
      category = category_derived;
      real_range = true;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};


class SumFunction : public Function {
  public:
    SumFunction() {
      type = function_sum;
      category = category_derived;
      real_range = false;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};



class MaxFunction : public Function {
  public:
    MaxFunction() {
      type = function_max;
      category = category_derived;
      real_range = false;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;
        
    Function* f_base;
};


class ChangeFunction : public Function {
  public:
    ChangeFunction() {
      type = function_change;
      category = category_derived;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;

    Function* f_base;
};


class Literal;
class RewardFunction : public Function {
  public:
    RewardFunction() {
      type = function_reward;
      category = category_derived;
    }
        
    void writeNice(ostream& os = std::cout) const;
    void write(ostream& os) const;

    double reward_value;
    MT::Array< TL::Literal* > grounded_pis;
};



// ---------------------------------------
//  Functions with arguments and values


struct FunctionAtom {
  TL::Function* f;
  uintA args;
  void write_deprecated(ostream& os) const;
  void write(ostream& os = cout) const;
  
  bool operator==(const FunctionAtom& fa) const;
  bool operator!=(const FunctionAtom& fa) const;
};



class FunctionValue {
  public:
    FunctionAtom* atom;
    double value;
    bool operator==(FunctionValue& fv) const;
    bool operator!=(FunctionValue& fv) const;
    
    void write_deprecated(ostream& os) const;
    void write(ostream& os = cout) const;
};






/****************************************
     RULE
 ***************************************/

struct Rule {
  MT::Array< TL::Literal* > context;
  TL::Atom* action;
  MT::Array< MT::Array< TL::Literal* > > outcomes;
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

  void write_deprecated(ostream& os) const;
  void write(ostream& os = std::cout, bool withAction = true) const;
  
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
  
  void write(ostream& os = std::cout);
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
      
  // ATTENTION:  If you want to use logicReasoning, then use "applyOriginalSub"-methods there!
  //  (These use the singleton objects for literals and atoms maintained by the logicObjectManager.)
  void apply(const MT::Array< TL::Literal* >& unsub_lits, MT::Array< TL::Literal* >& subPreds);
  void apply(const MT::Array< TL::FunctionValue* >& unsubFVs, MT::Array< TL::FunctionValue* >& subFVs);
  Atom* apply(Atom* unsub_lit);
  Literal* apply(Literal* unsub_lit);
  FunctionAtom* apply(FunctionAtom* unsub_fa);
  FunctionValue* apply(FunctionValue* unsub_fv);
  TL::State* apply(const TL::State& state);
  uint getSubs(uint in);
  void getIns(uintA& ids) const;
  void getOuts(uintA& ids) const;
  void getOutsDistinct(uintA& ids) const;
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
      
  void write_deprecated(ostream& os) const;
  void write(ostream& os = cout);
      
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
  
  void write(ostream& os = cout);
};




/****************************************
     STATE & TRIAL & EXPERIENCE
 ***************************************/

struct State {
  MT::Array<Literal*> lits_prim;
  MT::Array<Literal*> lits_derived;
  MT::Array<FunctionValue*> fv_prim;
  MT::Array<FunctionValue*> fv_derived;
  bool derivedDerived; // Have derived concepts been calculated?
    
  State() {
    derivedDerived = false;
  }
  ~State();
	
  bool operator==(const State& lit) const;
  bool operator!=(const State& lit) const;
	
  void write_deprecated(ostream& os) const;
  void write(ostream& os = std::cout, bool primOnly = false) const;
};

struct Trial {
  uintA constants;
  MT::Array<State*> states;
  MT::Array<Atom*> actions;
  
  // potential non-logical additional info [start]
  MT::Array< arr > positions; // index=time, inner array has 2 dims: 0=constant, 1=positions
  MT::Array< arr > angles; // index=time, inner array has 2 dims: 0=constant, 1=angles
  // potential non-logical additional info [end]
	
  ~Trial();
    
  // TODO umbenennen
//   void write_deprecated(ostream& os, bool writeContinuousFeatures = false) const;
  void writeNice(ostream& os) const;
  void write(const char* filename) const;
};



struct Experience {
  TL::State pre, post;
  TL::Atom* action;
  
  MT::Array< TL::Literal* > del, add;
  uintA changedConstants;
  
  Experience(const TL::State& pre, TL::Atom* action, const TL::State& post);
  ~Experience();
  
  bool noChange();
  
  void write(ostream& os = cout) const;
};

typedef MT::Array< Experience* > ExperienceA;




}



std::ostream& operator<<(std::ostream& os, const TL::Predicate& p);
std::ostream& operator<<(std::ostream& os, const TL::Function& f);
std::ostream& operator<<(std::ostream& os, const TL::State& s);
std::ostream& operator<<(std::ostream& os, const TL::Trial& s);
std::ostream& operator<<(std::ostream& os, const TL::Atom& s);
std::ostream& operator<<(std::ostream& os, const TL::Literal& s);
std::ostream& operator<<(std::ostream& os, const TL::FunctionAtom& fa);
std::ostream& operator<<(std::ostream& os, const TL::FunctionValue& fv);
std::ostream& operator<<(std::ostream& os, const TL::Rule& r);
std::ostream& operator<<(std::ostream& os, const TL::Experience& e);
std::ostream& operator<<(std::ostream& os, const MT::Array< TL::Literal* > & lits);
std::ostream& operator<<(std::ostream& os, const MT::Array< TL::Atom* > & atoms);


typedef MT::Array< TL::Predicate* > PredL;
typedef MT::Array< TL::Function* > FuncL;
typedef MT::Array< TL::FunctionValue* > FuncVL;
typedef MT::Array< TL::FunctionAtom* > FuncAL;
typedef MT::Array< TL::Atom* > AtomL;
typedef MT::Array< TL::Literal* > LitL;
typedef MT::Array< TL::ComparisonAtom* > CompAtomL;
typedef MT::Array< TL::ComparisonLiteral* > CompLitL;
typedef MT::Array< TL::State* > StateL;
typedef MT::Array< TL::TermType* > TermTypeL;
typedef MT::Array< TL::Experience* > ExperienceL;




namespace TL {
  
  
// ---------------------------------------------------------------
//    WRITING
// ---------------------------------------------------------------

void writeNice(const PredL&, ostream& os = cout);
void writeNice(const FuncL&, ostream& os = cout);
void write(const AtomL& atoms, ostream& os = cout);
void write(const LitL& lits, ostream& os = cout);
void write(const FuncVL& fvs, ostream& os = cout);
void write(const FuncAL& fis, ostream& os = cout);
void write(const ExperienceL& exs, ostream& os = cout);
void write(const MT::Array< LitL >& list_of_lits_list, ostream& os = cout);
void write(const RuleSet& rules, ostream& os = cout);
void write(const RuleSet& rules, const char* filename);
// language
void writeLanguage(const PredL& preds, const FuncL& funcs, const PredL& actions, TermTypeL& types, const char* filename);
void writeLanguage(const PredL& preds, const FuncL& funcs, const PredL& actions, TermTypeL& types, ostream& out);
  
  
  

// ---------------------------------------------------------------
//    HELPER FUNCTIONS
// ---------------------------------------------------------------

void sort(PredL& preds);
void sort(FuncL& funcs);
bool equivalent(const LitL& lits1, const LitL& lits2);
bool equivalent(const FuncVL& fv1, const FuncVL& fv2);
// calcs concepts on which derived concepts are based
void baseConcepts(TL::Predicate* p_derived, MT::Array< TL::Predicate* >& p_base, MT::Array< TL::Function* >& f_base);
void baseConcepts(TL::Function* f_derived, MT::Array< TL::Predicate* >& p_base, MT::Array< TL::Function* >& f_base);

inline bool compare(int a, int b, ComparisonType compType) {
  bool holds;
  switch(compType) {
    case comparison_equal: holds = a==b; break;
    case comparison_less: holds = a<b; break;
    case comparison_lessEqual: holds = a<=b; break;
    case comparison_greater: holds = a>b; break;
    case comparison_greaterEqual: holds = a>=b; break;
    default: HALT("Undefined comparison")
  }
  return holds;
}
inline bool compare(double a, double b, ComparisonType compType) {
  bool holds;
  switch(compType) {
    case comparison_equal: holds = areEqual(a,b); break;
    case comparison_less: holds = a<b; break;
    case comparison_lessEqual: holds = a<=b; break;
    case comparison_greater: holds = a>b; break;
    case comparison_greaterEqual: holds = a>=b; break;
    default: HALT("Undefined comparison")
  }
  return holds;
}
void writeComparison(ComparisonType comparisonType, std::ostream& os);

} // TL namespace

#endif // TL__LOGIC_DEFINITIONS
