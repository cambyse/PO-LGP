#ifndef TL__SYMBOL_GROUNDING
#define TL__SYMBOL_GROUNDING

#include <relational/logicDefinitions.h>
#include <MT/ors.h>




namespace relational {
  
  
  
// ------------------------------------------------------------------
//  SymbolGrounding


class SymbolGrounding {
public:
  
  enum GroundingType {NN, RBF};
  
  MT::String name;
  uint arity;
  TL::Predicate* pred;
  GroundingType type;

  SymbolGrounding(MT::String& name, uint arity, bool build_derived_predicates = false);
  
  // calculates whether symbol holds given the data x
  virtual bool holds(arr& x) const = 0;
  virtual void calculateLiterals(LitL& lits, const uintA& objects_ids, const MT::Array< arr > & objects_data) const;
  virtual void write() const;
  
  static void calculateLiterals(LitL& lits, const MT::Array<SymbolGrounding*>& sgs, const uintA& objects_ids, const MT::Array< arr > & objects_data);
};


class NN_Grounding : public SymbolGrounding {
public:
  arr w1a;
  arr w1b;
  arr w2a;
  arr w2b;
  
  NN_Grounding(MT::String& name, uint arity, bool build_derived_predicates = false);
  
  bool holds(arr& x) const;
  
  void write() const;
  void read(const char* file_w1a, const char* file_w1b, const char* file_w2a, const char* file_w2b);
};  


class RBF_Grounding : public SymbolGrounding { 
public:
  arr w_c;
  arr w_sigma;
  
  RBF_Grounding(MT::String& name, uint arity, bool build_derived_predicates = false);

  bool holds(arr& x) const;
  
  void write() const;
  void read(const char* file_w_c, const char* file_w_sigma);
};


void read(MT::Array<SymbolGrounding*>& pns, const char* prefix, SymbolGrounding::GroundingType grounding_type);
  
  
// ors::Graph interface
void getFeatureVector(arr& f, const ors::Graph& C, uint obj);
void getFeatureVectors(MT::Array< arr >& fs, const ors::Graph& C, const uintA& objs);
void calculateLiterals(LitL& lits, const MT::Array<SymbolGrounding*>& sgs, ors::Graph* C);


}

typedef MT::Array< relational::SymbolGrounding* > SGL;


namespace relational {

// ------------------------------------------------------------------
//  Nikolays Format -- Experiences
  
// state: 20
// 5 Objekte, 5. Objekt ist die Roboterhand
// (1-3): x,y,z-Koords
// (4): Groesse

// successor state: 20
// genauso

// action: 2

// (2): 

// reward: 1

// #define LOGIC_CONSTANTS_START 61
#define LOGIC_CONSTANTS_START 64
inline uint buildConstant(uint a, uint bound = LOGIC_CONSTANTS_START) {return a + bound;}  // meine Daten: erstes Objekt = table
inline uintA buildConstant(uintA& a, uint bound = LOGIC_CONSTANTS_START) {return a + bound;}

struct FullExperience {
  enum ActionType {grab, puton};
  
  MT::Array< arr > state_continuous_pre;
  MT::Array< arr > state_continuous_post;
  
  ActionType action_type;
  uintA action_args;
  
  double reward;
  
  TL::Experience experience_symbolic;
  
  void write_continuous_nice(ostream& out) const;
  void write_continuous(ostream& out) const;
  void write_symbolic(ostream& out) const;
  static void write_continuous(MT::Array<FullExperience* > exps, ostream& out);
  static void write_symbolic(MT::Array<FullExperience* > exps, ostream& out);
  static FullExperience* read_continuous(ifstream& in);
  static void read_continuous(MT::Array< FullExperience* >& experiences, const char* file_name);
  static void read_continuous_nikolayFormat(MT::Array< FullExperience* >& experiences, const char* file_name);
  static void sanityCheck(MT::Array< FullExperience* >& experiences);
};

void calculateLiterals(const MT::Array<SymbolGrounding*>& sgs, FullExperience& e);

}

typedef MT::Array< relational::FullExperience* > FullExperienceL;

#endif // TL__SYMBOL_GROUNDING
