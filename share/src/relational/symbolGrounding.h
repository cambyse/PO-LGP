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
void calculateLiterals(LitL& lits, const MT::Array<SymbolGrounding*>& sgs, ors::Graph* C);


}

typedef MT::Array< relational::SymbolGrounding* > SGL;


namespace relational {

// ------------------------------------------------------------------
//  Experiences
  
// state: 20
// 5 Objekte, 5. Objekt ist die Roboterhand
// (1-3): x,y,z-Koords
// (4): Groesse

// successor state: 20
// genauso

// action: 2
// (1): Aktionstyp grasp 1, puton 2
// (2): 

// reward: 1

inline uint buildConstant_nikolayData(uint a) {return a + 21;}

struct FullExperience {
  
  MT::Array< arr > state_continuous_pre;
  MT::Array< arr > state_continuous_post;
  
  uint action_type;
  uint action_target;
  
  double reward;
  
  TL::Experience experience_symbolic;
  
  void write_continuous(ostream& out);
  void write_symbolic(ostream& out);
  static void write_symbolic(MT::Array<FullExperience* > exps, ostream& out);
  static void read_nikolayFormat(MT::Array< FullExperience* >& experiences, const char* file_name);
};

void calculateLiterals(MT::Array<SymbolGrounding*>& pns, FullExperience& e);

}

#endif // TL__SYMBOL_GROUNDING
