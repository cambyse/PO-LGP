#ifndef RELATIONAL__SYMBOL_GROUNDING
#define RELATIONAL__SYMBOL_GROUNDING

#include <relational/symbols.h>
#include <relational/literals.h>
#include <MT/ors.h>




namespace relational {
  
using namespace PRADA;
  

struct ContinuousState;
  
// ------------------------------------------------------------------
//  GroundedSymbol


class GroundedSymbol {
public:
  
  enum GroundingType {NN, RBF, AL};
  
  MT::String name;
  uint arity;
  PRADA::Symbol* symbol;
  GroundingType type;

  GroundedSymbol(MT::String& name, uint arity, bool build_derived_predicates = false);
  
  // calculates whether symbol holds given the data x
  virtual bool holds(arr& x) const = 0;
  virtual void calculateSymbols(LitL& lits, const uintA& objects_ids, const MT::Array< arr > & objects_data) const;
  virtual void write() const;
  
  static void calculateSymbols(LitL& lits, const MT::Array<GroundedSymbol*>& sgs, const ContinuousState& const_state);
};


class NN_Grounding : public GroundedSymbol {
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


class RBF_Grounding : public GroundedSymbol { 
public:
  arr w_c;
  arr w_sigma;
  
  RBF_Grounding(MT::String& name, uint arity, bool build_derived_predicates = false);

  bool holds(arr& x) const;
  
  void write() const;
  void read(const char* file_w_c, const char* file_w_sigma);
};


void read(MT::Array<GroundedSymbol*>& pns, const char* prefix, GroundedSymbol::GroundingType grounding_type);
  
  
// ors::Graph interface
void getFeatureVector(arr& f, const ors::Graph& C, uint obj);
void getFeatureVectors(MT::Array< arr >& fs, const ors::Graph& C, const uintA& objs);
void calculateSymbols(LitL& lits, const MT::Array<GroundedSymbol*>& sgs, ors::Graph* C);


}

typedef MT::Array< relational::GroundedSymbol* > SGL;


namespace relational {
  

struct ContinuousState {
  uintA object_ids;
  MT::Array< arr > data;
  
  void write(ostream& out) const;
  void read(istream& in);
  
  bool operator==(const ContinuousState& other) const;
  bool operator!=(const ContinuousState& other) const;
};


ContinuousState* getContinuousState(const ors::Graph& C, const uintA& objects);



struct FullExperience {
  enum ActionType {grab, puton};
  
  ContinuousState state_continuous_pre;
  ContinuousState state_continuous_post;
  
  ActionType action_type;
  uintA action_args;
  
  double reward;
  
  PRADA::StateTransition experience_symbolic;
  
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

void calculateSymbols(const MT::Array<GroundedSymbol*>& sgs, FullExperience& e);

}

typedef MT::Array< relational::FullExperience* > FullExperienceL;

#endif // RELATIONAL__SYMBOL_GROUNDING
