#ifndef RELATIONAL_symbolGrounding_h
#define RELATIONAL_symbolGrounding_h

#include "symbols.h"
#include "literals.h"
#include <Kin/kin.h>




namespace relational {
   

struct ContinuousState;
  
// ------------------------------------------------------------------
//  GroundedSymbol


class GroundedSymbol {
public:
  
  enum GroundingType {NN, RBF, AL};
  
  mlr::String name;
  uint arity;
  Symbol* symbol;
  GroundingType type;

  GroundedSymbol(mlr::String& name, uint arity, bool build_derived_predicates = false);
  
  // calculates whether symbol holds given the data x
  virtual bool holds(arr& x) const = 0;
  virtual void calculateSymbols(LitL& lits, const uintA& objects_ids, const mlr::Array< arr > & objects_data) const;
  virtual void write() const;
  
  static void calculateSymbols(LitL& lits, const mlr::Array<GroundedSymbol*>& sgs, const ContinuousState& const_state);
};


class NN_Grounding : public GroundedSymbol {
public:
  arr w1a;
  arr w1b;
  arr w2a;
  arr w2b;
  
  NN_Grounding(mlr::String& name, uint arity, bool build_derived_predicates = false);
  
  bool holds(arr& x) const;
  
  void write() const;
  void read(const char* file_w1a, const char* file_w1b, const char* file_w2a, const char* file_w2b);
};  


class RBF_Grounding : public GroundedSymbol { 
public:
  arr w_c;
  arr w_sigma;
  
  RBF_Grounding(mlr::String& name, uint arity, bool build_derived_predicates = false);

  bool holds(arr& x) const;
  
  void write() const;
  void read(const char* file_w_c, const char* file_w_sigma);
};


void read(mlr::Array<GroundedSymbol*>& pns, const char* prefix, GroundedSymbol::GroundingType grounding_type);
  
  
// mlr::KinematicWorld interface
void getFeatureVector(arr& f, const mlr::KinematicWorld& C, uint obj);
void getFeatureVectors(mlr::Array< arr >& fs, const mlr::KinematicWorld& C, const uintA& objs);
void calculateSymbols(LitL& lits, const mlr::Array<GroundedSymbol*>& sgs, mlr::KinematicWorld* C);


}

typedef mlr::Array< relational::GroundedSymbol* > SGL;


namespace relational {
  

struct ContinuousState {
  uintA object_ids;
  mlr::Array< arr > data;
  
  void write(ostream& out) const;
  void read(istream& in);
  
  bool operator==(const ContinuousState& other) const;
  bool operator!=(const ContinuousState& other) const;
};


ContinuousState* getContinuousState(const mlr::KinematicWorld& C, const uintA& objects);



struct FullExperience {
  enum ActionType {grab, puton};
  
  ContinuousState state_continuous_pre;
  ContinuousState state_continuous_post;
  
  ActionType action_type;
  uintA action_args;
  
  double reward;
  
  StateTransition experience_symbolic;
  
  void write_continuous_nice(ostream& out) const;
  void write_continuous(ostream& out) const;
  void write_symbolic(ostream& out) const;
  static void write_continuous(mlr::Array<FullExperience* > exps, ostream& out);
  static void write_symbolic(mlr::Array<FullExperience* > exps, ostream& out);
  static FullExperience* read_continuous(ifstream& in);
  static void read_continuous(mlr::Array< FullExperience* >& experiences, const char* file_name);
  static void read_continuous_nikolayFormat(mlr::Array< FullExperience* >& experiences, const char* file_name);
  static void sanityCheck(mlr::Array< FullExperience* >& experiences);
};

void calculateSymbols(const mlr::Array<GroundedSymbol*>& sgs, FullExperience& e);

}

typedef mlr::Array< relational::FullExperience* > FullExperienceL;

#endif // RELATIONAL_symbolGrounding_h
