#ifndef SD_potentialTaskVariables_h
#define SD_potentialTaskVariables_h

#include "MT/ors.h"

struct PotentialField;
typedef MT::Array<ors::Shape*> ShapeList;

/* for n shapes describes the n-dim vector of values of a potential */
struct PotentialValuesTaskVariable:public TaskVariable{
  PotentialField *f;
  ShapeList refs;
  
  PotentialValuesTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs, PotentialField& _f);
  virtual void userUpdate();
};

struct PotentialFieldAlignTaskVariable:public TaskVariable{
  PotentialField *f;
  ShapeList refs;

  PotentialFieldAlignTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs, PotentialField& _f);
  virtual void userUpdate();
};

#ifdef  MT_IMPLEMENTATION
#include "potentialTaskVariables.cpp"
#endif

#endif
