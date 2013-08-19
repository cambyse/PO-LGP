#ifndef SD_potentialTaskVariables_h
#define SD_potentialTaskVariables_h

#include <Ors/ors.h>

struct PotentialField;
struct GraspObject_GP;
typedef MT::Array<ors::Shape*> ShapeList;

/* for n shapes describes the n-dim vector of values of a potential */
struct PotentialValuesTaskVariable:public DefaultTaskVariable{
  PotentialField *f;
  ShapeList refs;
  
  PotentialValuesTaskVariable(const char* _name, ors::Graph& _ors, const ShapeList& _refs, PotentialField& _f);
  virtual TaskVariable* newClone(){ return new PotentialValuesTaskVariable(*this); }
  virtual void userUpdate(const ors::Graph& ors);
};

struct PotentialFieldAlignTaskVariable:public DefaultTaskVariable{
  PotentialField *f;
  ShapeList refs;

  PotentialFieldAlignTaskVariable(const char* _name, ors::Graph& _ors, const ShapeList& _refs, PotentialField& _f);
  virtual TaskVariable* newClone(){ return new PotentialFieldAlignTaskVariable(*this); }
  virtual void userUpdate(const ors::Graph& ors);
};

struct GPVarianceTaskVariable:public DefaultTaskVariable{
  GraspObject_GP *f;
  ShapeList refs;

  GPVarianceTaskVariable(const char* _name, ors::Graph& _ors, const ShapeList& _refs, GraspObject_GP& _f);
  virtual TaskVariable* newClone(){ return new GPVarianceTaskVariable(*this); }
  virtual void userUpdate(const ors::Graph& ors);
};

#ifdef  MT_IMPLEMENTATION
#include "potentialTaskVariables.cpp"
#endif

#endif
