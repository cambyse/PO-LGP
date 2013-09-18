#ifndef SD_miscTaskVariables_h
#define SD_miscTaskVariables_h

#include <Ors/ors.h>

typedef MT::Array<ors::Shape*> ShapeList;

struct zOpposeTaskVariable:public DefaultTaskVariable{
  ShapeList refs;

  zOpposeTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs);
  virtual TaskVariable* newClone(){ return new zOpposeTaskVariable(*this); }
  virtual void userUpdate(const ors::Graph& ors);
};

struct zFocusTargetTaskVariable:public DefaultTaskVariable{
  ShapeList refs;
  arr target;

  zFocusTargetTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs);
  virtual TaskVariable* newClone(){ return new zFocusTargetTaskVariable(*this); }
  virtual void userUpdate(const ors::Graph& ors);
};


#ifdef  MT_IMPLEMENTATION
#include "miscTaskVariables.cpp"
#endif

#endif
