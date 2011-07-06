#ifndef SD_miscTaskVariables_h
#define SD_miscTaskVariables_h

#include "MT/ors.h"

typedef MT::Array<ors::Shape*> ShapeList;

struct zOpposeTaskVariable:public TaskVariable{
  ShapeList refs;

  zOpposeTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs);
  virtual void userUpdate();
};

struct zFocusTargetTaskVariable:public TaskVariable{
  ShapeList refs;
  arr target;

  zFocusTargetTaskVariable(const char* _name, ors::Graph& _ors, ShapeList& _refs);
  virtual void userUpdate();
};


#ifdef  MT_IMPLEMENTATION
#include "miscTaskVariables.cpp"
#endif

#endif
