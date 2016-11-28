#ifndef SD_miscTaskVariables_h
#define SD_miscTaskVariables_h

#include <Ors/ors.h>

typedef mlr::Array<mlr::Shape*> ShapeList;

struct zOpposeTaskVariable:public DefaultTaskVariable{
  ShapeList refs;

  zOpposeTaskVariable(const char* _name, mlr::KinematicWorld& _ors, ShapeList& _refs);
  virtual TaskVariable* newClone(){ return new zOpposeTaskVariable(*this); }
  virtual void userUpdate(const mlr::KinematicWorld& ors);
};

struct zFocusTargetTaskVariable:public DefaultTaskVariable{
  ShapeList refs;
  arr target;

  zFocusTargetTaskVariable(const char* _name, mlr::KinematicWorld& _ors, ShapeList& _refs);
  virtual TaskVariable* newClone(){ return new zFocusTargetTaskVariable(*this); }
  virtual void userUpdate(const mlr::KinematicWorld& ors);
};


#ifdef  MLR_IMPLEMENTATION
#include "miscTaskVariables.cpp"
#endif

#endif
