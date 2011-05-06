#ifndef MT_specialTaskVariables_h
#define MT_specialTaskVariables_h

#include "ors.h"
#include "soc.h"
#include "SD/ISF_GP.h"
#include "graspObjects.h"

//===========================================================================
//
// types
//

typedef MT::Array<ors::Shape*> ShapeList;

//===========================================================================
//
// specific helpers to create standard sets of task variables and goals
//

void createStandardRobotTaskVariables(soc::SocSystem_Ors& sys);
void setGraspGoals(soc::SocSystem_Ors& sys,uint T,uint shapeId);
void setGraspGoals(soc::SocSystem_Ors& sys,uint T,const char* objShape);
void setPlaceGoals(soc::SocSystem_Ors& sys,uint T,const char* objShape,const char* belowFromShape,const char* belowToShape);
void setHomingGoals(soc::SocSystem_Ors& sys,uint T,const char* objShape,const char* belowToShape);

//===========================================================================
//
// novel, more complex task variables
//

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



#ifdef MT_IMPLEMENTATION
#include "specialTaskVariables.cpp"
#endif

#endif
