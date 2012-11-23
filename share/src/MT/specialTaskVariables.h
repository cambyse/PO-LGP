#ifndef MT_specialTaskVariables_h
#define MT_specialTaskVariables_h

#include "ors.h"
#include "socNew.h"
#include "soc_orsSystem.h"

//===========================================================================
//
// types
//

typedef MT::Array<ors::Shape*> ShapeList;

//===========================================================================
//
// specific helpers to create standard sets of task variables and goals
//

void createStandardRobotTaskVariables(OrsSystem& sys);
void setGraspGoals(OrsSystem& sys, uint T, uint shapeId);
void setGraspGoals(OrsSystem& sys, uint T, const char* objShape);
void setPlaceGoals(OrsSystem& sys, uint T, const char* objShape, const char* belowFromShape, const char* belowToShape);
void setHomingGoals(OrsSystem& sys, uint T, const char* objShape, const char* belowToShape);


#ifdef  MT_IMPLEMENTATION
#include "specialTaskVariables.cpp"
#endif

#endif
