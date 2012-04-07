#ifndef MT_specialTaskVariables_h
#define MT_specialTaskVariables_h

#include "ors.h"
#include "soc.h"
#include "socSystem_ors.h"

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
void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId);
void setGraspGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape);
/*void setPlaceGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape, const char* belowFromShape, const char* belowToShape);*/
void setHomingGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape, const char* belowToShape);


#ifdef  MT_IMPLEMENTATION
#include "specialTaskVariables.cpp"
#endif

#endif
