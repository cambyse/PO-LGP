/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifndef MT_specialTaskVariables_h
#define MT_specialTaskVariables_h

#include <Ors/ors.h>
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
