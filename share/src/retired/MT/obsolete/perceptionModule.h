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
#ifndef MLR_perceptionModule_h
#define MLR_perceptionModule_h

#include "vision.h"
#include <Ors/ors.h>
#include <System/biros.h>

#include "robot_variables.h"

struct PerceptionModule:public Process {
  EarlyVisionOutput *input;
  PerceptionOutput *output;
  
  //INPUT
  arr objectType;
  
  //PARAMETERS for camera projection 3d<->2d
  arr Pl, Pr;
  //AverageTrack avTrack;
  mlr::Array<Object> objs;
  
  PerceptionModule():Process("PerceptionModule"){ input=NULL; }
  
  void open();
  void step();
  void close(){};
};

void realizeObjectsInOrs(mlr::KinematicWorld& ors, const mlr::Array<Object>& objects);

//void copyShapeInfos(mlr::KinematicWorld& A, const mlr::KinematicWorld& B);
void copyBodyInfos(mlr::KinematicWorld& A, const mlr::KinematicWorld& B);

#ifdef  MLR_IMPLEMENTATION
#  include "perceptionModule.cpp"
#endif

#endif
