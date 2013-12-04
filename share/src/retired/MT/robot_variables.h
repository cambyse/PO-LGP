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


#ifndef MT_robot_variables_h
#define MT_robot_variables_h

#include <Core/array.h>
#include <system/module.h>
#include <Ors/ors.h>

//===========================================================================
//
// basic data structures and forward declarations
//

struct Object {
  uint found;
  
  //-- 2d shape
  uint shapeType;
  arr shapeParamsL, shapeParamsR, shapePointsL, shapePointsR;
  
  //-- 3d information
  arr shapePoints3d;
  arr center3d, orsShapeParams;
  arr diagDiff;
  
  Object(){ found=0; }
};

typedef MT::Array<Object*> ObjectList;




//===========================================================================
//
// Variables
//

/** @brief q_state_Variable: the current state of all joints of a robot and how these
  DoFs map to motor indices and hand indices */
struct q_currentReferenceVar:public Variable {
  arr q_reference, v_reference;
  arr q_real;
  
  uintA armMotorIndices, handMotorIndices;
  bool readHandFromReal;
  
  q_currentReferenceVar():Variable("q_state"){ readHandFromReal=false; }
};

struct CurrentSceneInformation:public Variable {
  ors::KinematicWorld ors;

  CurrentSceneInformation():Variable("current scnene information"){}
};

struct SkinPressureVar:public Variable {
  arr y_real; //6D sensor reading
  
  SkinPressureVar():Variable("skinPressure"){
    y_real.resize(6);
    y_real.setZero();
  }
};

/** The list of current proxies (=near-to-collisions) */
struct currentProxiesVar:public Variable {
  MT::Array<ors::Proxy*> proxies;
  
  currentProxiesVar():Variable("proxies"){}
};

/** The current camera images */
struct CameraImages:public Variable {
  byteA rgbL, rgbR;
  CameraImages():Variable("camera_images"){}
  void loadDummyImages(){ read_ppm(rgbL, "left.ppm");  read_ppm(rgbR, "right.ppm"); }
};

/** The hsv output of early vision */
struct EarlyVisionOutput:public Variable {
  floatA hsvThetaL, hsvThetaR;
  
  EarlyVisionOutput():Variable("EarlyVisionOutput"){}
};

/** The Object List output of perception */
struct PerceptionOutput:public Variable {
  MT::Array<Object> objects;
  byteA disp;
  
  PerceptionOutput():Variable("PerceptionOutput"){};
};

/** The output of a motion planner */
struct FutureMotionPlan:public Variable {
  bool converged, executed;
  arr q, x, bwdMsg_v, bwdMsg_Vinv;
  double tau, totalTime, cost, ctrlTime;
  
  FutureMotionPlan():Variable("FutureMotionPlan"){ converged=executed=false; ctrlTime=0.; }
  void write(ostream& os){ os <<"FutureMotionPlan converged= " <<converged <<" cost= " <<cost <<" ctrlTime= " <<ctrlTime; }
};

/** The definition of the motion problem */
struct FutureMotionGoal:public Variable {
  enum GoalType { noGoalT=0, graspGoalT, placeGoalT, homingGoalT };
  GoalType goalType;
  //bool graspGoalAvailable, placeGoalAvailable;
  const char *graspShape, *belowFromShape, *belowToShape;
  
  FutureMotionGoal():Variable("FutureMotionGoal"){ goalType=noGoalT; }
  //graspGoalAvailable=placeGoalAvailable=false; }
};

struct GraspObject;
struct GraspObjectVar:public Variable {
  GraspObject *o;
  GraspObject *prior;//?remove
  
  GraspObjectVar():Variable("grasp object"){ o=NULL; prior=NULL;};
};


#endif
