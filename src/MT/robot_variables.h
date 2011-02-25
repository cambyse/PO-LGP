#ifndef MT_robot_variables_h
#define MT_robot_variables_h

#include "array.h"
#include "process.h"

//fwd declarations

namespace ors{  struct Proxy;  }


struct q_currentReferenceVar:public Variable{
  arr q_reference,v_reference;
  arr q_real;

  uintA armMotorIndices,handMotorIndices;
  bool readHandFromReal;

  q_currentReferenceVar():Variable("q_currentReference"){ readHandFromReal=false; }
};

struct currentProxiesVar:public Variable{
  MT::Array<ors::Proxy*> proxies;
  
  currentProxiesVar():Variable("currentProxies"){}
};

struct EarlyVisionOutput:public Variable{
  floatA hsvThetaL, hsvThetaR;
  
  EarlyVisionOutput():Variable("EarlyVisionOutput"){}
};

struct Object{
  uint found;
  
  //-- 2d shape
  uint shapeType;
  arr shapeParamsL,shapeParamsR,shapePointsL,shapePointsR;

  //-- 3d information
  arr shapePoints3d;
  arr center3d,orsShapeParams;
  arr diagDiff;

  Object(){ found=0; }
};

typedef MT::Array<Object*> ObjectList;

struct PerceptionOutput:public Variable{
  MT::Array<Object> objects;
  byteA disp;
  
  PerceptionOutput():Variable("PerceptionOutput"){};
};

struct FutureMotionPlan:public Variable{
  bool converged, executed;
  arr q,x,bwdMsg_v,bwdMsg_Vinv;
  double tau,totalTime,cost,ctrlTime;
  
  FutureMotionPlan():Variable("FutureMotionPlan"){ converged=executed=false; ctrlTime=0.; }
  void write(ostream& os){ os <<"FutureMotionPlan converged= " <<converged <<" cost= " <<cost <<" ctrlTime= " <<ctrlTime; }
};

enum GoalType { noGoalT=0, graspGoalT, placeGoalT, homingGoalT };

struct FutureMotionGoal:public Variable{
  GoalType goalType;
  //bool graspGoalAvailable,placeGoalAvailable;
  const char *graspShape,*belowFromShape,*belowToShape;
  
 FutureMotionGoal():Variable("FutureMotionGoal"){ goalType=noGoalT; }
  //graspGoalAvailable=placeGoalAvailable=false; }
};

#endif
