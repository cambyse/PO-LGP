#ifndef MT_robot_variables_h
#define MT_robot_variables_h

#include "array.h"
#include "process.h"

//===========================================================================
//
// basic data structures and forward declarations
//

//fwd declarations

namespace ors{  struct Proxy;  }

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




//===========================================================================
//
// Variables
//

/*!\brief q_state_Variable: the current state of all joints of a robot and how these
  DoFs map to motor indices and hand indices */
struct q_currentReferenceVar:public Variable{
  arr q_reference,v_reference;
  arr q_real;

  uintA armMotorIndices,handMotorIndices;
  bool readHandFromReal;

  q_currentReferenceVar():Variable("q_state"){ readHandFromReal=false; }
};

struct SkinPressureVar:public Variable{
  arr y_real; //6D sensor reading

  SkinPressureVar():Variable("skinPressure"){
    y_real.resize(6);
    y_real.setZero();
  }
};

/*! The list of current proxies (=near-to-collisions) */
struct currentProxiesVar:public Variable{
  MT::Array<ors::Proxy*> proxies;
  
  currentProxiesVar():Variable("proxies"){}
};

/*! The current camera images */
struct CameraImages:public Variable{
  byteA rgbL, rgbR;
  CameraImages():Variable("camera_images"){}
  void loadDummyImages(){ read_ppm(rgbL,"left.ppm");  read_ppm(rgbR,"right.ppm"); }
};

/*! The hsv output of early vision */
struct EarlyVisionOutput:public Variable{
  floatA hsvThetaL, hsvThetaR;
  
  EarlyVisionOutput():Variable("EarlyVisionOutput"){}
};

/*! The Object List output of perception */
struct PerceptionOutput:public Variable{
  MT::Array<Object> objects;
  byteA disp;
  
  PerceptionOutput():Variable("PerceptionOutput"){};
};

/*! The output of a motion planner */
struct FutureMotionPlan:public Variable{
  bool converged, executed;
  arr q,x,bwdMsg_v,bwdMsg_Vinv;
  double tau,totalTime,cost,ctrlTime;
  
  FutureMotionPlan():Variable("FutureMotionPlan"){ converged=executed=false; ctrlTime=0.; }
  void write(ostream& os){ os <<"FutureMotionPlan converged= " <<converged <<" cost= " <<cost <<" ctrlTime= " <<ctrlTime; }
};

/*! The definition of the motion problem */
struct FutureMotionGoal:public Variable{
  enum GoalType { noGoalT=0, graspGoalT, placeGoalT, homingGoalT };
  GoalType goalType;
  //bool graspGoalAvailable,placeGoalAvailable;
  const char *graspShape,*belowFromShape,*belowToShape;
  
 FutureMotionGoal():Variable("FutureMotionGoal"){ goalType=noGoalT; }
  //graspGoalAvailable=placeGoalAvailable=false; }
};

struct GraspObject;
struct GraspObjectVar:public Variable{
  GraspObject *o;
  GraspObject *prior;//?remove

  GraspObjectVar():Variable("grasp object"){ o=NULL; prior=NULL;};
};


#endif
