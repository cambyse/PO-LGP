#ifndef MT_robotActionInterface_h
#define MT_robotActionInterface_h

#include "ors.h"
#include "perceptionModule.h"
#include "motionPlannerModule.h"


//===========================================================================
//
// Robot Action Inferface
//

struct TaskAbstraction;
struct RobotModuleGroup;

//private space:
struct sRobotActionInterface;

struct RobotActionInterface{
  sRobotActionInterface *s;
  RobotActionInterface();
  ~RobotActionInterface();

  void open();
  void close();
  
  void joystick();
  void wait(double sec=0);
  void homing();
  void reach(const char* shapeName,const arr& posGoal,double maxVel=.1);
  void reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel=.1);
  void setMesh(const char* shapeName,const ors::Mesh& mesh);

  // -- planned motions
  void perceiveObjects(PerceptionModule& perc);
  void pickObject(ReceedingHorizonProcess& planner, const char* objShape);
  void placeObject(ReceedingHorizonProcess& planner, const char* objShape,const char* belowFromShape,const char* belowToShape);
  void plannedHoming(ReceedingHorizonProcess& planner,const char* objShape,const char* belowToShape);
  void graspISF();
  
  RobotModuleGroup* getProcessGroup();
  TaskAbstraction* getTask();
};

#ifdef  MT_IMPLEMENTATION
#  include "robotActionInterface.cpp"
#endif

#endif
