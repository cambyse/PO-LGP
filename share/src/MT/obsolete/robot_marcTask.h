#ifndef MT_robot_marc_h
#define MT_robot_marc_h

#include "robot.h"

//===========================================================================
//
// Trivial bwd msg task
//

struct TrivialBwdMsgTask:public TaskAbstraction {
  FutureMotionPlan *planVar;
  
  TrivialBwdMsgTask();
  virtual void updateTaskVariables(ControllerProcess*); //overloading the virtual
};


//===========================================================================
//
// helper
//

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody, const char* belowShape);


//===========================================================================
//
// Marc's Robot Task
//

struct MarcsRobotTask:public RobotProcessGroup, public TaskAbstraction { //one could argue if this should be private...
  MarcsRobotTask();
  ~MarcsRobotTask();
  
  //plan moves
  void planGraspTrajectory(const char* objShape);
  void planPlaceTrajectory(const char* objShape, const char* belowFromShape, const char* belowToShape);
  void followTrajectory();
  void closeHand(const char* objShape, const char* belowShape);
  void openHand(const char* objShape);
  
  void reactivateCollisions(const MT::Array<const char*>& shapes);
  void reactivateCollisions(const MT::Array<ors::Shape*>& shapes);
  void loadTrajectory(const char* filename="z.plan");
  void loadPlainTrajectory(const char* filename="z.plan");
  
  //interactive
  void watch();
  void watchTrajectory();
  void joystick();
  void waitJoyClean();
  
  //vision
  arr objectPosition;
  void localizeObject(const char* identifier);
  void reachObject();
  void positionObjectRandomlyInSimulation();
};

#ifdef  MT_IMPLEMENTATION
#  include "robot_marcTask.cpp"
#endif

#endif
