#ifndef MT_robot_h
#define MT_robot_h

//#undef _FORTIFY_SOURCE
//#include <cstdlib>
#include "opengl.h"

#include "robot_variables.h"

#include "ors.h"
#include "joystick.h"
#include "soc.h"
#include <NJ/UrgModule.h>
#include <NP/camera.h>
#include "schunk.h"
#include "vision.h"
#include "earlyVisionModule.h"
#include "guiModule.h"
#include "process.h"

struct ControllerProcess; //RENAME: ControllerProcess
//struct RobotActionInterfaceWS;
struct RevelInterface;
struct TaskAbstraction;
enum CtrlMode { stopCM, joystickCM, reachCM, followTrajCM, closeHandCM, openHandCM, homingCM, functionCM, prefixedCM };

struct TaskGoalUpdater{
  virtual void operator()(TaskAbstraction*,ControllerProcess*) = 0;
};

//===========================================================================
//
// helpers
//

void q_hand_home(arr &);



//===========================================================================
//
// Task Abstraction
//

/*! A collection of TaskVariables and their target/precision information */
struct TaskAbstraction{
  JoystickInterface *joyVar;
  FutureMotionPlan *planVar;

  TaskVariableList TVall;
  
  TaskVariable *TV_eff,*TV_rot,*TV_col,*TV_lim,*TV_q,*TV_skin; //vars for control
  TaskVariable *TV_up ,*TV_up2,*TV_z1,*TV_z2,*TV_f1,*TV_f2,*TV_f3;    //vars for grasping
  double TV_x_yprec,TV_x_vprec,TV_rot_vprec,TV_q_vprec;
  
  //control options
  uint controlMode; //RENAME taskMode
  
  //-- options/parameters
  double joyRate;      //joystick speed
  intA joyState;
  arr reachPoint;      //defines the 3D reach point in reachCM

  //trajectory messages - output buffers for planners
  double plan_count,plan_speed;
  uint plan_scale;
  arr plan_v,plan_b,plan_Vinv;

  TaskGoalUpdater *taskGoalUpdater;
  Lock taskGoalUpdaterLock;
  
  TaskAbstraction();

  virtual void initTaskVariables(ControllerProcess*);
  virtual void updateTaskVariables(ControllerProcess*); //RENAME  updateTaskGoals
};


//===========================================================================
//
// Robot Controller Module
//

/*! given a task (=TaskVariable configuration) computed the desired next joint state of the robot */
struct ControllerProcess:public Process{ //--non-threaded!!
  q_currentReferenceVar *q_referenceVar;
  currentProxiesVar *proxiesVar;
  SchunkSkinModule *skinVar;
  
  //INPUT
  TaskAbstraction *task;
  Lock taskLock; //lock this if you change the task!!
  bool useBwdMsg, forceColLimTVs, fixFingers;
  arr bwdMsg_v,bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  arr skinState;

  //OUTPUT
  arr q_reference,v_reference; //,q_orsInit;  //the SIMULATION state (the modules buffer real states, simulation is synchronized with modules in the loop)
  arr q_home; //posture as loaded from the ors file
  
  //INTERNAL
  ors::Graph ors;
  SwiftInterface swift;
  soc::SocSystem_Ors sys;
  CycleTimer timer;
  
  ControllerProcess();

  //main routines
  void open();
  void step();
  void close();
};

//===========================================================================
//
// Robot Controller
//

/*! simply a collection of standard robot processes: open() opens them all,
    close() closes them all, step() communicates between them and steps them all */
       
struct RobotModuleGroup{
  //Variables
  q_currentReferenceVar q_currentReference;
  currentProxiesVar currentProxies;

  //Processes
  bool openArm,openHand,openSkin,openJoystick,openLaser,openBumble,openEarlyVision,openGui,openThreadInfoWin;
  ControllerProcess ctrl;
  SchunkArmModule arm;
  SchunkHandModule hand;
  SchunkSkinModule skin;

  JoystickInterface joy;
  UrgModule urg;
  EarlyVisionModule evis;
  BumblebeeModule bumble;
  GuiModule gui;
  ThreadInfoWin threadWin;

  //ticcer -- for strictly timed stepping
  Metronome ticcer;
  CycleTimer timer;
  
  //internal: communication ControllerProcess <-> Schunk
  uintA motorIndex;          //association between ors-joints and schunk-motors
  
  //IMPORTANT: call signal(SIGINT,RobotModuleGroup::signalStopCallback); in main.cpp
  static bool signalStop;
  static void signalStopCallback(int);
  
  //step/loop information
  uint stepCounter;

  //logFile and stuff [mostly obsolete so far]
  RevelInterface *revel;
  ostream *log;

  RobotModuleGroup();
  ~RobotModuleGroup();

  //main routines
  void open();
  void step();
  void close();
};


#ifdef MT_IMPLEMENTATION
#  include "robot.cpp"
#endif

#endif
