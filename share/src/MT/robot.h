#ifndef MT_robot_h
#define MT_robot_h

#include "opengl.h"

#include "robot_variables.h"

#include "ors.h"
#include "joystick.h"
#include "socSystem_ors.h"
#include <NJ/UrgInterface.h>
#include "schunk.h"
#include "vision.h"
#include "earlyVisionModule.h"
#include "guiModule.h"
#include <biros/biros.h>
#include <biros/biros_internal.h>
#include <NP/camera.h>
#include <NP/uvccamera.h>

struct ControllerProcess;
struct TaskAbstraction;
enum CtrlMode { stopCM, joystickCM, reachCM, followTrajCM, closeHandCM, openHandCM, homingCM, functionCM, prefixedCM };

struct TaskGoalUpdater {
  virtual void operator()(TaskAbstraction*, ControllerProcess*) = 0;
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
struct TaskAbstraction {
  JoystickInterface *joyVar;
  FutureMotionPlan *planVar;
  
  TaskVariableList TVall;
  
  TaskVariable *TV_eff, *TV_rot, *TV_col, *TV_lim, *TV_q, *TV_skin; //vars for control
  TaskVariable *TV_up , *TV_up2, *TV_z1, *TV_z2, *TV_f1, *TV_f2, *TV_f3;    //vars for grasping
  double TV_x_yprec, TV_x_vprec, TV_rot_vprec, TV_q_vprec;
  
  //-- options/parameters
  double joyRate;      //joystick speed
  intA joyState;
  arr reachPoint;      //defines the 3D reach point in reachCM
  
  //trajectory messages - output buffers for planners
  double plan_count, plan_speed;
  uint plan_scale;
  arr plan_v, plan_b, plan_Vinv;
  
  TaskAbstraction();
  
  virtual void initTaskVariables(ControllerProcess*);
  virtual void updateTaskGoals(ControllerProcess*); //RENAME  updateTaskGoals
  
  // helper
  void prepare_skin(ControllerProcess*, bool);
};


//===========================================================================
//
// Robot Controller Module
//

/*! given a task (=TaskVariable configuration) computed the desired next joint state of the robot */
struct ControllerProcess:public Process { //--non-threaded!!
  q_currentReferenceVar *q_referenceVar;
  SkinPressureVar *skinPressureVar;
  currentProxiesVar *proxiesVar;
  FutureMotionPlan *planVar;
  JoystickInterface *joyVar;
 
  //INPUT
  TaskAbstraction *task;
  Lock taskLock; //lock this if you change the task!!
  bool useBwdMsg, forceColLimTVs, fixFingers;
  arr bwdMsg_v, bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  arr skinState;
  
  //OUTPUT
  arr q_reference, v_reference; //, q_orsInit;  //the SIMULATION state (the modules buffer double states, simulation is synchronized with modules in the loop)
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
  
  TaskAbstraction *change_task(TaskAbstraction *task);
};

//===========================================================================
//
// Robot Controller
//

/*! simply a collection of standard robot processes: open() opens them all,
    close() closes them all, step() communicates between them and steps them all */

struct RobotProcessGroup {
  //Variables
  q_currentReferenceVar q_currentReference;
  SkinPressureVar skinPressureVar;
  currentProxiesVar currentProxies;
  CameraImages currentCameraImages;
  PerceptionOutput percOutput;
	EarlyVisionOutput evisOutput;

  //Processes
  bool openArm, openHand, openSkin, openJoystick, openLaser, openBumble, openEarlyVision, openGui, openThreadInfoWin;
  ControllerProcess ctrl;
  SchunkArmModule arm;
  SchunkHandModule hand;
  SchunkSkinModule skin;
  
  JoystickInterface joy;
  UrgInterface urg;
  EarlyVisionModule evis;
  CameraModule bumble;
  GuiModule gui;
  ThreadInfoWin threadWin;
  
  //internal: communication ControllerProcess <-> Schunk
  uintA motorIndex;          //association between ors-joints and schunk-motors
  
  //IMPORTANT: call signal(SIGINT, RobotProcessGroup::signalStopCallback); in main.cpp
  static bool signalStop;
  static void signalStopCallback(int);
  
  //step/loop information
  //uint stepCounter;
  
  RobotProcessGroup();
  ~RobotProcessGroup();
  
  //main routines
  void open();
  void close();
};

#define _BasicRobotTask(BasicRobotTaskName)   \
  \
  struct BasicRobotTaskName:public TaskAbstraction{ \
    virtual void initTaskVariables(ControllerProcess *ctrl){ \
      TaskAbstraction::initTaskVariables(ctrl); \
    }; \
    virtual void updateTaskGoals(ControllerProcess*); \
    static BasicRobotTaskName *p; \
    static BasicRobotTaskName *a(){if(!p) p=new BasicRobotTaskName(); return p;}; \
  };
// end template

_BasicRobotTask(DoNothing)
_BasicRobotTask(Stop)
_BasicRobotTask(Homing)
_BasicRobotTask(OpenHand)
_BasicRobotTask(CloseHand)
_BasicRobotTask(Reach)
_BasicRobotTask(FollowTrajectory)
_BasicRobotTask(Joystick)

#ifdef  MT_IMPLEMENTATION
#  include "robot.cpp"
#endif

#endif
