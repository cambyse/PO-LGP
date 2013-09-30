#ifndef MT_robot_h
#define MT_robot_h

//#undef _FORTIFY_SOURCE
//#include <cstdlib>
#include <Gui/opengl.h>

#include <Ors/ors.h>
#include <MT/joystick.h>
#include <MT/soc.h>
#include <NJ/UrgInterface.h>
#include <NP/camera.h>
#include <MT/schunk.h>
#include <MT/vision.h>
#include <MT/earlyVisionModule.h>
#include <MT/guiModule.h>

struct ControllerProcess;
//struct RobotActionInterfaceWS;
struct RevelInterface;
enum CtrlMode { stopCM, joystickCM, reachCM, followTrajCM, closeHandCM, openHandCM };


//===========================================================================
//
// helpers
//

void q_hand_home(arr &);


//===========================================================================
//
// Task Abstraction
//

/** A collection of TaskVariables and their target/precision information */
struct TaskAbstraction{
  TaskVariableList TVall;
  
  TaskVariable *TV_eff, *TV_rot, *TV_col, *TV_lim, *TV_q, *TV_skin; //vars for control
  TaskVariable *TV_up , *TV_up2, *TV_z1, *TV_z2, *TV_f1, *TV_f2, *TV_f3;    //vars for grasping
  double TV_x_yprec, TV_x_vprec, TV_rot_vprec, TV_q_vprec;
  
  //control options
  uint controlMode;

  //-- options/parameters
  double joyRate;      //joystick speed
  arr reachPoint;      //defines the 3D reach point in reachCM

  //trajectory messages - output buffers for planners
  double plan_count, plan_speed;
  uint plan_scale;
  arr plan_v, plan_b, plan_Vinv;

  TaskAbstraction();

  virtual void initTaskVariables(ControllerProcess*);
  virtual void updateTaskVariables(ControllerProcess*);
};


//===========================================================================
//
// Robot Controller Module
//

/** given a task (=TaskVariable configuration) computed the desired next joint state of the robot */
struct ControllerProcess{ //--non-threaded!!
  //INPUT
  TaskAbstraction *task;
  bool useBwdMsg, forceColLimTVs;
  arr bwdMsg_v, bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  intA joyState;
  arr skinState;

  //OUTPUT
  arr q_reference, v_reference; //, q_orsInit;  //the SIMULATION state (the modules buffer double states, simulation is synchronized with modules in the loop)
  arr q_home; //posture as loaded from the ors file
  
  //INTERNAL
  ors::Graph ors;
  SwiftInterface swift;
  OrsSystem sys;
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

/** simply a collection of standard robot modules: open() opens them all,
    close() closes them all, step() communicates between them and steps them all */
       
struct RobotProcessGroup{
  //modules
  bool openArm, openHand, openSkin, openJoystick, openLaser, openBumble, openEarlyVision, openGui, openThreadInfoWin;
  ControllerProcess ctrl;
  SchunkArmModule arm;
  SchunkHandModule hand;
  SchunkSkinModule skin;
  JoystickInterface joy;
  UrgInterface urg;
  EarlyVisionModule evis;
  BumblebeeModule bumble;
  GuiModule gui;

  //ticcer -- for strictly timed stepping
  Metronome ticcer;
  CycleTimer timer;
  
  //internal: communication ControllerProcess <-> Schunk
  uintA motorIndex;          //association between ors-joints and schunk-motors
  
  //IMPORTANT: call signal(SIGINT, RobotProcessGroup::signalStopCallback); in main.cpp
  static bool signalStop;
  static void signalStopCallback(int);
  
  //step/loop information
  uint stepCounter;

  //logFile and stuff [mostly obsolete so far]
  RevelInterface *revel;
  ostream *log;

  RobotProcessGroup();
  ~RobotProcessGroup();

  //main routines
  void open();
  void step();
  void close();
};


#ifdef  MT_IMPLEMENTATION
#  include "robot.cpp"
#endif

#endif
