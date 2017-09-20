#pragma once

#include <memory>
#include <bits/shared_ptr.h>

#include <Core/array.h>
#include <Kin/kin.h>

#include "act.h"
#include "act_CtrlTask.h"
#include "act_PathOpt.h"
#include "act_PathFollow.h"
#include "act_TaskController.h"
#include "act_ComPR2.h"
#include "act_ComBaxter.h"
#include "act_Thread.h"
#include "act_Tweets.h"
#include "act_Script.h"
#include "act_Event.h"
#include "act_AtEvent.h"
#include "act_Recorder.h"
#include "act_Perception.h"
#include "script_PickAndPlace.h"

template<class T> Signaler* operator-(std::shared_ptr<T>& p){ return dynamic_cast<Signaler*>(p.get()); }
template<class T> SignalerL operator+(std::shared_ptr<T>& p){ return ARRAY<Signaler*>(dynamic_cast<Signaler*>(p.get())); }
template<class T, class S> SignalerL operator+(std::shared_ptr<T>& a, std::shared_ptr<S>& b){ return ARRAY<Signaler*>(dynamic_cast<Signaler*>(a.get()), dynamic_cast<Signaler*>(b.get())); }
template<class T> SignalerL& operator+(SignalerL& A, std::shared_ptr<T>& p){ A.append(dynamic_cast<Signaler*>(p.get())); return A; }

//==============================================================================

struct Roopi {
  struct Roopi_private* s;

  Access<ActL> acts;

  Roopi(bool autoStartup=false, bool controlView=true);
  ~Roopi();

  //==============================================================================
  //
  //-- CONTROL FLOW: This is robotics independent; generic methods to schedule, trigger scripts, etc

  void report();

  /** wait until the status of each act in the signaler set is non-zero (zero usually means 'still running')
      after this, you can query the status and make decisions based on that */
  bool wait(const SignalerL& acts, double timeout=-1.);
  bool wait(); ///< keyboard/click input
  void wait(double seconds); ///< given real time (devided by hyperSpeed)
  bool waitEvent(const Act::Ptr& event, double timeout=-1.); ///< wait for event

  /** An event function takes a list of signalers as input and, depending on their status (or variable value),
   * returns an int. The Act_Event changes its status to this int */
  Act::Ptr event(SignalerL sigs, const EventFunction& eventFct);

  /** this takes an int-valued function (use a lambda expression to capture scope) and
      runs it in a thread as activity - when done, the activity broadcasts its status equal to the int-return-value */
  Act::Ptr run(const std::function<int()>& script);

  Act::Ptr loop(double beatIntervalSec, const std::function<int()>& script);

  /// run a script once when the event becomes true, that is, the script is 'uploaded' and triggered later
  Act::Ptr at(const Act_Event::Ptr& event, const std::function<int ()>& script);

  /// run a script whenever the event becomes true. This is like a permanent transition rule in RAP
  Act::Ptr whenever(const Act_Event::Ptr& event, const std::function<int()>& script);

  //==============================================================================
  //
  //-- INITIALIZATION

  void setKinematics(const char* filename, bool controlView=true);          ///< set kinematics by hand (done in 'autoStartup')
  void setKinematics(const mlr::KinematicWorld& K, bool controlView=true);  ///< set kinematics by hand (done in 'autoStartup')
  shared_ptr<Act_TaskController> startTaskController();         ///< start the task controller by hand (done in 'autoStartup')
  Act::Ptr startTweets(bool go=true);             ///< start the status tweeter by hand (done in 'autoStartup')
  shared_ptr<Act_Thread> startRos();         ///< start ROS by hand (done in 'autoStartup')
  shared_ptr<Act_ComPR2> startPR2();         ///< start PR2 coms by hand (done in 'autoStartup')
  shared_ptr<Act_ComBaxter> startBaxter();         ///< start Baxter coms by hand

  //==============================================================================

  //-- INFORMATION: access to variables; standard config things

  bool useRos();
  const mlr::String& getRobot();                     ///< returns "pr2", "baxter", or "none"
  arr get_q0();                                      ///< return the 'homing pose' of the robot
  Act_TaskController& getTaskController();           ///< get taskController (to call verbose, or lock..)
  Act_ComPR2& getComPR2();                           ///< to call 'stopSendingMotion'
  Act_ComBaxter& getComBaxter();

  //-- direct access to variables and threads
  template<class T> Access<T> variable(const char* var_name){    ///< get a handle to a typed variable: you can R/W-access the data or read/wait for the revision
    return Access<T>(NULL, var_name, false); }
  VariableBase::Ptr variableStatus(const char* var_name);
  Thread* threadStatus(const char* thread_name);
//  template<class T> VariableData<T>& variable(const char* var_name){ return getVariable<T>(var_name);}
  template<class T> T* thread(const char* thread_name){ return getThread<T>(thread_name); }
  void reportCycleTimes();

  //==============================================================================
  //
  //-- KINEMATIC EDITING (to be done more...)

  mlr::Shape* newMarker(const char* name, const arr& pos);        ///< adds a shape to the model world
  void kinematicSwitch(const char* object, const char* attachTo, bool placing); ///< switches kinematics in the model world
  WToken<mlr::KinematicWorld> setK();                             ///< get write access to the model world
  RToken<mlr::KinematicWorld> getK();                             ///< get read access to the model world

  //==============================================================================
  //
  //-- MOTION CONTROL

  Act_CtrlTask::Ptr newCtrlTask(){ return Act_CtrlTask::Ptr(new Act_CtrlTask(this)); }  ///< set the CtrlTask yourself (see newHoldingTask as example)
  Act_CtrlTask::Ptr newCtrlTask(TaskMap *map, const arr& PD={1.,.9}, const arr& target={0.}, const arr& prec={1.});
  Act_CtrlTask::Ptr newCtrlTask(const char* specs);
  // predefined
  Act_CtrlTask::Ptr home();
  Act_CtrlTask::Ptr lookAt(const char* shapeName, double prec=1e-2, const char* endeff_name=NULL);
  Act_CtrlTask::Ptr focusWorkspaceAt(const char* shapeName, double prec=2e-1, const char* endeff_name=NULL);
  Act_CtrlTask::Ptr moveVel(const char* endeff_name, arr velocity);
  Act_CtrlTask::Ptr newHoldingTask();
  Act_CtrlTask::Ptr newCollisionAvoidance();
  Act_CtrlTask::Ptr newLimitAvoidance();
  // persistent
  void hold(bool still);
  Act_CtrlTask::Ptr collisions(bool on);
  void deactivateCollisions(const char* s1, const char* s2);

  //==============================================================================
  //
  //-- VARIOUS ACTIVITIES

  Act_Thread::Ptr  newThread(Thread* th)  { return Act_Thread::Ptr(new Act_Thread(this, th)); } ///< a trivial wrapper to make a thread (create it with new YourThreadClass) an activity
  Act_ComPR2::Ptr  newComPR2()            { return Act_ComPR2::Ptr(new Act_ComPR2(this)); } ///< subscribers/publishers that communicate with PR2
  Act_PathOpt::Ptr newPathOpt()           { return Act_PathOpt::Ptr(new Act_PathOpt(this)); } ///< a path optimization activity, access komo yourself to define the problem
  Act_ComBaxter::Ptr newComBaxter()       { return Act_ComBaxter::Ptr(new Act_ComBaxter(this));}

  Act_Th<struct RosCom_Spinner> RosCom(); ///< thread for the ROS spinner
  Act_Thread::Ptr PhysX();           ///< run PhysX (nvidia physical simulator)
  Act_Thread::Ptr GamepadControl();  ///< activate gamepad to set controls
  Act_Thread::Ptr CameraView(const char* modelWorld_name="modelWorld");      ///< compute and display the camera view
  Act_PclPipeline PclPipeline(bool view=false);
  Act_PerceptionFilter PerceptionFilter(bool view=false);

  //==============================================================================
  //
  // MACROS which call scripts

  Act::Ptr graspBox(const char* objName, LeftOrRight lr){
    return run( [this, objName, lr](){ return Script_graspBox(*this, objName, lr); } );
  }
  Act::Ptr pointBox(const char* objName, LeftOrRight lr){
    return run( [this, objName, lr](){ return Script_pointBox(*this, objName, lr); } );
  }
  Act::Ptr place(const char* objName, const char* ontoName){
    return run( [this, objName, ontoName](){ return Script_place(*this, objName, ontoName); } );
  }
  Act::Ptr placeDistDir(const char* objName, const char* ontoName, double deltaX=0., double deltaY=0., double deltaZ=0., int deltaTheta=0){
    return run( [this, objName, ontoName, deltaX, deltaY, deltaZ, deltaTheta](){ return Script_placeDistDir(*this, objName, ontoName, deltaX, deltaY, deltaZ, deltaTheta); } );
  }
  Act::Ptr pointPosition(const char* objName, const char* ontoName, LeftOrRight lr, double deltaX=0., double deltaY=0., double deltaZ=0.){
    return run( [this, objName, ontoName, lr, deltaX, deltaY, deltaZ](){ return Script_pointPosition(*this, objName, ontoName, lr, deltaX, deltaY, deltaZ); } );
  }
  Act::Ptr setGripper(LeftOrRight lr, double gripSize){
    return run( [this, lr, gripSize](){ return Script_setGripper(*this, lr, gripSize); } );
  }
  Act::Ptr workspaceReady(const char* objName){
    return run( [this, objName](){ return Script_workspaceReady(*this, objName); } );
  }
  Act::Ptr armsNeutral(){
    return run( [this](){ return Script_armsNeutral(*this); } );
  }
  Act::Ptr setTorso(int up_down){
    return run( [this, up_down](){ return Script_setTorso(*this,up_down); } );
  }

};
