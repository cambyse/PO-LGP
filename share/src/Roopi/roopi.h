#pragma once

#include <Core/array.h>
#include <Kin/kin.h>

#include "act.h"
#include "act_CtrlTask.h"
#include "act_PathOpt.h"
#include "act_PathFollow.h"
#include "act_TaskController.h"
#include "act_ComRos.h"
#include "act_ComPR2.h"
#include "act_Thread.h"
#include "act_Tweets.h"
#include "act_Script.h"
#include "act_Recorder.h"
#include "script_PickAndPlace.h"


//==============================================================================

struct Roopi {
  struct Roopi_private* s;

  Access_typed<ActL> acts;

  Roopi(bool autoStartup=false);
  ~Roopi();

  //-- initialization (start... means persistent activities)
  void setKinematics(const char* filename);          ///< set kinematics by hand (done in 'autoStartup')
  void setKinematics(const mlr::KinematicWorld& K);  ///< set kinematics by hand (done in 'autoStartup')
  Act_TaskController& startTaskController();         ///< start the task controller by hand (done in 'autoStartup')
  Act_Tweets& startTweets(bool go=true);             ///< start the status tweeter by hand (done in 'autoStartup')

  //-- control flow
  /** wait until the status of each act in the set if non-zero (zero usually means 'still running')
      after this, you can query the status and make decisions based on that */
  bool wait(std::initializer_list<Act*> acts, double timeout=-1.);

  /** this takes an int-valued function (use a lambda expression to capture scope) and
      run it in a thread as activity - when done, the activity broadcasts its status equal to the int-return-value */
  Act_Script runScript(const std::function<int()>& script);

  //TODO: runScriptOnEvent(const std::function<int()>& script, Event, bool whenever=false);
  //TODO: define the notion of an Event as a set of act and conditions of their status -> wait(Event)

  //-- get some information
  const mlr::String& getRobot();                     ///< returns "pr2", "baxter", or "none"
  arr get_q0();                                      ///< return the 'homing pose' of the robot
  Act_TaskController& getTaskController();           ///< get taskController (to call verbose, or lock..)

  //-- kinematic editing (to be done more..)
  mlr::Shape* newMarker(const char* name, const arr& pos);        ///< adds a shape to the model world
  void kinematicSwitch(const char* object, const char* attachTo); ///< switches kinematics in the model world
  WToken<mlr::KinematicWorld> setK();                             ///< get write access to the model world
  RToken<mlr::KinematicWorld> getK();                             ///< get read access to the model world
  void resyncView();

  //-- control

  Act_CtrlTask newCtrlTask()         { return Act_CtrlTask(this); }
  Act_CtrlTask newCtrlTask(TaskMap *map, const arr& PD={1.,.9}, const arr& target={0.}, const arr& prec={1.});
  Act_CtrlTask newCtrlTask(const char* specs);
  void hold(bool still);
  Act_CtrlTask home();
  Act_CtrlTask lookAt(const char* shapeName);
  Act_CtrlTask* collisions(bool on);

  Act_CtrlTask newHoldingTask();
  Act_CtrlTask newCollisionAvoidance();
  Act_CtrlTask newLimitAvoidance();


  //-- some activities

  Act_Thread  newThread(Thread* th)  { return Act_Thread(this, th); } ///< a trivial wrapper to make a thread (create it with new YourThreadClass) an activity
  Act_Thread  newComROS()            { return Act_Thread(this, newRosComSpinner()); } ///< thread for the ROS spinner
  Act_ComPR2  newComPR2()            { return Act_ComPR2(this); } ///< subscribers/publishers that communicate with PR2
  Act_PathOpt newPathOpt()           { return Act_PathOpt(this); } ///< a path optimization activity, access komo yourself to define the problem

  Act_Thread newPhysX();           ///< run PhysX (nvidia physical simulator)
  Act_Thread newGamepadControl();  ///< activate gamepad to set controls
  Act_Thread newCameraView();      ///< compute and display the camera view

  //==============================================================================
  //
  // MACROS, which call scripts
  //

  Act_Script graspBox(const char* objName, LeftOrRight lr){
    return runScript( [this, objName, lr](){ return Script_graspBox(*this, objName, lr); } );
  }
  Act_Script place(const char* objName, const char* ontoName){
    return runScript( [this, objName, ontoName](){ return Script_place(*this, objName, ontoName); } );
  }
};

//==============================================================================


#if 0
  //-- control tasks

  /// creates a new CtrlTask; pass a 'newed' map as argument, it will be deleted later; after creation it is inactive
  CtrlTask *createCtrlTask(const char* name, TaskMap* map, bool active=false);

  /// activate a control task after you've set parameters, etc
  void activateCtrlTask(CtrlTask* t, bool reinitializeReferences = false);
  /// deactivate a control task
  void deactivateCtrlTask(CtrlTask* t);

  /// removes a CtrlTask and deletes also the task map
  void destroyCtrlTask(CtrlTask* t);

  /// modifies CtrlTasks
  void modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef = NoArr);

  /// holds all joints in position. Desactivates all other tasks
  void holdPosition();
  /// release hold position task. Has no effect on other tasks
  void releasePosition();

  /// returns whether a CtrlTask has converged
  bool converged(CtrlTask* ct, double tolerance = 1e-2);

  /// wait for all given CtrlTasks to be converged
  bool waitForConv(CtrlTask* ct, double maxTime = -1, double tolerance = 1e-2);
  bool waitForConv(const CtrlTaskL& cts, double maxTime = -1, double tolerance = 1e-2);

  /// wait for a CtrlTask to be at a specific state;
  bool waitForConvTo(CtrlTask* ct, const arr& desState, double maxTime = -1, double tolerance = 1e-2);

  // low-level ctr - use is discouraged!!
  struct TaskControlThread* tcm(); //low-level access of the tcm - really necessary? Danny: yes
//  void addCtrlTask(CtrlTask* ct); ///< adds CtrlTask directly to the taskController
//  void addCtrlTasks(CtrlTaskL cts); ///< adds multiple CtrlTasks directly to the taskController

  // shortcuts to crate standard CtrlTasks (mt) I'd remove this for now
  //  /// adds qItself task to the taskController
  //  CtrlTask* _addQItselfCtrlTask(const arr& qRef, const arr& Kp = ARR(30.0), const arr& Kd = ARR(5.0));

  //  /// creates inactive default task with zero gains and adds it to the taskController
  //  CtrlTask* _addDefaultCtrlTask(const char* name,
  //                               const TaskMap_DefaultType type,
  //                               const char* iShapeName, const mlr::Vector& iVec = NoVector,
  //                               const char* jShapeName = NULL, const mlr::Vector& jVec = NoVector);

  //-- trajectory tasks

  void interpolateToReferenceThread(CtrlTask *task, double executionTime, const arr &reference, const arr &start);

  TaskReferenceInterpolAct* createTaskReferenceInterpolAct(const char* name, CtrlTask* task);
  void interpolateToReference(TaskReferenceInterpolAct* t, double executionTime, const arr& reference, const arr& start = NoArr);
  bool waitForFinishedTaskReferenceInterpolAct(TaskReferenceInterpolAct* t, double maxTime = -1);

  void interpolateToReference(CtrlTask* task, double executionTime, const arr& reference, const arr& start = NoArr);

  void followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory);
  void followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot = {}, const arrA& trajYDDot = {});
  void followQTrajectory(const Roopi_Path* path);

  //-- planning tasks

  Roopi_Path* createPathInJointSpace(CtrlTask* task, double executionTime, bool verbose = false);
  Roopi_Path* createPathInJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose = false);

  //-- macros

  /// move "shape" to "pos" using a motion planner.
  bool goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose = false);

  /// move to joint configuration using a motion planner
  bool gotToJointConfiguration(const arr& jointConfig, double executionTime, bool verbose = false);

  //-- low-level access

  arr getJointState();
  arr getJointSign();
  arr getTorques();
  arr getFTLeft();
  arr getFTRight();

  /// get current value of the underlying task map
  arr getTaskValue(CtrlTask* task);

  /// sync the joint configuration of the model world into the planWorld
  void syncWorldWithReal(mlr::KinematicWorld& K);
  mlr::KinematicWorld& getPlanWorld();
  void copyModelWorld(mlr::KinematicWorld& K);

  double getLimitConstraint(double margin = 0.05);
  double getCollisionConstraint(double margin = 0.1);

  //-- TODO

  //getModelWorld() what return type?

//  //-- wait for & stop tasks
  void stop(const CtrlTaskL& tasks);
  void waitConv(const CtrlTaskL& tasks);
#endif


