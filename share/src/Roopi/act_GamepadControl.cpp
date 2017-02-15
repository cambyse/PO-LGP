#include "act_GamepadControl.h"
#include <Control/TaskControlThread.h>
#include <Control/gamepad2tasks.h>
#include <Hardware/gamepad/gamepad.h>
#include "roopi.h"

struct sAct_GamepadControl : Thread{
  struct Gamepad2Tasks *g2t;
  ACCESS(arr, gamepadState)
  ACCESS(CtrlTaskL, ctrlTasks)
  ACCESS(mlr::KinematicWorld, modelWorld)

  Act_GamepadControl* P;
  GamepadInterface *gamepadTh;
  TaskControlMethods *tc;


  sAct_GamepadControl(Act_GamepadControl* P)
    : Thread("Act_GamepadControl", 0.01), g2t(NULL), P(P), gamepadTh(NULL), tc(NULL){
  }

  virtual void open();
  virtual void step();
  virtual void close();

};

//===========================================================================

Act_GamepadControl::Act_GamepadControl(Roopi *r) : Act(r) {
  s = new sAct_GamepadControl(this);
  s->threadLoop();
}

//===========================================================================

Act_GamepadControl::~Act_GamepadControl(){
  s->threadClose();
  delete s;
  roopi.hold(true);
}

//===========================================================================

void sAct_GamepadControl::open(){
  gamepadTh = new GamepadInterface;
  gamepadTh->threadLoop();
}

void sAct_GamepadControl::step(){
  if(!g2t){
    TaskControlThread *taskController = getThread<TaskControlThread>("TaskControlThread");
    CHECK(taskController,"that didn't work");
    taskController->waitForOpened();
    tc = taskController->taskController;
    if(!tc) return;
    g2t = new Gamepad2Tasks(*tc, modelWorld.get(), taskController->q0);

    ctrlTasks.writeAccess();
    taskController->taskController->qNullCostRef.active = false;
    for(CtrlTask *t:ctrlTasks()) t->active = false;
    ctrlTasks().append( g2t->getTasks() );
    ctrlTasks.deAccess();
  }

  arr gamepad = gamepadState.get();
  ctrlTasks.writeAccess();
  g2t->updateTasks(gamepad, modelWorld.get());
  ctrlTasks.deAccess();

  if(stopButtons(gamepad)) P->setStatus(AS_done);

//  if(step_count>10 && gamepad_shutdown) moduleShutdown().incrementValue();
}

void sAct_GamepadControl::close(){
  ctrlTasks.writeAccess();
  tc->qNullCostRef.active = true;
  ctrlTasks.deAccess();

  delete g2t;
  g2t=NULL;
  delete gamepadTh;
}

//===========================================================================

