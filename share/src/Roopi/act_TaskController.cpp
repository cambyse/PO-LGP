#include "act_TaskController.h"

#include <Control/TaskControllerModule.h>

//struct sAct_TaskController : Thread {
//  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)

//  sAct_TaskController() : Thread("CtrlTaskUpdater", .05) {}

//  virtual void open();
//  virtual void step(){
//    ctrlTasks.
//  }
//  virtual void close();

//};

Act_TaskController::Act_TaskController(Roopi* r)
  : Act(r){
  tcm = new TaskControllerModule("none", NoWorld);

  tcm->threadLoop();
  tcm->waitForOpened();
}

Act_TaskController::~Act_TaskController(){
  tcm->threadClose();
  delete tcm;
}
