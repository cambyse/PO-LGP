#include "act_TaskController.h"
#include "roopi.h"

#include <Control/TaskControllerModule.h>

Act_TaskController::Act_TaskController(Roopi* r)
  : Act(r){
  tcm = new TaskControllerModule("none", NoWorld);

  tcm->threadLoop();
}

Act_TaskController::~Act_TaskController(){
  tcm->threadClose();
  delete tcm;
}

void Act_TaskController::verbose(int verbose){
  if(verbose) tcm->verbose = true;
  else tcm->verbose = false;
}

void Act_TaskController::lockJointGroupControl(const char *groupname, bool lockThem){
  tcm->waitForOpened();
  tcm->stepMutex.lock();
  tcm->taskController->lockJointGroup(groupname, roopi.setKinematics(), lockThem);
  tcm->stepMutex.unlock();
}
