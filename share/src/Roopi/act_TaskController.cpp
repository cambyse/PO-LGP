#include "act_TaskController.h"
#include "roopi.h"

#include <Control/TaskControlThread.h>

Act_TaskController::Act_TaskController(Roopi* r)
  : Act(r){
  tcm = new TaskControlThread("none", NoWorld);
  tcm->threadLoop(true);
}

Act_TaskController::~Act_TaskController(){
  delete tcm;
}

void Act_TaskController::verbose(int verbose){
  if(verbose) tcm->verbose = true;
  else tcm->verbose = false;
}

void Act_TaskController::lockJointGroupControl(const char *groupname, bool lockThem){
  tcm->waitForStatusSmallerThan(tsOPENING);
  while(tcm->step_count<2) mlr::wait(.01);
  tcm->stepMutex.lock();
  tcm->taskController->lockJointGroup(groupname, roopi.setK(), lockThem);
  tcm->stepMutex.unlock();
}