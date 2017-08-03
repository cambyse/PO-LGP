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
  tcm->waitForStatusSmallerThan(tsToOpen);
  while(tcm->step_count<2) mlr::wait(.01);
  tcm->stepMutex.lock();
  tcm->taskController->lockJointGroup(groupname, roopi.setK(), lockThem);
  tcm->stepMutex.unlock();
}

void Act_TaskController::setRealWorldJoint(mlr::Joint* jt, const arr& q)
{
  arr real_q = tcm->realWorld.q;

  mlr::Joint* real_joint = tcm->realWorld.getFrameByName(jt->to()->name)->joint();

  int idx = real_joint->qIndex;
  for (uint i = 0; i < q.d0; i++)
  {
    real_q(idx+i) = q(i);
  }
  tcm->stepMutex.lock();
  tcm->realWorld.setJointState(real_q);
  tcm->ctrl_q_real.set() = real_q;
  tcm->stepMutex.unlock();
}

TaskControlThread* Act_TaskController::getTCM()
{
  return tcm;
}
