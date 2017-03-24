#include "teleopControl.h"
#include <Control/TaskControlThread.h>
#include <Control/teleop2tasks.h>

//===========================================================================

TeleopControlActivity::TeleopControlActivity()
  : Thread("TeleopControlActivity", 0.01), taskController(NULL), t2t(NULL){
  taskController = getThread<TaskControlThread>("TaskControlThread");
  CHECK(taskController,"that didn't work");
}

//===========================================================================

TeleopControlActivity::~TeleopControlActivity(){
}

//===========================================================================

void TeleopControlActivity::open(){
}

//===========================================================================

void TeleopControlActivity::step(){
  if(!t2t){
    if(!taskController->taskController) return;
    t2t = new Teleop2Tasks(*taskController->taskController, modelWorld.get());
    t2t->deactivateTasks();
    ctrlTasks.set() = t2t->getTasks();
    taskController->verbose = false;
  }
//  arr Teleop = TeleopState.get();
//  t2t->updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get());
//  if(step_count>10 && Teleop_shutdown) moduleShutdown()->incrementValue();

  if(!initmapper.get()){
    arr gpstate = gamepadState.get();
    CHECK(gpstate.N, "ERROR: No GamePad found");
    t2t->updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get(),gpstate(0), modelWorld.get());
    //t2t->updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get(),0);
  }else{
    t2t->deactivateTasks();
  }

}

//===========================================================================

void TeleopControlActivity::close(){
  delete t2t;
  t2t=NULL;
}

//===========================================================================

RUN_ON_INIT_BEGIN(TeleopControl)
//registerActivity<TeleopControlActivity>("TeleopControlActivity");
RUN_ON_INIT_END(TeleopControl)
