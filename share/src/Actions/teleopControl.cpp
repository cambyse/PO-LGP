#include "teleopControl.h"
#include <Actions/TaskControllerModule.h>
#include <Motion/teleop2tasks.h>

//===========================================================================

TeleopControlActivity::TeleopControlActivity()
  : Module("TeleopControlActivity", 0.01), taskController(NULL), t2t(NULL){
  taskController = getThread<TaskControllerModule>("TaskControllerModule");
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
    t2t = new Teleop2Tasks(*taskController->taskController);
    t2t->deactivateTasks();
    ctrlTasks.set() = t2t->getTasks();
    taskController->verbose = true;
  }
//  arr Teleop = TeleopState.get();
//  t2t->updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get());
//  if(step_count>10 && Teleop_shutdown) moduleShutdown().incrementValue();

  if(!initmapper.get()){
    t2t->updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get());
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
registerActivity<TeleopControlActivity>("TeleopControlActivity");
RUN_ON_INIT_END(TeleopControl)
