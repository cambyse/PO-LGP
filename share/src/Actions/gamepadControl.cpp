#include "gamepadControl.h"
#include <Control/TaskControllerModule.h>
#include <Control/gamepad2tasks.h>

//===========================================================================

GamepadControlActivity::GamepadControlActivity()
  : Thread("GamepadControlActivity", 0.01), taskController(NULL), g2t(NULL){
  taskController = getThread<TaskControllerModule>("TaskControllerModule");
  CHECK(taskController,"that didn't work");
}

//===========================================================================

GamepadControlActivity::~GamepadControlActivity(){
}

//===========================================================================

void GamepadControlActivity::open(){
}

//===========================================================================

void GamepadControlActivity::step(){
  if(!g2t){
    if(!taskController->taskController) return;
    g2t = new Gamepad2Tasks(*taskController->taskController, taskController->q0);
    ctrlTasks.set() = g2t->getTasks();
//    taskController->verbose = true;
  }
  arr gamepad = gamepadState.get();
  ctrlTasks.writeAccess();
  g2t->updateTasks(gamepad);
  ctrlTasks.deAccess();
//  if(step_count>10 && gamepad_shutdown) moduleShutdown().incrementValue();
}

//===========================================================================

void GamepadControlActivity::close(){
  delete g2t;
  g2t=NULL;
}

//===========================================================================

RUN_ON_INIT_BEGIN(gamepadControl)
//registerActivity<GamepadControlActivity>("GamepadControlActivity");
RUN_ON_INIT_END(gamepadControl)
