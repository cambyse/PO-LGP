#include "gamepadControl.h"
#include <pr2/TaskControllerModule.h>
#include <Motion/gamepad2tasks.h>

GamepadControlActivity::GamepadControlActivity()
  : Module("GamepadControlActivity", 0.01), taskController(NULL), g2t(NULL){
  taskController = dynamic_cast<TaskControllerModule*>(&registry().getNode("Module","TaskControllerModule")->V<Module>());
  CHECK(taskController,"that didn't work");
}

GamepadControlActivity::~GamepadControlActivity(){
}

void GamepadControlActivity::open(){
}

void GamepadControlActivity::step(){
  if(!g2t){
    if(!taskController->feedbackController) return;
    g2t = new Gamepad2Tasks(*taskController->feedbackController);
    ctrlTasks.set() = { g2t->endeffR, g2t->endeffL, g2t->base, g2t->torso, g2t->head, g2t->headAxes, g2t->limits, g2t->coll,  g2t->gripperL, g2t->gripperR };
    taskController->verbose = true;
  }
  arr gamepad = gamepadState.get();
  bool gamepad_shutdown = g2t->updateTasks(gamepad);
  if(step_count>10 && gamepad_shutdown) moduleShutdown().incrementValue();
}

void GamepadControlActivity::close(){
  delete g2t;
  g2t=NULL;
}
