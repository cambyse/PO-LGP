#include <Motion/feedbackControl.h>
#include <Actions/ControlActivities.h>
#include <Actions/swig.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/TaskControllerModule.h>

// ============================================================================

void script1(ActionSwigInterface& S){
  S.setFact("(Control gazeAt endeffKinect endeffR){ PD=[.1, .9, .5, 10.], prec=10 }");
  S.waitForCondition("(conv Control gazeAt endeffKinect endeffR)");

  S.setFact("(PlayFunnySound)");

  S.setFact("(Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffL base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(PlayFunnySound)!");

  S.setFact("(Control wheels){ target=[0, -.3, -.2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.7, -.2, .7], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.7, +.2, .7], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffR base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffR base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(Control gazeAt endeffKinect endeffR)!");

  S.setFact("(Control wheels){ target=[0, 0, 0], PD=[.5, .9, .5, 10.]}");
  S.setFact("(HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
  S.waitForCondition("(conv Control wheels)");
}

// ============================================================================

void script3(ActionSwigInterface& S){
  S.execScript("script.g");
}

// ============================================================================

void forceControl(ActionSwigInterface& S){
  S.setFixBase(true);

  S.setFact("(Control gazeAt endeffHead endeffL){ PD=[.5 .9 .1 10.], prec=1. }");
  S.setFact("(Control vec endeffL){ vec1=[1 0 0], target=[0.7071, 0, -0.7071], PD=[.5, .9, .1, 10.] }");
  S.setFact("(Control pos endeffL){ target=[.8, .3, .9], PD=[.5, .9, .1, 10.] }");
  S.waitForCondition("(conv Control vec endeffL), (conv Control pos endeffL)");

  S.setFact("(Control pos endeffL)!, (conv Control pos endeffL)!");
  S.setFact("(Control pos endeffL){ target=[.8, .3, .8], PD=[.5, .9, .1, 10.] }"); //lowering hand
  S.waitForCondition("(conv Control pos endeffL)");

  //-- direct access to the task controller -- a bit awkward, but generic
  TaskControllerModule *taskController = dynamic_cast<TaskControllerModule*>(registry().get<Module*>({"Module","TaskControllerModule"}));
  taskController->verbose = true;

#if 0
  // directly generate a push task
  TaskMap *map = new DefaultTaskMap(posTMT, taskController->modelWorld.get(), "endeffForceL");
  CtrlTask *task = new CtrlTask("Push", map, 1., .8, 1., 1.);
  task->f_ref = ARR(0, 0, -7);
  task->f_Igain = .003;
  taskController->ctrlTasks.set()->append(task);

  cout <<"NOW!" <<endl;
  mlr::wait(2.);
  cout <<"DONE!" <<endl;

  taskController->ctrlTasks.set()->removeValue(task);
  delete task;
  delete map;
#endif

  //-- back to high-level interface
  taskController->verbose = false;
  S.setFact("(Control pos endeffL)!, (conv Control pos endeffL)!");
  S.setFact("(Control pos endeffL){ target=[.8, .3, .9], PD=[.5, .9, .1, 10.] }"); //lifting hand
  S.waitForCondition("(conv Control vec endeffL), (conv Control pos endeffL)");

  S.setFact("(Control pos endeffL)!, (Control gazeAt endeffHead endeffL)!, (Control vec endeffL)!, (HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
}

// ============================================================================

#include <Gui/opengl.h>

int main(int argc, char** argv) {
#if 0
  ors::KinematicWorld W("model.kvg");
  W.gl().watch();
  W.gl().camera.setKinect();
  W.gl().camera.X = W.getShapeByName("endeffKinect")->X;
  W.gl().camera.X.addRelativeRotationDeg(180,1,0,0);
  W.gl().watch();
  return 0;
#endif

  ActionSwigInterface S;
  ACCESSname(RelationalMachine, RM)
  cout <<RM.get()->KB <<endl;

  S.createNewSymbol("wheels");
  S.createNewSymbol("pos");
  S.createNewSymbol("vec");
  S.createNewSymbol("gazeAt");

  script1(S);
//  script3(S);
//  forceControl(S);

  threadCloseModules();
  registry().clear();

  cout <<"bye bye" <<endl;
  return 0;
}


