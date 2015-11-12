#include <Motion/feedbackControl.h>
#include <Actions/ControlActivities.h>
#include <Actions/swig.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/TaskControllerModule.h>

// ============================================================================

struct MyTask : ControlActivity {
  virtual void configureControl(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void stepControl(double dt){}
  virtual bool isConv();
};

void MyTask::configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new DefaultTaskMap(specs, world);
  task = new CtrlTask(name, *map, specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
}

bool MyTask::isConv(){
  return ((task->y_ref.nd==1 && task->y.N==task->y_ref.N
           && maxDiff(task->y, task->y_ref)<stopTolerance
           && maxDiff(task->v, task->v_ref)<stopTolerance));
}

// ============================================================================

void script1(ActionSwigInterface& S){
  S.setFact("(Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffL base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

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

  S.setFact("(Control wheels){ target=[0, 0, 0], PD=[.5, .9, .5, 10.]}");
  S.setFact("(HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
  S.waitForCondition("(conv Control wheels)");
}

// ============================================================================


void script2(ActionSwigInterface& S){
  newActivity<FollowReferenceActivity>(*S.getRM().get()->state, {"Control", "wheels"}, { NO(target, ARR(0, .3, .2)), NO(PD, ARR(.5, .9, .5, 10.))});
  S.setFact("(MyTask pos endeffR base_footprint){ type=pos, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask pos endeffL base_footprint){ type=pos, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv MyTask pos endeffL base_footprint)");
  S.setFact("(MyTask pos endeffL base_footprint)!, (MyTask pos endeffR base_footprint)!, (conv MyTask pos endeffL base_footprint)!, (conv MyTask pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

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
//  gazeAtHand (FollowReferenceActivity){ type="gazeAt", PD=[.5 .9 .1 10.], prec=1. }
//  alignHand (FollowReferenceActivity){ type="vec", vec1=[1 0 0], target=[0.7071, 0, -0.7071], PD=[.5, .9, .1, 10.] }
//  positionHand (FollowReferenceActivity){ type="pos", target=[.7, .3, .7], PD=[.5, .9, .1, 10.] }
//  lowerHand (FollowReferenceActivity){ type="pos", target=[.7, .3, .49], PD=[.5, .9, .1, 10.] }
//  controlForce (FollowReferenceActivity){ type="forceCtrl", ref1="endeffForceL", target=[0 0 -7], timeOut=5. }
//  homing (HomingActivity){ type="homing" }

  S.setFixBase(true);

  S.setFact("(Control gazeAt endeffHead endeffL){ PD=[.5 .9 .1 10.], prec=1. }");
  S.setFact("(Control vec endeffL){ vec1=[1 0 0], target=[0.7071, 0, -0.7071], PD=[.5, .9, .1, 10.] }");
  S.setFact("(Control pos endeffL){ target=[.7, .3, .7], PD=[.5, .9, .1, 10.] }");
  S.waitForCondition("(conv Control vec endeffL), (conv Control pos endeffL)");

  S.setFact("(Control pos endeffL)!, (conv Control pos endeffL)!");
  S.setFact("(Control pos endeffL){ target=[.7, .3, .49], PD=[.5, .9, .1, 10.] }"); //lowering hand
  S.waitForCondition("(conv Control pos endeffL)");

  //-- direct access to the task controller -- a bit awkward, but generic
  TaskControllerModule *taskController = dynamic_cast<TaskControllerModule*>(&registry().getNode("Module","TaskControllerModule")->V<Module>());
  taskController->verbose = true;

  // directly generate a push task
  TaskMap *map = new DefaultTaskMap(posTMT, taskController->modelWorld.get(), "endeffForceL");
  CtrlTask *task = new CtrlTask("Push", map, 1., .8, 1., 1.);
  task->f_ref = ARR(0, 0, -7);
  task->f_Igain = .003;
  taskController->ctrlTasks.set()->append(task);

  mlr::wait(2.);

  taskController->ctrlTasks.set()->removeValue(task);
  delete task;
  delete map;

  //-- back to high-level interface
  taskController->verbose = false;
  S.setFact("(Control pos endeffL)!, (Control gazeAt endeffHead endeffL)!, (Control vec endeffL)!, (HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
}

// ============================================================================

int main(int argc, char** argv) {
  registerActivity<MyTask>("MyTask");

  ActionSwigInterface S;

  S.setFixBase(true);

  S.createNewSymbol("wheels");
  S.createNewSymbol("pos");
  S.createNewSymbol("vec");
  S.createNewSymbol("gazeAt");

//  script1(S);
//  script2(S);
//  script3(S);
  forceControl(S);

  return 0;
}


