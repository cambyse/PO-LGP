#include <Motion/feedbackControl.h>
#include <Actions/ControlActivities.h>
#include <Actions/swig.h>
#include <Actions/RelationalMachineModule.h>
#include <pr2/TaskControllerModule.h>

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
  S.setFact("(Control gazeAt endeffKinect r_gripper_palm_link_0){ PD=[.1, .9, .5, 10.], prec=10 }");

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

  S.setFact("(Control gazeAt endeffKinect r_gripper_palm_link_0)!");

  S.setFact("(Control wheels){ target=[0, 0, 0], PD=[.5, .9, .5, 10.]}");
  S.setFact("(HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
  S.waitForCondition("(conv Control wheels)");
}

// ============================================================================


void script2(ActionSwigInterface& S){
  newActivity<FollowReferenceActivity>(*S.getRM().get()->state, {"Control", "wheels"}, { Nod("target", ARR(0, .3, .2)), Nod("PD", ARR(.5, .9, .5, 10.))});
  S.setFact("(Control gazeAt endeffKinect r_gripper_palm_link_0){ PD=[.1, .9, .5, 10.], prec=10 }");
  mlr::wait(5.);
  S.setFact("(MyTask pos endeffR base_footprint){ type=pos, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask pos endeffL base_footprint){ type=pos, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv MyTask pos endeffL base_footprint)");
  S.setFact("(MyTask pos endeffL base_footprint)!, (MyTask pos endeffR base_footprint)!, (conv MyTask pos endeffL base_footprint)!, (conv MyTask pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(Control wheels){ target=[0, -.3, -.2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.7, -.1, .8], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.7, +.1, .8], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffR base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(Control wheels){ target=[0, .3, .2], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffR base_footprint){ target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(Control pos endeffL base_footprint){ target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv Control wheels)");
  S.waitForCondition("(conv Control pos endeffR base_footprint)");
  S.setFact("(Control pos endeffL base_footprint)!, (Control pos endeffR base_footprint)!, (conv Control pos endeffL base_footprint)!, (conv Control pos endeffR base_footprint)!, (Control wheels)!, (conv Control wheels)!");

  S.setFact("(Control gazeAt endeffKinect r_gripper_palm_link_0)!");
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
  TaskControllerModule *taskController = dynamic_cast<TaskControllerModule*>(&registry().getNode("Module","TaskControllerModule")->V<Module>());
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

  registerActivity<MyTask>("MyTask");

  ActionSwigInterface S;
  ACCESSname(RelationalMachine, RM)
  cout <<RM.get()->KB <<endl;

  S.createNewSymbol("wheels");
  S.createNewSymbol("pos");
  S.createNewSymbol("vec");
  S.createNewSymbol("gazeAt");

//  script1(S);
//  script2(S);
  script3(S);
//  forceControl(S);

  threadCloseModules();
  registry().clear();

  cout <<"bye bye" <<endl;
  return 0;
}


