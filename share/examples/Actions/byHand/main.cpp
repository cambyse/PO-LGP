#include <Motion/feedbackControl.h>
#include <Actions/ControlActivities.h>
#include <Actions/swig.h>
#include <Actions/RelationalMachineModule.h>

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

int main(int argc, char** argv) {
  registerActivity<MyTask>("MyTask");

  ActionSwigInterface S;

  S.createNewSymbol("wheels");
  S.createNewSymbol("pos");

//  script1(S);
//  script2(S);
  script3(S);

  return 0;
}


