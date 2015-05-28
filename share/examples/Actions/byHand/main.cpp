#include <Motion/feedbackControl.h>
#include <Actions/taskCtrlActivities.h>
#include <Actions/swig.h>

// ============================================================================

struct MyTask : TaskCtrlActivity {
  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt){}
  virtual bool isConv();
};

void MyTask::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
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

int main(int argc, char** argv) {
  registerActivity<MyTask>("MyTask");

  ActionSwigInterface S(false);

  S.createNewSymbol("MyTask");
  S.createNewSymbol("FollowReferenceActivity"); //-> wird automatisiert
  S.createNewSymbol("HomingActivity"); //-> wird automatisiert
  S.createNewSymbol("endeffR"); //-> wird automatisiert
  S.createNewSymbol("endeffL"); //-> wird automatisiert
  S.createNewSymbol("wheels"); //-> wird automatisiert

  S.setFact("(FollowReferenceActivity wheels){ type=wheels, target=[.2, 0, .3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffR){ type=pos, ref1=endeffR, ref2=base_footprint, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffL){ type=pos, ref1=endeffL, ref2=base_footprint, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity wheels)");
  S.waitForCondition("(conv MyTask endeffL)");
  S.setFact("(MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!");

  S.setFact("(FollowReferenceActivity wheels){ type=wheels, target=[-.2, 0, -.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffR){ type=pos, ref1=endeffR, ref2=base_footprint, target=[.7, -.2, .7], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffL){ type=pos, ref1=endeffL, ref2=base_footprint, target=[.7, +.2, .7], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity wheels)");
  S.waitForCondition("(conv MyTask endeffL)");
  S.setFact("(MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!");

  S.setFact("(FollowReferenceActivity wheels){ type=wheels, target=[.2, 0, .3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffR){ type=pos, ref1=endeffR, ref2=base_footprint, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffL){ type=pos, ref1=endeffL, ref2=base_footprint, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv MyTask endeffL)");
  S.setFact("(MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!, (FollowReferenceActivity wheels)!, (conv FollowReferenceActivity wheels)!");

  S.setFact("(FollowReferenceActivity wheels){ type=wheels, target=[0, 0, 0], PD=[.5, .9, .5, 10.]}");
  S.setFact("(HomingActivity)");
  S.waitForCondition("(conv HomingActivity)");
  S.waitForCondition("(conv FollowReferenceActivity wheels)");

  return 0;
}


