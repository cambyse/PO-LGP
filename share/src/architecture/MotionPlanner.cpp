#include "motion.h"
#include <MT/aico.h>

struct sMotionPlanner{
  soc::SocSystem_Ors sys, *sys_parent;
  AICO planner;
};

MotionPlanner_AICO::MotionPlanner_AICO():Process("MotionPlanner_AICO"){
}

void MotionPlanner_AICO::open(){ NIY }

void MotionPlanner_AICO::close(){ NIY }
    
void MotionPlanner_AICO::step(){
  if(!motionPlan->get_hasGoal(this)){
    MT::wait(0.01);
    MT_MSG("TODO");
    //broadcast mechanism instead of sleep?
    //stop your own loop and let somebody else wake you up?
    return;
  }

  //pull the current world configuration from the world
  geometricState->readAccess(this);
  s->sys.ors->copyShapesAndJoints(geometricState->ors);
  geometricState->deAccess(this);

  s->planner.init_messages();
  double d=s->planner.step();

  motionPlan->writeAccess(this);
  motionPlan->q_plan = s->planner.q;
  motionPlan->tau  = s->sys.getTau();
  motionPlan->converged=(d<s->planner.tolerance);// NIKOLAY : enssure reasonable plans
  motionPlan->deAccess(this);
}

