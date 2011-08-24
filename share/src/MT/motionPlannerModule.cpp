#include "motionPlannerModule.h"
#include <MT/specialTaskVariables.h>

ReceedingHorizonProcess::ReceedingHorizonProcess():Process("ReceedingHorizon"){
  sys=sys_parent=NULL;
  graspShapeName=NULL;
  active=false;
};

void ReceedingHorizonProcess::open(){
  CHECK(sys_parent,"please set sys_parent before launching a ReceedingHorizonProcess");

  sys=sys_parent->newClone(true);
  sys->os = &cout;
  sys->setTimeInterval(4., MT::getParameter<uint>("reachPlanTrajectoryLength"));
  planner.init(*sys);
}

void ReceedingHorizonProcess::step(){
  if(!goalVar) return;
  if(!active){ //then become active now!
    goalVar->readAccess(this);
    sys->ors->copyShapesAndJoints(*sys_parent->ors);
    planner.init_messages();
    if(goalVar->goalType==FutureMotionGoal::graspGoalT){
      setGraspGoals(*sys, sys->nTime(), goalVar->graspShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys,q,0);
      //planner.init_trajectory(q);
      active=true;
    }else if(goalVar->goalType==FutureMotionGoal::placeGoalT){
      setPlaceGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowFromShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys,q,0);
      //planner.init_trajectory(q);
      active=true;
    }else if(goalVar->goalType==FutureMotionGoal::homingGoalT){
      setHomingGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys,q,1); //task id is q!!!
      //planner.init_trajectory(q);
      active=true;
    }
    goalVar->deAccess(this);
    planVar->writeAccess(this);
    planVar->converged=false;
    planVar->deAccess(this);
  }
  if(active){ //perhaps become inactive
    goalVar->readAccess(this);
    if(goalVar->goalType==FutureMotionGoal::noGoalT){
      active=false;
    }
    goalVar->deAccess(this);
  }
  if(!active) return; //no goal available yet

  /*if(time_shift){
    shiftTargets(sys->vars,-time_shift);
    planner.shiftSolution(-time_shift); //shift everything 10 steps forward...
    time_shift=0;
  }*/
  double d=planner.step();
  //if(planner.cost<1.) planAvailable=true; else planAvailable=false;

  
  goalVar->readAccess(this);
  if(goalVar->goalType==FutureMotionGoal::noGoalT){  goalVar->deAccess(this);  return;  }
  goalVar->deAccess(this);

  if(planVar){
    planVar->writeAccess(this);
    planVar->bwdMsg_v   =planner.v_old;    //the old versions are those guaranteed to be best-so-far
    planVar->bwdMsg_Vinv=planner.Vinv_old;
    planVar->q    = planner.q_old;
    planVar->x    = planner.b_old;
    planVar->cost = planner.cost_old;
    planVar->tau  = sys->getTau();
    planVar->totalTime = planVar->tau*sys->nTime();
    if(d<planner.tolerance) planVar->converged=true;// NIKOLAY : enssure reasonable plans
    planVar->deAccess(this);
  }
}

void ReceedingHorizonProcess::close(){
}

//===========================================================================

#if 0
MotionPlannerModuleGroup::MotionPlannerModuleGroup(){
}

void MotionPlannerModuleGroup::open(){
  recho.sys_parent=sys_parent;
  recho.threadOpen();
}

void MotionPlannerModuleGroup::step(){
  if(!graspShapeName) return;
  static bool first=true;
  if(recho.threadIsIdle() && first){
    //ors::Shape *s = recho.sys->ors->bodies(graspTargetBodyId)->shapes(0);
    cout <<"*** triggering motion planning to shape " <<graspShapeName <<endl;
    setGraspGoals(*recho.sys, recho.sys->nTime(), graspShapeName); ///HERE IS THE LINK TO THE BRAIN!
    arr q;
    soc::straightTaskTrajectory(*recho.sys,q,0);
    recho.planner.initMessagesWithReferenceQ(q);
    first=false;
  }
  recho.threadStepOrSkip(0);
}
void MotionPlannerModuleGroup::close(){
  recho.threadClose();
}
#endif
