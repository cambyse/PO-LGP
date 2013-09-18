#include "motionPlannerModule.h"
#include <MT/specialTaskVariables.h>

ReceedingHorizonProcess::ReceedingHorizonProcess():Process("ReceedingHorizon"){
  sys=sys_parent=NULL;
  graspShapeName=NULL;
  active=false;
};

void ReceedingHorizonProcess::open(){
  CHECK(sys_parent, "please set sys_parent before launching a ReceedingHorizonProcess");
  
  sys=sys_parent->newClone(true);
  sys->os = &cout;
  sys->setTimeInterval(4., MT::getParameter<uint>("reachPlanTrajectoryLength"));
  NIY //planner.init(*sys); CHANGE THIS BACK as soon as socNew is finished
}

void ReceedingHorizonProcess::step(){
  if(!goalVar) return;
  if(!active){ //then become active now!
    goalVar->readAccess(this);
    sys->ors->copyShapesAndJoints(*sys_parent->ors);
    planner.init_messages();
    if(goalVar->goalType==FutureMotionGoal::graspGoalT){
      setGraspGoals(*sys, sys->get_T(), goalVar->graspShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 0);
      //planner.init_trajectory(q);
    } else if(goalVar->goalType==FutureMotionGoal::placeGoalT){
      setPlaceGoals(*sys, sys->get_T(), goalVar->graspShape, goalVar->belowFromShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 0);
      //planner.init_trajectory(q);
    } else if(goalVar->goalType==FutureMotionGoal::homingGoalT){
      setHomingGoals(*sys, sys->get_T(), goalVar->graspShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 1); //task id is q!!!
      //planner.init_trajectory(q);
    }
    active=true;
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
    shiftTargets(sys->vars, -time_shift);
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
    planVar->bwdMsg_v   =planner.v();
    planVar->bwdMsg_Vinv=planner.Vinv();
    NIY //planVar->q    = planner.q();
    planVar->x    = planner.b();
    planVar->cost = planner.cost();
    planVar->tau  = sys->getTau();
    planVar->totalTime = planVar->tau*sys->get_T();
    planVar->converged=(d<planner.tolerance());// NIKOLAY : enssure reasonable plans
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
  if(recho.isIdle() && first){
    //ors::Shape *s = recho.sys->ors->bodies(graspTargetBodyId)->shapes(0);
    cout <<"*** triggering motion planning to shape " <<graspShapeName <<endl;
    setGraspGoals(*recho.sys, recho.sys->get_T(), graspShapeName); ///HERE IS THE LINK TO THE BRAIN!
    arr q;
    soc::straightTaskTrajectory(*recho.sys, q, 0);
    recho.planner.initMessagesWithReferenceQ(q);
    first=false;
  }
  recho.threadStepOrSkip(0);
}
void MotionPlannerModuleGroup::close(){
  recho.threadClose();
}
#endif
