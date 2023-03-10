#include "behaviors.h"
//#include <JK/utils/util.h>

/*

What are behaviors?

They're convenience functions in the main() scope which pick certain variables from the biros system,
set some fields, and thereby trigger the processes to do some thing. So the variables becomes the ``interface''
to the whole system. Behaviors usually have a duration -- they wait until a certain state in some variable is reached.
Some behaviors are sequential like a finite state machine.

*/

#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>
#include <motion/FeedbackControlTasks.h>

#define VAR(Type) \
  Type *_##Type;  biros().getVariableData<Type>(_##Type, #Type, NULL);

void wait(double sec){
  VAR(MotionPrimitive);
  VAR(GamepadState);
  _MotionPrimitive->set_mode(MotionPrimitive::stop, NULL);
  
  double time=mlr::realTime();
  for(;;){
    mlr::wait(.2);
    if(_GamepadState->get_state(NULL)(0)&0x30) break;
    if(sec>0 && mlr::realTime()-time>sec) break;
  }
}

void gamepad(){
  VAR(MotionPrimitive);
  VAR(GamepadState);
  Gamepad_FeedbackControlTask gamepadTask;
  _MotionPrimitive->setFeedbackTask(gamepadTask, true, false, NULL);

  for(;;){
    mlr::wait(.2);
    if(_GamepadState->get_state(NULL)(0)&0x30) break;
  }
  
  _MotionPrimitive->set_mode(MotionPrimitive::stop, NULL);
}
void homing(bool fixFingers){
  VAR(MotionPrimitive);
  VAR(HardwareReference);
  VAR(GamepadState);
  Homing_FeedbackControlTask homeTask;
  _MotionPrimitive->setFeedbackTask(homeTask, true, fixFingers, NULL);
  
  for(;;){
    mlr::wait(.2);
    double dist;
    if(fixFingers)
      dist=length(_HardwareReference->get_q_reference(NULL).sub(0,6));
    else 
      dist=length(_HardwareReference->get_q_reference(NULL));
    cout <<"\rhoming dist = " <<dist <<std::flush;
    if(dist<1e-1) break;
    if(_GamepadState->get_state(NULL)(0)&0x30) break;
  }
  
  _MotionPrimitive->set_mode(MotionPrimitive::stop, NULL);
}

void reach(const char* shapeName, const arr& posGoal, double maxVel){
#if 0
  s->robotProcesses.ctrl.change_task(DoNothing::a());
  s->robotProcesses.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->mytask;
  
  DefaultTaskVariable TV("reach", *s->robotProcesses.ctrl.sys.ors, posTVT, shapeName, NULL, 0);
  TV.setGainsAsNatural(3., 1., false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0., 0., 0.);
  TV.updateState(*s->robotProcesses.ctrl.sys.ors); //non-thread state -- ors actually needs a lock
  
  task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
  task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
  task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities
  
  s->robotProcesses.ctrl.sys.setTaskVariables({&TV, task->TV_col, task->TV_lim, task->TV_q}); //non-thread safe: task variable list needs a lock
  s->robotProcesses.ctrl.taskLock.unlock();
  
  for(; !schunkShutdown;){
    mlr::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->robotProcesses.gamepad.state(0)&0x30) break;
  }
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.sys.setTaskVariables(s->robotProcesses.ctrl.task->TVall);
  s->robotProcesses.ctrl.taskLock.unlock();
  s->robotProcesses.ctrl.change_task(Stop::a());
#else
  NIY;
#endif
}

void reachAndAlign(const char* shapeName, const arr& posGoal, const arr& vecGoal, double maxVel){
#if 0
  s->robotProcesses.ctrl.change_task(DoNothing::a());
  s->robotProcesses.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->mytask;
  
  DefaultTaskVariable TV("reach", *s->robotProcesses.ctrl.sys.ors, posTVT, shapeName, NULL, 0);
  TV.setGainsAsNatural(3., 1., false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0., 0., 0.);
  TV.updateState(*s->robotProcesses.ctrl.sys.ors);
  
  DefaultTaskVariable TValign("align", *s->robotProcesses.ctrl.sys.ors, zalignTVT, shapeName, NULL, 0);
  mlr::Vector vecGoalOrs; vecGoalOrs.set(vecGoal.p);
  TValign.jrel.rot.setDiff(Vector_z, vecGoalOrs);
  TValign.setGainsAsNatural(2., 1., false);
  TValign.y_prec = 1e1;  TValign.y_target = ARR(1.);
  TValign.v_prec = 0.;   TValign.v_target = ARR(0.);
  TValign.updateState(*s->robotProcesses.ctrl.sys.ors);
  
  task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
  task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
  task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities
  
  s->robotProcesses.ctrl.sys.setTaskVariables({&TV, &TValign, task->TV_col, task->TV_lim, task->TV_q});
  s->robotProcesses.ctrl.taskLock.unlock();
  
  for(; !schunkShutdown;){
    mlr::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->robotProcesses.gamepad.state(0)&0x30) break;
  }
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.sys.setTaskVariables(s->robotProcesses.ctrl.task->TVall);
  s->robotProcesses.ctrl.taskLock.unlock();
  s->robotProcesses.ctrl.change_task(Stop::a());
#else
  NIY;
#endif
}

void setMesh(const char* shapeName, const mlr::Mesh& mesh){
#if 0
  mlr::KinematicWorld *ors = s->robotProcesses.ctrl.sys.ors;
  mlr::Shape *shape = ors->getShapeByName(shapeName);
  shape->mesh = mesh;
  shape->type = mlr::ST_mesh;
  if(s->robotProcesses.openGui){
    s->robotProcesses.gui.ors->copyShapesAndJoints(*ors);
    s->robotProcesses.gui.ors2->copyShapesAndJoints(*ors);
  }
#else
  NIY;
#endif
}

void waitForPerceivedObjects(uint numObjects, uint foundSteps){
  VAR(MotionPrimitive);
  VAR(PerceptionOutput);
  VAR(GeometricState);
  VAR(GamepadState);
  _MotionPrimitive->set_mode(MotionPrimitive::stop, NULL);

  for(;;){
    _PerceptionOutput->readAccess(NULL);
    bool bPerceive = false;
    for(uint i=0; i<_PerceptionOutput->objects.N; i++){
      if(_PerceptionOutput->objects.N>=numObjects){
	bool found=true;
	for(uint j=0;j<numObjects;j++)
	  found = found && _PerceptionOutput->objects(j).found>foundSteps;
	if(found){
	  _GeometricState->writeAccess(NULL);
	  for(uint j=0;j<numObjects;j++){
	    mlr::Shape *sh=_GeometricState->ors.getShapeByName(STRING("cyl"<<j+1));
	    sh->rel.pos.set(_PerceptionOutput->objects(j).center3d.p);
	    sh->rel.pos -= sh->body->X.pos;
	  }
	  _GeometricState->deAccess(NULL);
	
          bPerceive = true;
          MLR_MSG("objs found");
        }else{
	  cout <<"looking at objects";
	  for(uint j=0;j<numObjects;j++) cout <<' ' <<_PerceptionOutput->objects(j).found;
	  cout <<endl;
        }
      }
    }
    _PerceptionOutput->deAccess(NULL);
    if(bPerceive) break;
    
    mlr::wait(.2);
    if(_GamepadState->get_state(NULL)(0)&0x30) break;
  }
}

void waitForSmallMotionQueue(MotionFuture *motionFuture, uint queuesize){
  int rev = 0;
  while(motionFuture->getTodoFrames(NULL)>queuesize){
    rev = motionFuture->waitForRevisionGreaterThan(rev);
  }
}

void waitForEmptyQueue(MotionFuture *motionFuture){
  int rev = 0;
  while(!motionFuture->get_done(NULL)){
    rev = motionFuture->waitForRevisionGreaterThan(rev);
  }
}

void pickOrPlaceObject(Action::ActionPredicate action, const char* objShape, const char* belowToShape){
  VAR(MotionFuture);
  
  waitForSmallMotionQueue(_MotionFuture, 1);

  _MotionFuture->appendNewAction(action, objShape, belowToShape, NULL);
  
  waitForSmallMotionQueue(_MotionFuture, 1);
  waitForEmptyQueue(_MotionFuture);
  
  if(action==Action::grasp) _MotionFuture->appendNewAction(Action::closeHand, NULL, NULL, NULL);
  if(action==Action::place) _MotionFuture->appendNewAction(Action::openHand, NULL, NULL, NULL);
  
  waitForEmptyQueue(_MotionFuture);
}

double reach2(const char* objShape, const char* belowToShape){
  VAR(MotionFuture);
  waitForSmallMotionQueue(_MotionFuture, 1);

  _MotionFuture->appendNewAction(Action::reach, objShape, belowToShape, NULL);

  MotionPlanner *planner = _MotionFuture->planners(0);
  //cout << "COSTS: " << planner->motionPrimitive->cost << " AND " << planner->motionPrimitive->iterations_till_convergence << endl;

  waitForSmallMotionQueue(_MotionFuture, 1);
  waitForEmptyQueue(_MotionFuture);
  return planner->motionPrimitive->cost;
}
void homing2(const char* objShape, const char* belowToShape){
  VAR(MotionFuture);

  waitForSmallMotionQueue(_MotionFuture, 1);

  _MotionFuture->appendNewAction(Action::home, objShape, belowToShape, NULL);

  waitForSmallMotionQueue(_MotionFuture, 1);
  waitForEmptyQueue(_MotionFuture);

}
void plannedHoming(const char* objShape, const char* belowToShape){
#if 0
  planner.goalVar->writeAccess(NULL);
  planner.goalVar->goalType=FutureMotionGoal::homingGoalT;
  planner.goalVar->graspShape=objShape;
  planner.goalVar->belowToShape=belowToShape;
  planner.goalVar->deAccess(NULL);
  
  s->robotProcesses.ctrl.change_task(Stop::a());
  
  bool bPlanDone = false;
  bool converged, executed;
  for(; !schunkShutdown;){
    planner.planVar->readAccess(NULL);
    converged=planner.planVar->converged;
    executed =planner.planVar->executed;
    planner.planVar->deAccess(NULL);
    if(converged){
      s->robotProcesses.ctrl.change_task(FollowTrajectory::a());
    }
    //WE COULD SPLIT THIS
    if(executed){
      s->robotProcesses.ctrl.change_task(Stop::a());
      planner.goalVar->writeAccess(NULL);
      planner.goalVar->goalType=FutureMotionGoal::noGoalT;
      planner.goalVar->deAccess(NULL);
      bPlanDone=true;
    }
    
    if(bPlanDone)  break;
    
    mlr::wait(.2);
    if(s->robotProcesses.gamepad.state(0)==16 || s->robotProcesses.gamepad.state(0)==32) return;
  }
  
  mlr::wait(.5); //make the robot really stop...
#else
  NIY;
#endif
}

void graspISF(){

  /*
  Percept_ISF_process perceive;
  GraspObjectVar graspobj;
  perceive.perc_out = &s->perc....
  &graspobj.o = perceive.graspobj;
  
  perceive.threadOpen();
  s->robotProcesses.ctrl.change_task(GraspISF::a());
  
  perceive.threadClose();
  */
}

