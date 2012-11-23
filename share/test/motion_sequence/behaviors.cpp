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

void wait(double sec){
  VAR(MotionPrimitive);
  VAR(JoystickState);
  _MotionPrimitive->set_mode(MotionPrimitive::none, NULL);
  
  double time=MT::realTime();
  for(;;){
    MT::wait(.2);
    if(_JoystickState->get_state(NULL)(0)&0x30) break;
    if(sec>0 && MT::realTime()-time>sec) break;
  }
}

void joystick(){
  VAR(MotionPrimitive);
  VAR(JoystickState);
  Joystick_FeedbackControlTask joyTask;
  _MotionPrimitive->setFeedbackTask(joyTask, true, false, NULL);

  for(;;){
    MT::wait(.2);
    if(_JoystickState->get_state(NULL)(0)&0x30) break;
  }
  
  _MotionPrimitive->set_mode(MotionPrimitive::none, NULL);
}

void homing(bool fixFingers){
  VAR(MotionPrimitive);
  VAR(HardwareReference);
  VAR(JoystickState);
  Homing_FeedbackControlTask homeTask;
  _MotionPrimitive->setFeedbackTask(homeTask, true, fixFingers, NULL);
  
  for(;;){
    MT::wait(.2);
    double dist;
    if(fixFingers)
      dist=norm(_HardwareReference->get_q_reference(NULL).sub(0,6));
    else 
      dist=norm(_HardwareReference->get_q_reference(NULL));
    cout <<"\rhoming dist = " <<dist <<std::flush;
    if(dist<1e-1) break;
    if(_JoystickState->get_state(NULL)(0)&0x30) break;
  }
  
  _MotionPrimitive->set_mode(MotionPrimitive::none, NULL);
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
  
  s->robotProcesses.ctrl.sys.setTaskVariables(ARRAY<TaskVariable*>(&TV, task->TV_col, task->TV_lim, task->TV_q)); //non-thread safe: task variable list needs a lock
  s->robotProcesses.ctrl.taskLock.unlock();
  
  for(; !schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->robotProcesses.joy.state(0)&0x30) break;
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
  ors::Vector vecGoalOrs; vecGoalOrs.set(vecGoal.p);
  TValign.jrel.rot.setDiff(VEC_z, vecGoalOrs);
  TValign.setGainsAsNatural(2., 1., false);
  TValign.y_prec = 1e1;  TValign.y_target = ARR(1.);
  TValign.v_prec = 0.;   TValign.v_target = ARR(0.);
  TValign.updateState(*s->robotProcesses.ctrl.sys.ors);
  
  task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
  task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
  task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities
  
  s->robotProcesses.ctrl.sys.setTaskVariables(ARRAY<TaskVariable*>(&TV, &TValign, task->TV_col, task->TV_lim, task->TV_q));
  s->robotProcesses.ctrl.taskLock.unlock();
  
  for(; !schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->robotProcesses.joy.state(0)&0x30) break;
  }
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.sys.setTaskVariables(s->robotProcesses.ctrl.task->TVall);
  s->robotProcesses.ctrl.taskLock.unlock();
  s->robotProcesses.ctrl.change_task(Stop::a());
#else
  NIY;
#endif
}

void setMesh(const char* shapeName, const ors::Mesh& mesh){
#if 0
  ors::Graph *ors = s->robotProcesses.ctrl.sys.ors;
  ors::Shape *shape = ors->getShapeByName(shapeName);
  shape->mesh = mesh;
  shape->type = ors::meshST;
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
  VAR(JoystickState);
  _MotionPrimitive->set_mode(MotionPrimitive::none, NULL);

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
            ors::Shape *sh=_GeometricState->ors.getShapeByName(STRING("cyl"<<j+1));
            sh->rel.pos.set(_PerceptionOutput->objects(j).center3d.p);
            sh->rel.pos -= sh->body->X.pos;
          }
          _GeometricState->deAccess(NULL);

          bPerceive = true;
          MT_MSG("objs found");
        }else{
          cout <<"looking at objects";
          for(uint j=0;j<numObjects;j++) cout <<' ' <<_PerceptionOutput->objects(j).found;
          cout <<endl;
        }
      }
    }
    _PerceptionOutput->deAccess(NULL);
    if(bPerceive) break;

    MT::wait(.2);
    if(_JoystickState->get_state(NULL)(0)&0x30) break;
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

void pickOrPlaceObject(MotionPrimitive::ActionPredicate action, const char* objShape, const char* belowToShape){
  VAR(MotionFuture);
  
  waitForSmallMotionQueue(_MotionFuture, 1);

  _MotionFuture->appendNewAction(action, objShape, belowToShape, NoArr, NULL);
  
  //waitForSmallMotionQueue(_MotionFuture, 1);
  //waitForEmptyQueue(_MotionFuture);
  
  if(action==MotionPrimitive::grasp) _MotionFuture->appendNewAction(MotionPrimitive::closeHand, NULL, NULL, NoArr, NULL);
  if(action==MotionPrimitive::place) _MotionFuture->appendNewAction(MotionPrimitive::openHand, NULL, NULL, NoArr, NULL);
  
  //waitForEmptyQueue(_MotionFuture);
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
    
    MT::wait(.2);
    if(s->robotProcesses.joy.state(0)==16 || s->robotProcesses.joy.state(0)==32) return;
  }
  
  MT::wait(.5); //make the robot really stop...
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

