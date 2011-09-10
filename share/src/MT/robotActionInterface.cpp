

//===========================================================================
//
// Marc's Robot Task
//
#include "robotActionInterface.h"
#include "robot.h"
#include "specialTaskVariables.h"

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody, const char* belowShape);

//private space:
struct sRobotActionInterface {
  RobotProcessGroup robotProcesses;
  TaskAbstraction mytask;
};

RobotActionInterface::RobotActionInterface(){
  s=new sRobotActionInterface;
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.task = &s->mytask;
  s->mytask.joyVar = &s->robotProcesses.joy;
  s->robotProcesses.ctrl.taskLock.unlock();
}

RobotActionInterface::~RobotActionInterface(){
}

RobotProcessGroup* RobotActionInterface::getProcessGroup(){
  return  &s->robotProcesses;
}

TaskAbstraction* RobotActionInterface::getTask(){
  return &s->mytask;
}

void RobotActionInterface::open(){
  s->robotProcesses.open();
}

void RobotActionInterface::close(){
  s->robotProcesses.close();
}

void RobotActionInterface::wait(double sec){
  s->robotProcesses.ctrl.change_task(Stop::a());
  double time=MT::realTime();
  for(; !schunkShutdown;){
    MT::wait(.2);
    if(s->robotProcesses.joy.state(0)==16 || s->robotProcesses.joy.state(0)==32) break;
    if(sec>0 && MT::realTime()-time>sec) break;
  }
  //while(s->robotProcesses.joy.state(0)!=0) s->robotProcesses.step();
}

void RobotActionInterface::joystick(){
  s->robotProcesses.ctrl.change_task(Joystick::a());
  for(; !schunkShutdown;){
    MT::wait(.2);
    if(s->robotProcesses.joy.state(0)==16 || s->robotProcesses.joy.state(0)==32) break;
  }
  s->robotProcesses.ctrl.change_task(Stop::a());
  //for(uint t=0; t<10; t++) s->robotProcesses.step();
  //while(s->robotProcesses.joy.state(0)!=0) s->robotProcesses.step();
}

void RobotActionInterface::homing(){
  s->robotProcesses.ctrl.change_task(Homing::a());
  for(; !schunkShutdown;){
    MT::wait(.2);
    double dist=norm(s->robotProcesses.ctrl.q_reference);
    cout <<"\rhoming dist = " <<dist <<std::flush;
    if(dist<1e-1) break;
    if(s->robotProcesses.joy.state(0)&0x30) break;
  }
  s->robotProcesses.ctrl.change_task(Stop::a());
}

void RobotActionInterface::reach(const char* shapeName, const arr& posGoal, double maxVel){
  s->robotProcesses.ctrl.change_task(DoNothing::a());
  s->robotProcesses.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->mytask;
  
  DefaultTaskVariable TV("reach", *s->robotProcesses.ctrl.sys.ors, posTVT, shapeName, NULL, 0);
  TV.setGainsAsNatural(3., 1., false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0., 0., 0.);
  TV.updateState(); //non-thread state -- ors actually needs a lock
  
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
}

void RobotActionInterface::reachAndAlign(const char* shapeName, const arr& posGoal, const arr& vecGoal, double maxVel){
  s->robotProcesses.ctrl.change_task(DoNothing::a());
  s->robotProcesses.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->mytask;
  
  DefaultTaskVariable TV("reach", *s->robotProcesses.ctrl.sys.ors, posTVT, shapeName, NULL, 0);
  TV.setGainsAsNatural(3., 1., false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0., 0., 0.);
  TV.updateState();
  
  DefaultTaskVariable TValign("align", *s->robotProcesses.ctrl.sys.ors, zalignTVT, shapeName, NULL, 0);
  ors::Vector vecGoalOrs; vecGoalOrs.set(vecGoal.p);
  TValign.jrel.rot.setDiff(VEC_z, vecGoalOrs);
  TValign.setGainsAsNatural(2., 1., false);
  TValign.y_prec = 1e1;  TValign.y_target = ARR(1.);
  TValign.v_prec = 0.;   TValign.v_target = ARR(0.);
  TValign.updateState();
  
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
}

void RobotActionInterface::setMesh(const char* shapeName, const ors::Mesh& mesh){
  ors::Graph *ors = s->robotProcesses.ctrl.sys.ors;
  ors::Shape *shape = ors->getShapeByName(shapeName);
  shape->mesh = mesh;
  shape->type = ors::meshST;
  if(s->robotProcesses.openGui){
    s->robotProcesses.gui.ors->copyShapesAndJoints(*ors);
    s->robotProcesses.gui.ors2->copyShapesAndJoints(*ors);
  }
}

void RobotActionInterface::perceiveObjects(PerceptionModule& perc){
  s->robotProcesses.ctrl.change_task(Stop::a());
  for(; !schunkShutdown;){
    perc.output.readAccess(NULL);
    bool bPerceive = false;
    for(uint i=0; i<perc.output.objects.N; i++){
      if(perc.output.objects.N>=3){
        if(perc.output.objects(0).found>5 &&
            perc.output.objects(1).found>5 &&
            perc.output.objects(2).found>5){
          ors::Shape *sh=s->robotProcesses.ctrl.ors.getShapeByName("cyl1");
          sh->rel.pos.set(perc.output.objects(0).center3d.p);
          sh->rel.pos -= sh->body->X.pos;
          sh=s->robotProcesses.ctrl.ors.getShapeByName("cyl2");
          sh->rel.pos.set(perc.output.objects(1).center3d.p);
          sh->rel.pos -= sh->body->X.pos;
          
          s->robotProcesses.gui.processLock.writeLock();
          s->robotProcesses.gui.ors->copyShapesAndJoints(s->robotProcesses.ctrl.ors);
          s->robotProcesses.gui.ors2->copyShapesAndJoints(s->robotProcesses.ctrl.ors);
          s->robotProcesses.gui.processLock.unlock();
          bPerceive = true;
          MT_MSG("objs found");
        }else{
          MT_MSG("looking at objects"
                 <<perc.output.objects(0).found <<", "
                 <<perc.output.objects(1).found <<", "
                 <<perc.output.objects(2).found
                );
        }
      }
    }
    perc.output.deAccess(NULL);
    if(bPerceive) break;
    
    MT::wait(.2);
    if(s->robotProcesses.joy.state(0)==16 || s->robotProcesses.joy.state(0)==32) break;
  }
}


//TODO: make goalVar, & planner & perception part of RobotActionInterface

void RobotActionInterface::pickObject(ReceedingHorizonProcess& planner, const char* objShape){
  //trigger the planner to start planning
  planner.goalVar->writeAccess(NULL);
  planner.goalVar->goalType=FutureMotionGoal::graspGoalT;
  planner.goalVar->graspShape=objShape;
  planner.goalVar->deAccess(NULL);
  
  // the robot halts
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
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  reattachShape((s->robotProcesses.ctrl.ors), &s->robotProcesses.ctrl.swift, objShape, "m9", "table");
  reattachShape(*(s->robotProcesses.gui.ors), NULL, objShape, "m9", NULL);
  reattachShape(*(s->robotProcesses.gui.ors2), NULL, objShape, "m9", NULL);
  s->robotProcesses.ctrl.taskLock.unlock();
  
  s->robotProcesses.ctrl.change_task(CloseHand::a());
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.forceColLimTVs=false;
  s->robotProcesses.ctrl.taskLock.unlock();
  
  MT::wait(3.);
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.forceColLimTVs=true;
  s->robotProcesses.ctrl.taskLock.unlock();
  
  s->robotProcesses.ctrl.change_task(Stop::a());
  
}

void RobotActionInterface::placeObject(ReceedingHorizonProcess& planner, const char* objShape, const char* belowFromShape, const char* belowToShape){
  planner.goalVar->writeAccess(NULL);
  planner.goalVar->goalType=FutureMotionGoal::placeGoalT;
  planner.goalVar->graspShape=objShape;
  planner.goalVar->belowFromShape=belowFromShape;
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
      s->robotProcesses.ctrl.taskLock.writeLock();
      s->robotProcesses.ctrl.fixFingers=true;
      s->robotProcesses.ctrl.taskLock.unlock();
      s->robotProcesses.ctrl.change_task(FollowTrajectory::a());
    }
    //WE COULD SPLIT THIS
    if(executed){
      s->robotProcesses.ctrl.change_task(Stop::a());
      s->robotProcesses.ctrl.taskLock.writeLock();
      s->robotProcesses.ctrl.fixFingers=false;
      s->robotProcesses.ctrl.taskLock.unlock();
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
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  reattachShape(s->robotProcesses.ctrl.ors, &s->robotProcesses.ctrl.swift, objShape, "OBJECTS", belowToShape);
  reattachShape(*s->robotProcesses.gui.ors, NULL, objShape, "OBJECTS", NULL);
  reattachShape(*s->robotProcesses.gui.ors2, NULL, objShape, "OBJECTS", NULL);
  s->robotProcesses.ctrl.taskLock.unlock();
  
  s->robotProcesses.ctrl.change_task(OpenHand::a());
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.forceColLimTVs=false;
  s->robotProcesses.ctrl.taskLock.unlock();
  
  MT::wait(3.);
  
  s->robotProcesses.ctrl.taskLock.writeLock();
  s->robotProcesses.ctrl.forceColLimTVs=true;
  s->robotProcesses.ctrl.taskLock.unlock();
  s->robotProcesses.ctrl.change_task(Stop::a());
}

void RobotActionInterface::plannedHoming(ReceedingHorizonProcess& planner, const char* objShape, const char* belowToShape){

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
}

void RobotActionInterface::graspISF(){

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

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody, const char* belowShape){
  ors::Shape *obj  = ors.getShapeByName(objShape);
  obj->body->shapes.removeValue(obj);
  obj->body = ors.getBodyByName(toBody);
  obj->ibody = obj->body->index;
  obj->body->shapes.append(obj);
  obj->rel.setDifference(obj->body->X, obj->X);
  if(swift && belowShape){
    swift->initActivations(ors);
    swift->deactivate(obj, ors.getShapeByName(belowShape));
  }
}

