
//===========================================================================
//
// Marc's Robot Task
//
#include "robotActionInterface.h"
#include "robot.h"
#include "specialTaskVariables.h"

#include <MT/robot_marcTask.h>

//private space:
struct sRobotActionInterface{
	RobotModuleGroup master;
	TaskAbstraction mytask;
};

RobotActionInterface::RobotActionInterface(){
	s=new sRobotActionInterface;
	s->master.ctrl.taskLock.writeLock();
	s->master.ctrl.task = &s->mytask;
	s->mytask.joyVar = &s->master.joy;
	s->master.ctrl.taskLock.unlock();
}

RobotActionInterface::~RobotActionInterface(){
}

RobotModuleGroup* RobotActionInterface::getProcessGroup(){
	return  &s->master;
}

TaskAbstraction* RobotActionInterface::getTask(){
	return &s->mytask;
}

void RobotActionInterface::open(){
	s->master.open();
}

void RobotActionInterface::close(){
	s->master.close();
}

void RobotActionInterface::wait(double sec){
	s->master.ctrl.taskLock.writeLock();
	s->mytask.controlMode = stopCM;
	s->master.ctrl.taskLock.unlock();
	double time=MT::realTime();
	for(;!schunkShutdown;){
		MT::wait(.2);
		if(s->master.joy.state(0)==16 || s->master.joy.state(0)==32) break;
		if(sec>0 && MT::realTime()-time>sec) break;
	}
	//while(s->master.joy.state(0)!=0) s->master.step();
}

void RobotActionInterface::joystick(){
	s->master.ctrl.taskLock.writeLock();
	s->mytask.controlMode = joystickCM;
	s->master.ctrl.taskLock.unlock();
	for(;!schunkShutdown;){
		MT::wait(.2);
		if(s->master.joy.state(0)==16 || s->master.joy.state(0)==32) break;
	}
	s->master.ctrl.taskLock.writeLock();
	s->mytask.controlMode = stopCM;
	s->master.ctrl.taskLock.unlock();
	for(uint t=0;t<10;t++) s->master.step();
	//while(s->master.joy.state(0)!=0) s->master.step();
}

void RobotActionInterface::homing(){
	s->master.ctrl.taskLock.writeLock();
	s->mytask.controlMode = homingCM;
	s->master.ctrl.taskLock.unlock();
	for(;!schunkShutdown;){
		MT::wait(.2);
		double dist=norm(s->master.ctrl.q_reference);
		cout <<"\rhoming dist = " <<dist <<std::flush;
		if(dist<1e-1) break;
		if(s->master.joy.state(0)&0x30) break;
	}
	s->master.ctrl.taskLock.writeLock();
	s->mytask.controlMode = stopCM;
	s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
	s->master.ctrl.taskLock.writeLock();
	TaskAbstraction *task = &s->mytask;
	s->mytask.controlMode = prefixedCM;

	TaskVariable TV("reach",*s->master.ctrl.sys.ors,posTVT,shapeName,NULL,0);
	TV.setGainsAsNatural(3.,1.,false);
	TV.y_prec = 1e2;  TV.y_target = posGoal;
	TV.v_prec = 0.;   TV.v_target = ARR(0.,0.,0.);
	TV.updateState(); //non-thread state -- ors actually needs a lock

	task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
	task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
	task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities

	s->master.ctrl.sys.setTaskVariables(ARRAY(&TV,task->TV_col,task->TV_lim,task->TV_q)); //non-thread safe: task variable list needs a lock
	s->master.ctrl.taskLock.unlock();

	for(;!schunkShutdown;){
		MT::wait(.2);
		cout <<"\rdist = " <<TV.err <<std::flush;
		if(TV.err<1e-2) break;
		if(s->master.joy.state(0)&0x30) break;
	}

	s->master.ctrl.taskLock.writeLock();
	s->master.ctrl.sys.setTaskVariables(s->master.ctrl.task->TVall);
	s->mytask.controlMode = stopCM;
	s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel){
	s->master.ctrl.taskLock.writeLock();
	TaskAbstraction *task = &s->mytask;
	s->mytask.controlMode = prefixedCM;

	TaskVariable TV("reach",*s->master.ctrl.sys.ors,posTVT,shapeName,NULL,0);
	TV.setGainsAsNatural(3.,1.,false);
	TV.y_prec = 1e2;  TV.y_target = posGoal;
	TV.v_prec = 0.;   TV.v_target = ARR(0.,0.,0.);
	TV.updateState();

	TaskVariable TValign("align",*s->master.ctrl.sys.ors,zalignTVT,shapeName,NULL,0);
	ors::Vector vecGoalOrs; vecGoalOrs.set(vecGoal.p);
	TValign.jrel.rot.setDiff(VEC_z,vecGoalOrs);
	TValign.setGainsAsNatural(2.,1.,false);
	TValign.y_prec = 1e1;  TValign.y_target = ARR(1.);
	TValign.v_prec = 0.;   TValign.v_target = ARR(0.);
	TValign.updateState();

	task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
	task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
	task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities

	s->master.ctrl.sys.setTaskVariables(ARRAY(&TV,&TValign,task->TV_col,task->TV_lim,task->TV_q));
	s->master.ctrl.taskLock.unlock();

	for(;!schunkShutdown;){
		MT::wait(.2);
		cout <<"\rdist = " <<TV.err <<std::flush;
		if(TV.err<1e-2) break;
		if(s->master.joy.state(0)&0x30) break;
	}

	s->master.ctrl.taskLock.writeLock();
	s->master.ctrl.sys.setTaskVariables(s->master.ctrl.task->TVall);
	s->mytask.controlMode = stopCM;
	s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::setMesh(const char* shapeName,const ors::Mesh& mesh){
	ors::Graph *ors = s->master.ctrl.sys.ors;
	ors::Shape *shape = ors->getShapeByName(shapeName);
	shape->mesh = mesh;
	shape->type = ors::meshST;
	if(s->master.openGui){
		s->master.gui.ors->copyShapesAndJoints(*ors);
		s->master.gui.ors2->copyShapesAndJoints(*ors);
	}
}

bool RobotActionInterface::perceiveObjects( PerceptionModule & perc){
	perc.output.readAccess(NULL);
	bool bPerceive = true;
	for(uint i = 0; i < perc.output.objects.N; i++)
		bPerceive = bPerceive && (perc.output.objects(i).found>3);
	if(bPerceive && perc.output.objects.N >=3 ){//so all perceived
		//perc.output.readAccess(NULL);
		ors::Shape *sh=s->master.ctrl.ors.getShapeByName("cyl1");//harc coded shape names !!!!
		sh->rel.pos.set(perc.output.objects(0).center3d.p);
		sh->rel.pos -= sh->body->X.pos;
		sh=s->master.ctrl.ors.getShapeByName("cyl2");
		sh->rel.pos.set(perc.output.objects(1).center3d.p);
		sh->rel.pos -= sh->body->X.pos;

		s->master.gui.ors->copyShapesAndJoints(s->master.ctrl.ors);//stranege bugg here !!!
		s->master.gui.ors2->copyShapesAndJoints(s->master.ctrl.ors);
	}
	perc.output.deAccess(NULL);
	return bPerceive;
}


bool RobotActionInterface::reachGrasp(ReceedingHorizonProcess & planner, char * name){
	planner.goalVar->goalType=graspGoalT;
	planner.goalVar->graspShape=name;
	if(planner.planVar->converged){
		s->mytask.controlMode=followTrajCM;
	}
	bool bAns = false;
	if(planner.planVar->executed){
		s->mytask.controlMode=stopCM;
		planner.goalVar->goalType=noGoalT;
		bAns = true;
	}
	return bAns;
}

bool RobotActionInterface::reattach(char * name){
	s->mytask.controlMode=stopCM;
	static int count=0;  count++;
	bool bAns = false;
	if(count>50){
	   reattachShape((s->master.ctrl.ors), &s->master.ctrl.swift, name, "m9", "table");
	   reattachShape(*(s->master.gui.ors), NULL, name, "m9", NULL);
	   reattachShape(*(s->master.gui.ors2), NULL, name, "m9", NULL);
	   bAns = true;
	}
	return bAns;
}

bool RobotActionInterface::closeHandAndAttach(){
  s->mytask.controlMode=closeHandCM;
  s->master.ctrl.forceColLimTVs=false;
	static int count=0;  count++;
	bool bAns = false;
  if(count>300){
    s->master.ctrl.forceColLimTVs=true;
    s->mytask.controlMode=stopCM;
    bAns = true;
  }
	return bAns;
}

bool RobotActionInterface::wait4PlannerAndReset(ReceedingHorizonProcess& planner){
  s->mytask.controlMode=stopCM;
  bool bAns = false;
  if(planner.threadIsIdle()){
    planner.planVar->converged=false;
    planner.planVar->executed=false;
    planner.planVar->ctrlTime=0.;
    bAns = true;
  }
  return bAns;
}

bool RobotActionInterface::place(
    ReceedingHorizonProcess& planner, const char * sh1, const char * sh_fr, const char * sh_to ){
  planner.goalVar->goalType=placeGoalT;
  planner.goalVar->graspShape=sh1;
  planner.goalVar->belowFromShape=sh_fr;
  planner.goalVar->belowToShape=sh_to;
	bool bAns = false;
  if(planner.planVar->converged){
    s->master.ctrl.fixFingers=true;
    s->mytask.controlMode=followTrajCM;
  }
  if(planner.planVar->executed){
    s->mytask.controlMode=stopCM;
    s->master.ctrl.fixFingers=false;
    planner.goalVar->goalType=noGoalT;
    bAns = true; //SD: FIX: not a binary success ...
  }
  return bAns; 
}

bool RobotActionInterface::stopMotion(){
  bool bAns = false;
  s->mytask.controlMode=stopCM;
  static int count=0;  count++;
  if(count>50){
    bAns = true;
  }
  return bAns; 
}

bool RobotActionInterface::openHandReattach(const char * sh1, const char *sh2){
  bool bAns = false;
  s->mytask.controlMode=openHandCM;
  s->master.ctrl.forceColLimTVs=false;
  static int count=0;  count++;
  if(count>300){
    s->master.ctrl.forceColLimTVs=true;
    s->mytask.controlMode=stopCM;
    reattachShape(s->master.ctrl.ors, &s->master.ctrl.swift, sh1, "OBJECTS", sh2);
    reattachShape(*s->master.gui.ors, NULL, sh1, "OBJECTS", NULL);
    reattachShape(*s->master.gui.ors2, NULL, sh1, "OBJECTS", NULL);
    bAns = true;
  }
  return bAns; 
}

bool RobotActionInterface::homing(ReceedingHorizonProcess& planner,
    const char * sh1, const char *sh2){
  bool bAns = false;
  planner.goalVar->goalType=homingGoalT;
  planner.goalVar->graspShape=sh1;
  planner.goalVar->belowToShape=sh2;
  if(planner.planVar->converged){
    s->mytask.controlMode=followTrajCM;
  }
  if(planner.planVar->executed){
    s->mytask.controlMode=stopCM;
    planner.goalVar->goalType=noGoalT;
    bAns = true;
  }
  return bAns; 
}

	//===========================================================================
	//
	// obsolete implementations
	//

	/*
void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal; double maxVel;
    double dist;
    TaskVariable *TV;
    ControllerProcess *ctrl;
    MyUpdater(ControllerProcess *_ctrl,const char* shapeName,const arr& _posGoal,double _maxVel){
      ctrl=_ctrl;  posGoal=_posGoal;  maxVel=_maxVel;  dist=1e10;
      TV = new TaskVariable("reach",*ctrl->sys.ors,posTVT,shapeName,NULL,0);
      TV->targetType=directTT; 
      ctrl->sys.setTaskVariables(ARRAY(TV));
    }
    ~MyUpdater(){
      ctrl->sys.setTaskVariables(ctrl->task->TVall);
      delete TV;
    }
    void operator()(TaskAbstraction *task,ControllerProcess *ctrl){
      arr d=posGoal-TV->y;
      dist=norm(d);
      if(dist>maxVel) d *= maxVel/dist;
      TV->active = true;
      TV->y_prec = 0.;
      TV->v_prec = 1e3;
      TV->v_target = d;
    }
  } updater(&s->master.ctrl,shapeName,posGoal,maxVel);

  switchTask(s->mytask,updater);
  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<updater.dist <<std::flush;
    if(updater.dist<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->mytask.controlMode = stopCM;
}

void RobotActionInterface::reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal,vecGoal; double maxVel;
    double dist;
    TaskVariable *TVp,*TVv;
    ControllerProcess *ctrl;
    MyUpdater(ControllerProcess *_ctrl,const char* shapeName,const arr& _posGoal,const arr& _vecGoal,double _maxVel){
      ctrl=_ctrl;  posGoal=_posGoal;  vecGoal=_vecGoal;  maxVel=_maxVel;  dist=1e10;
      TVp = new TaskVariable("reach",*ctrl->sys.ors,posTVT,shapeName,NULL,0);
      TVp->targetType=directTT; 
      TVv = new TaskVariable("reach",*ctrl->sys.ors,zalignTVT,shapeName,NULL,0);
      TVv->targetType=directTT; 
      ctrl->sys.setTaskVariables(ARRAY(TVp,TVv));
    }
    ~MyUpdater(){
      ctrl->sys.setTaskVariables(ctrl->task->TVall);
      delete TVp;
      delete TVv;
    }
    void operator()(TaskAbstraction *task,ControllerProcess *ctrl){
      arr d=posGoal-TVp->y;
      dist=norm(d);
      if(dist>maxVel) d *= maxVel/dist;
      TVp->active = true;  TVp->y_prec = 0.;  TVp->v_prec = 1e3;  TVp->v_target = d;
      TVv->active = true;  TVv->y_prec = 1e1;  TVv->y_target = ARR(1.);  TVv->v_prec = 0.;
    }
  } updater(&s->master.ctrl,shapeName,posGoal,vecGoal,maxVel);

  switchTask(s->mytask,updater);
  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<updater.dist <<std::flush;
    if(updater.dist<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->mytask.controlMode = stopCM;
}
	 */
