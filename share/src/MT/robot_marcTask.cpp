#include "robot_marcTask.h"
#include "specialTaskVariables.h"


//===========================================================================
//
// Trivial bwd message task
//

TrivialBwdMsgTask::TrivialBwdMsgTask():TaskAbstraction(){
  planVar = NULL;
};

void TrivialBwdMsgTask::updateTaskVariables(ControllerProcess *ctrl){
  activateAll(TVall, false); //deactivate all variables
  
  TV_col->active=true;
  TV_lim->active=true;
  ctrl->useBwdMsg=false;
  
  if(planVar){
    planVar->writeAccess(ctrl);
    if(planVar->converged){
      uint t=planVar->ctrlTime/planVar->tau;
      ctrl->bwdMsg_v    = planVar->bwdMsg_v   [t];
      ctrl->bwdMsg_Vinv = planVar->bwdMsg_Vinv[t];
      planVar->ctrlTime+=0.01;
      if(planVar->ctrlTime>planVar->totalTime){
        planVar->ctrlTime = planVar->totalTime;
        planVar->executed = true;
      }
      ctrl->useBwdMsg=true;
    }
    planVar->deAccess(ctrl);
    //if(motion.recho.planner.cost < 1.) if(counter<motion.recho.sys->nTime()-1) counter++;
  }
  
  //TaskVariable * t = listFindByName(recho.sys->vars, "endeffector");
  //cout  <<" clone "  <<t->y  <<" target"  <<t->y_trajectory[t->y_trajectory.d0-1]  <<" prec "  <<t->y_prec_trajectory(t->y_trajectory.d0-1) <<endl;
}


//===========================================================================
//
// Marc's Robot Task
//

MarcsRobotTask::MarcsRobotTask(){
  ctrl.taskLock.writeLock();
  ctrl.task = this;
  this->joyVar = &joy;
  ctrl.taskLock.unlock();
}

MarcsRobotTask::~MarcsRobotTask(){
}

void MarcsRobotTask::watch(){
  if(!gui.gl) return;
  gui.gl->watch();
}

void MarcsRobotTask::watchTrajectory(){
  if(!gui.gl) return;
  gui.gl->add(ors::glDrawGraph, &ctrl.ors);
  ctrl.sys.gl=gui.gl;
  arr q;
  do {
    soc::getPositionTrajectory(q, plan_b);
    ctrl.sys.displayTrajectory(q, NULL, 1, "plan_b");
    soc::getPositionTrajectory(q, plan_v);
    ctrl.sys.displayTrajectory(q, NULL, 1, "plan_v");
  } while(ctrl.sys.gl->pressedkey!=27);
  ctrl.sys.gl=NULL;
  gui.gl->drawers.popLast();
}

#ifdef MT_NILS
vision::ObjectList objects_db;
bool is_loaded = false;
void loadObjects(){
  if(!is_loaded){
    vision::load_object_library(objects_db, "./objects_db");
    is_loaded = true;
  }
};

void MarcsRobotTask::localizeObject(const char* identifier){
  loadObjects();
  
  vision::Features F_left, F_right;
  vision::Detection2DList D;
  byteA left, right;
  uint t_det = 10; double t_outlier = 100.;
  uint num_nn = 25; double t_nn =  0.0575;
  doubleA avg_c3d(1, 3);
  avg_c3d.setZero();
  int avg_c3d_counter = 0;
  
  vision::Object *obj=NULL;
  for(uint i = 0; i < objects_db.N; i++){
    if(strcmp(objects_db(i)->name_.c_str(), identifier) == 0){
      obj = objects_db(i);
      break;
    }
  }
  CHECK(obj, "this must not happen");
  
  for(uint k=0; k<50 && !signalStop; k++){
    D.clear();
    
    bumble.capture(left, right);
    resize(left, left, 0.5);
    resize(right, right, 0.5);
    
    vision::detect(D, left, right, ARRAY(obj), t_det, t_outlier, t_nn, num_nn, camera_calibration);
    
    if(D.N > 0){
      avg_c3d += D(0)->c_3D;
      avg_c3d_counter++;
    }
    
    for(uint obi = 0; obi < D.N; obi++){
      vision::Detection2D *o = D(obi);
      vision::draw_interest_points(left, o->F_left.keypoints, 0);
      vision::draw_object(left, o->C_left2, o->c_left2, o->identifier.c_str(), 255, 0, 0);
      vision::draw_interest_points(right, o->F_right.keypoints, 0);
      vision::draw_object(right, o->C_right2, o->c_right2, o->identifier.c_str(), 255, 0, 0);
    }
    cvShow(left, "left");
    cvShow(right, "right");
    
    if(avg_c3d_counter >= 3)
      break;
  }
  
//   if(!D.N) return;
  if(avg_c3d_counter < 3)
    std::cout  <<"avg_c3d_counter < 3"  <<std::endl;
    
  ors::Vector pos;
  avg_c3d/=(double)avg_c3d_counter;
  pos.set(avg_c3d.p);
  cout  <<pos  <<endl;
  //adding dosen radius
  pos(2) += .02;
  pos = ors.getShapeByName("camera")->X*pos;
  cout  <<pos  <<endl;
  objectPosition.setCarray(pos.v, 3);
  
  ors::Shape *s = ors.getShapeByName(D(0)->identifier.c_str());
  s->X.p=pos;
  s->rel.setDifference(s->body->X, s->X);
  cout  <<"localized relative position = "  <<s->rel.p  <<endl;
  if(gui){
    s = gui.ors.getShapeByName(D(0)->identifier.c_str());
    s->X.p=pos;
    s->rel.setDifference(s->body->X, s->X);
  }
}
#else
void MarcsRobotTask::localizeObject(const char* identifier){NIY;}
#endif

//   byteA left, right;
//   ors::Vector pos;
//   for(uint k=0;k<3 && !signalStop;k++){
//     bumble.capture(left, right);
//     localizeHsv(objectPosition, left, right, ARRAY<float>(.0, 1., 1.), ARRAY<float>(.2, .5, .5), 3);
//     pos.set(objectPosition.p);
//     cout  <<pos  <<endl;
//     pos = ors.getShapeByName("camera")->X*pos;
//     cout  <<pos  <<endl;
//     objectPosition.setCarray(pos.v, 3);
//   }
//   ors.getBodyByName("target")->X.p=pos;
//   gui.ors.getBodyByName("target")->X.p=pos;
//   ors.getBodyByName("target")->shapes(0)->cont=false;
// }

void MarcsRobotTask::reachObject(){
  controlMode = reachCM;
  //TV_eff->setGainsAsAttractor(10, .2);
  reachPoint = objectPosition;
  for(; !signalStop;){
    step();
    if(joy.state(0)==16 || joy.state(0)==32) break;
  }
  controlMode = stopCM;
  for(uint t=0; t<10; t++) step();
  //TV_eff->targetType=directTT;
}

void MarcsRobotTask::positionObjectRandomlyInSimulation(){
  ors::Body *target = ctrl.ors.getBodyByName("target");
  target->X.pos(0) = 0.  + (rnd.uni()-0.5)*0.5;
  target->X.pos(1) = -.5 + rnd.uni()*0.2-0.2;
  target->X.pos(2) = .9  + (rnd.uni()-0.5)*0.35;
}

void MarcsRobotTask::planGraspTrajectory(const char* objShape){
  NIY;
#if 0
  if(signalStop) return;
  
  //create your own system
  soc::SocSystem_Ors *planSys;
  planSys=ctrl.sys.newClone(true);
  
  uint T=384>>plan_scale;
  planSys->setTimeInterval(4., T);
  
  setGraspGoals(*planSys, T, objShape);
  // see r3293 for the original task variable setting in this code
  
  if(gui.gl) gui.gl->add(ors::glDrawGraph, planSys->ors);
  if(gui.gl) planSys->gl=gui.gl;
  
  arr q;
  MT::timerStart();
  soc::SocSolver solver;
  solver.init();
  solver.go(*planSys);
  
  plan_b = solver.b;
  plan_v = solver.v;
  plan_Vinv=solver.Vinv;
  
  static uint COUNT=0;
  ofstream fil(STRING("z.planReach" <<COUNT++));
  plan_v.writeTagged(fil, "v");
  plan_Vinv.writeTagged(fil, "Vinv");
  plan_b.writeTagged(fil, "b");
  
  if(gui.gl) gui.gl->drawers.popLast();
#endif
}

void transferBetweenDifferentQlin(arr& xTo, const arr& xFrom, soc::SocSystem_Ors& sysFrom, soc::SocSystem_Ors& sysTo){
  uint T=xFrom.d0, nFrom = sysFrom.ors->Qlin.d1, nTo=sysTo.ors->Qlin.d1;
  arr Tlin, Toff, Qbig, Qbigoff;
  Tlin = sysTo.ors->Qinv * sysFrom.ors->Qlin;
  Toff = sysTo.ors->Qinv * (sysFrom.ors->Qoff - sysTo.ors->Qoff);
  
  Qbig.resize(2*nTo, 2*nFrom);  Qbig.setZero();
  Qbig.setMatrixBlock(Tlin, 0, 0);
  Qbig.setMatrixBlock(Tlin, nTo, nFrom);
  
  Qbigoff.resize(2*nTo);  Qbigoff.setZero();
  Qbigoff.setVectorBlock(Toff, 0);
  
  if(xFrom.nd==2){
    if(xFrom.d1==2*nFrom){//dynamic
      xTo.resize(T, 2*nTo);
      for(uint t=0; t<T; t++) xTo[t] = Qbig * xFrom[t] + Qbigoff;
    } else NIY;
    return;
  }
  if(xFrom.nd==3){
    if(xFrom.d1==2*nFrom){//dynamic
      xTo.resize(T, 2*nTo, 2*nTo);
      for(uint t=0; t<T; t++) xTo[t] = Qbig * xFrom[t] * ~Qbig;
    } else NIY;
    return;
  }
  NIY;
}

void MarcsRobotTask::planPlaceTrajectory(const char* objShape, const char* belowFromShape, const char* belowToShape){
  NIY;
#if 0
  if(signalStop) return;
  
  //create your own system
  soc::SocSystem_Ors *planSys;
  planSys=ctrl.sys.newClone(true);
  //planSys = &sys;
  
  // setup a 7DoF system
  arr Qlin_old=planSys->ors->Qlin, Qoff_old=planSys->ors->Qoff, Qinv_old=planSys->ors->Qinv;
  planSys->ors->Qlin.clear();
  planSys->ors->Qoff.clear();
  planSys->ors->Qinv.clear();
  
  planSys->ors->getJointState(planSys->ors->Qoff);
  for(uint i=0; i<7; i++) planSys->ors->Qoff(i)=0.; //do not change offset for arm joints
  planSys->ors->Qlin.resize(16, 7); planSys->ors->Qlin.setDiag(1.);
  planSys->ors->Qinv.resize(7, 16); planSys->ors->Qinv.setDiag(1.);
  //cout  <<planSys->ors->Qlin  <<planSys->ors->Qinv  <<planSys->ors->Qoff  <<endl;
  
  // reinit the system
  uint T=384>>plan_scale;
  arr W;
  W  <<"[.1 .1 .2 .2 .2 1 1]";
  planSys->initBasics(planSys->ors, planSys->swift, planSys->gl, T, 4., true, &W);
  updateState(planSys->vars);
  
  setPlaceGoals(*planSys, T, objShape, belowFromShape, belowToShape);
  // see r3293 for the original task variable setting in this code
  
  if(gui.gl) gui.gl->add(ors::glDrawGraph, planSys->ors);
  if(gui.gl) planSys->gl=gui.gl;
  
  MT::timerStart();
  soc::SocSolver solver;
  solver.init();
  solver.go(*planSys);
  
  transferBetweenDifferentQlin(plan_b,    solver.b,    *planSys, ctrl.sys);
  transferBetweenDifferentQlin(plan_v,    solver.v,    *planSys, ctrl.sys);
  transferBetweenDifferentQlin(plan_Vinv, solver.Vinv, *planSys, ctrl.sys);
  
  static uint COUNT=0;
  ofstream fil(STRING("z.planPlace" <<COUNT++));
  plan_v.writeTagged(fil, "v");
  plan_Vinv.writeTagged(fil, "Vinv");
  plan_b.writeTagged(fil, "b");
  
  planSys->gl=NULL;
  if(gui.gl) gui.gl->drawers.popLast();
#endif
}

void MarcsRobotTask::loadTrajectory(const char* filename){
  ifstream fil;
  MT::open(fil, filename);
  plan_v.readTagged(fil, "v");
  plan_Vinv.readTagged(fil, "Vinv");
  plan_b.readTagged(fil, "b");
  fil.close();
}

void MarcsRobotTask::loadPlainTrajectory(const char* filename){
  ifstream fil;
  MT::open(fil, filename);
  arr q, qStretch;
  q.readTagged(fil, "q");
  fil.close();
  soc::interpolateTrajectory(qStretch, q, MT::getParameter<double>("loadPlainTrajectoryStretch"));
  q=qStretch;
  soc::getPhaseTrajectory(plan_v, q, .01/plan_speed);
  plan_Vinv.resize(q.d0, 2*q.d1, 2*q.d1);
  double prec = MT::getParameter<double>("loadPlainTrajectoryPrec");
  for(uint t=0; t<q.d0; t++){
    plan_Vinv[t].setDiag(prec);
    for(uint i=q.d1; i<2*q.d1; i++) plan_Vinv[t](i, i)=0.;
  }
  fil.close();
  
  ofstream fil2("z.plan");
  plan_v.writeTagged(fil2, "v");
  plan_Vinv.writeTagged(fil2, "Vinv");
}

void MarcsRobotTask::joystick(){
  controlMode = joystickCM;
  for(; !signalStop;){
    step();
    //cout  <<"tip3 inlink frame = "  <<ors.getBodyByName("tip3")->inLinks(0)->Xworld.p  <<endl;
    if(joy.state(0)==16 || joy.state(0)==32) break;
  }
  controlMode = stopCM;
  for(uint t=0; t<10; t++) step();
  waitJoyClean();
}

void MarcsRobotTask::waitJoyClean(){
  for(; !signalStop;){
    joy.step();
    if(joy.state(0)==0) break;
    MT::wait(.001);
  }
}

void MarcsRobotTask::followTrajectory(){
  controlMode = followTrajCM;
  plan_count=0.;
  for(; !signalStop;){
    if((uint)plan_count >= plan_v.d0) break;
    step();
    if(joy.state(0)==16 || joy.state(0)==32) break;
  }
  controlMode = stopCM;
  for(uint t=0; t<10; t++) step();
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

void MarcsRobotTask::closeHand(const char* objShape, const char* belowShape){
  //deactivate collision testing with target shape
  ors::Shape *obj  =ctrl.ors.getShapeByName(objShape);
  ors::Shape *below=ctrl.ors.getShapeByName(belowShape);
  obj->cont=false;
  below->cont=false;
  ctrl.swift.initActivations(ctrl.ors);
  
  stepCounter=0;
  controlMode = closeHandCM;
  for(; !signalStop;){
    step();
    if(joy.state(0)==16 || joy.state(0)==32) break;
    if(norm(TV_skin->y - TV_skin->y_target) < 1e-3) break;
    if(stepCounter>400) break; //early stop!!
  }
  controlMode = stopCM;
  for(uint t=0; t<10; t++) step();
  
  //attach shape to hand
  obj->body->shapes.removeValue(obj);
  obj->body = ctrl.ors.getBodyByName("m9");
  obj->ibody = obj->body->index;
  obj->body->shapes.append(obj);
  obj->rel.setDifference(obj->body->X, obj->X);
  obj->cont=true;
  below->cont=false;  //below remains turned off!!
  ctrl.swift.initActivations(ctrl.ors);
  
  for(uint t=0; t<10; t++) step(); //reiterate stepping to get out of collision...
  
  if(gui.ors){
    obj=gui.ors->getShapeByName(objShape);
    obj->body->shapes.removeValue(obj);
    obj->body = gui.ors->getBodyByName("m9");
    obj->ibody = obj->body->index;
    obj->body->shapes.append(obj);
    obj->rel.setDifference(obj->body->X, obj->X);
  }
}

void MarcsRobotTask::openHand(const char* objShape){
  stepCounter=0;
  controlMode = openHandCM;
  for(; !signalStop;){
    step();
    if(joy.state(0)==16 || joy.state(0)==32) break;
    //if(stepCounter>200 && norm(TV_skin->y - TV_skin->y_target) < 1e-3) break;
    if(stepCounter>300) break; //early stop!!
    
  }
  controlMode = stopCM;
  for(uint t=0; t<10; t++) step();
  
  //attach shape to back to target-body
  ors::Shape *obj=ctrl.ors.getShapeByName(objShape);
  obj->body->shapes.removeValue(obj);
  obj->body = ctrl.ors.getBodyByName("OBJECTS");
  obj->ibody = obj->body->index;
  obj->body->shapes.append(obj);
  obj->rel.setDifference(obj->body->X, obj->X);
  obj->cont=true;
  ctrl.swift.initActivations(ctrl.ors);
  
  if(gui.ors){
    obj=gui.ors->getShapeByName(objShape);
    obj->body->shapes.removeValue(obj);
    obj->body = gui.ors->getBodyByName("OBJECTS");
    obj->ibody = obj->body->index;
    obj->body->shapes.append(obj);
    obj->rel.setDifference(obj->body->X, obj->X);
  }
  
  for(uint t=0; t<50; t++) step(); //reiterate stepping to get out of collision...
}

void MarcsRobotTask::reactivateCollisions(const MT::Array<const char*>& shapes){
  if(signalStop) return;
  const char *s;
  uint i;
  for_list(i, s, shapes) ctrl.ors.getShapeByName(s)->cont=true;
  ctrl.swift.initActivations(ctrl.ors);
}

void MarcsRobotTask::reactivateCollisions(const MT::Array<ors::Shape*>& shapes){
  if(signalStop) return;
  ors::Shape *s;
  uint i;
  for_list(i, s, shapes) s->cont=true;
  ctrl.swift.initActivations(ctrl.ors);
}

/*void MarcsRobotTask::deactivateInterCollisions(const MT::Array<const char*>& shapes){
  ors::BodyList L;
  for_list(i, s, shapes) L->setAppend(ors.getShapeByName(s)->body);
  swift.deactivate(L);
}*/

/*bool MarcsRobotTask::signalStop=false;
void MarcsRobotTask::signalStopCallback(int){
  signalStop=true;
  RobotModuleGroup::signalStop=true;
}*/

