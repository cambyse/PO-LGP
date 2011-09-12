#include "specialTaskVariables.h"


void createStandardRobotTaskVariables(soc::SocSystem_Ors& sys){
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  arr I2(7, 14); I2.setDiag(1.);
  //arr skinIdx; copy(skinIdx, ctrl->skinIndex);
  
  TaskVariable *TV_eff  = new DefaultTaskVariable("endeffector", *sys.ors, posTVT, "m9", "<t(0 0 -.24)>", 0, 0, 0);
  TaskVariable *TV_q    = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  TaskVariable *TV_rot  = new DefaultTaskVariable("endeffector rotation", *sys.ors, rotTVT, "m9", 0, 0, 0, 0);
  TaskVariable *TV_col  = new DefaultTaskVariable("collision", *sys.ors, collTVT, 0, 0, 0, 0, ARR(.03)); //MARGIN, perhaps .05?
  TaskVariable *TV_lim  = new DefaultTaskVariable("limits", *sys.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  //TaskVariable *TV_skin = new TaskVariable("skin", *sys.ors, skinTVT, 0, 0, 0, 0, skinIdx);
  TaskVariable *TV_up   = new DefaultTaskVariable("up1", *sys.ors, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_up2  = new DefaultTaskVariable("up2", *sys.ors, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_z1   = new DefaultTaskVariable("oppose12", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  TaskVariable *TV_z2   = new DefaultTaskVariable("oppose13", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
#if 1
  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", *sys.ors, posTVT, "tipHook1", 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", *sys.ors, posTVT, "tipHook2", 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", *sys.ors, posTVT, "tipHook3", 0, 0);
#else
  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", *sys.ors, posTVT, "tip1", "<t( .0   -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", *sys.ors, posTVT, "tip2", "<t( .033 -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", *sys.ors, posTVT, "tip3", "<t(-.033 -.09 .0)>", 0, 0, 0);
#endif
  TaskVariable *TV_qhand= new DefaultTaskVariable("qhand", *sys.ors, qLinearTVT, 0, 0, 0, 0, I2);
  TaskVariableList TVs;
  TVs.append(ARRAY(TV_eff, TV_q, TV_rot, TV_col, TV_lim)); //TV_skin
  TVs.append(ARRAY(TV_up, TV_up2, TV_z1, TV_z2, TV_f1, TV_f2, TV_f3, TV_qhand));
  sys.setTaskVariables(TVs);
}

void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId){
  sys.setx0AsCurrent();
  
  //load parameters only once!
  static bool firstTime=true;
  static double midPrec, endPrec, palmPrec, colPrec, limPrec, endVelPrec;
  if(firstTime){
    firstTime=false;
    MT::getParameter(midPrec, "reachPlanMidPrec");
    MT::getParameter(endPrec, "reachPlanEndPrec");
    MT::getParameter(palmPrec, "reachPlanPalmPrec");
    MT::getParameter(colPrec, "reachPlanColPrec");
    MT::getParameter(limPrec, "reachPlanLimPrec");
    MT::getParameter(endVelPrec, "reachPlanEndVelPrec");
  }
  
  //set the time horizon
  CHECK(T==sys.nTime(), "");
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj = sys.ors->shapes(shapeId);
  obj->cont=true;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  xtarget.setCarray(obj->X.pos.p, 3);
  xtarget(2) += .02; //grasp it 2cm above center
  
  //endeff
  V=listFindByName(sys.vars, "endeffector");
  ((DefaultTaskVariable*)V)->irel.setText("<t(0 0 -.26)>");
  V->updateState();
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, palmPrec, 0., 0.);
  
  //up
  V=listFindByName(sys.vars, "up1");
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  V->updateState();
  V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //finger tips
  V=listFindByName(sys.vars, "pos1");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "pos2");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "pos3");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}

void setGraspGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape){
  setGraspGoals(sys, T, sys.ors->getShapeByName(objShape)->index);
}

void setPlaceGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape, const char* belowFromShape, const char* belowToShape){
  sys.setx0AsCurrent();
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj  = sys.ors->getShapeByName(objShape);
  ors::Shape *from = sys.ors->getShapeByName(belowFromShape);
  ors::Shape *onto = sys.ors->getShapeByName(belowToShape);
  CHECK(obj->body==sys.ors->getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  onto->cont=false;
  from->cont=false;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  double midPrec, endPrec;
  MT::getParameter(midPrec, "reachPlanMidPrec");
  MT::getParameter(endPrec, "reachPlanEndPrec");
  arr xtarget;
  xtarget.setCarray(onto->X.pos.p, 3);
  xtarget(2) += .5*(onto->size[2]+obj->size[2])+.005; //above 'place' shape
  
  //endeff
  V=listFindByName(sys.vars, "endeffector");
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState();
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  //special: condition effector velocities:
  uint t, M=T/8;
  for(t=0; t<M; t++){
    V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., .2);
    V -> v_prec_trajectory(t) = 1e1;
  }
  for(t=T-M; t<T; t++){
    V -> v_trajectory[t]() = (1./M*(T-t))*ARR(0., 0., -.2);
    V -> v_prec_trajectory(t) = 1e1;
  }
  
  //up1
  V=listFindByName(sys.vars, "up1");
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState();
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //up2
  V=listFindByName(sys.vars, "up2");
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState();
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanColPrec"), 0.);
  V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanLimPrec"), 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
  
  //keep hand fixed
  //V=listFindByName(sys.vars, "qhand");      V->y_target=V->y; V->setInterpolatedTargetsConstPrecisions(T, 1e1, 0.);
}

void setHomingGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape, const char* belowToShape){
  sys.setx0AsCurrent();
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  ors::Shape *obj  = sys.ors->getShapeByName(objShape);
  ors::Shape *onto = sys.ors->getShapeByName(belowToShape);
  obj->cont=true;
  onto->cont=true;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  double midPrec, endPrec;
  MT::getParameter(midPrec, "homingPlanMidPrec");
  MT::getParameter(endPrec, "homingPlanEndPrec");
  
  //endeff
  V=listFindByName(sys.vars, "endeffector");
  //V->irel = obj->rel;
  V->updateState();
  V->setInterpolatedTargetsEndPrecisions(T, 0, 0, 0., 0.);
  //special: condition effector velocities: move above object
  uint t, M=T/8;
  for(t=0; t<M; t++){
    V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., .2);
    V -> v_prec_trajectory(t) = 1e1;
  }
  //for(t=M;t<T;t++){
  //  V -> v_trajectory[t]() = 0;
  //  V -> v_prec_trajectory(t) = 0;
  //}
  
  //col lim and relax
  V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanColPrec"), 0.);
  V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanLimPrec"), 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;
  V->setInterpolatedTargetsEndPrecisions(T,
                                         midPrec, endPrec,
                                         midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}

