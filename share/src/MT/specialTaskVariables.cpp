/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "specialTaskVariables.h"


void createStandardRobotTaskVariables(OrsSystem& sys){
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  arr I2(7, 14); I2.setDiag(1.);
  //arr skinIdx; copy(skinIdx, ctrl->skinIndex);
  
  TaskVariable *TV_eff  = new DefaultTaskVariable("endeffector", sys.getOrs(), posTVT, "m9", "<t(0 0 -.24)>", 0, 0, 0);
  TaskVariable *TV_q    = new DefaultTaskVariable("qitself", sys.getOrs(), qItselfTVT, 0, 0, 0, 0, 0);
  TaskVariable *TV_rot  = new DefaultTaskVariable("endeffector rotation", sys.getOrs(), rotTVT, "m9", 0, 0, 0, 0);
  TaskVariable *TV_col  = new DefaultTaskVariable("collision", sys.getOrs(), collTVT, 0, 0, 0, 0, ARR(.03)); //MARGIN, perhaps .05?
  TaskVariable *TV_lim  = new DefaultTaskVariable("limits", sys.getOrs(), qLimitsTVT, 0, 0, 0, 0, limits);
  //TaskVariable *TV_skin = new TaskVariable("skin", sys.getOrs(), skinTVT, 0, 0, 0, 0, skinIdx);
  TaskVariable *TV_up   = new DefaultTaskVariable("up1", sys.getOrs(), zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_up2  = new DefaultTaskVariable("up2", sys.getOrs(), zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_z1   = new DefaultTaskVariable("oppose12", sys.getOrs(), zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  TaskVariable *TV_z2   = new DefaultTaskVariable("oppose13", sys.getOrs(), zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
#if 1
  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", sys.getOrs(), posTVT, "tipHook1", 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", sys.getOrs(), posTVT, "tipHook2", 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", sys.getOrs(), posTVT, "tipHook3", 0, 0);
#else
  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", sys.getOrs(), posTVT, "tip1", "<t( .0   -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", sys.getOrs(), posTVT, "tip2", "<t( .033 -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", sys.getOrs(), posTVT, "tip3", "<t(-.033 -.09 .0)>", 0, 0, 0);
#endif
  TaskVariable *TV_qhand= new DefaultTaskVariable("qhand", sys.getOrs(), qLinearTVT, 0, 0, 0, 0, I2);
  TaskVariableList TVs;
  TVs.append(ARRAY(TV_eff, TV_q, TV_rot, TV_col, TV_lim)); //TV_skin
  TVs.append(ARRAY(TV_up, TV_up2, TV_z1, TV_z2, TV_f1, TV_f2, TV_f3, TV_qhand));
  sys.setTaskVariables(TVs);
}

void setGraspGoals(OrsSystem& sys, uint T, uint shapeId){
  sys.setx0ToCurrent();
  
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
  CHECK(T==sys.get_T(), "");
  
  //deactivate all variables
  activateAll(sys.vars(), false);
  
  //activate collision testing with target shape
  ors::Shape *obj = sys.getOrs().shapes(shapeId);
  obj->cont=true;
  sys.getSwift().initActivations(sys.getOrs());
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  xtarget.setCarray(obj->X.pos.p, 3);
  xtarget(2) += .02; //grasp it 2cm above center
  
  //endeff
  V=listFindByName(sys.vars(), "endeffector");
  ((DefaultTaskVariable*)V)->irel.setText("<t(0 0 -.26)>");
  V->updateState(sys.getOrs());
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, palmPrec, 0., 0.);
  
  //up
  V=listFindByName(sys.vars(), "up1");
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  V->updateState(sys.getOrs());
  V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //finger tips
  V=listFindByName(sys.vars(), "pos1");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars(), "pos2");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars(), "pos3");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //opposing fingers
  V=listFindByName(sys.vars(), "oppose12");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars(), "oppose13");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  V=listFindByName(sys.vars(), "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  V=listFindByName(sys.vars(), "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
  V=listFindByName(sys.vars(), "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}

void setGraspGoals(OrsSystem& sys, uint T, const char* objShape){
  setGraspGoals(sys, T, sys.getOrs().getShapeByName(objShape)->index);
}

void setPlaceGoals(OrsSystem& sys, uint T, const char* objShape, const char* belowFromShape, const char* belowToShape){
  sys.setx0ToCurrent();
  
  //deactivate all variables
  activateAll(sys.vars(), false);
  
  //activate collision testing with target shape
  ors::Shape *obj  = sys.getOrs().getShapeByName(objShape);
  ors::Shape *from = sys.getOrs().getShapeByName(belowFromShape);
  ors::Shape *onto = sys.getOrs().getShapeByName(belowToShape);
  CHECK(obj->body==sys.getOrs().getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  onto->cont=false;
  from->cont=false;
  sys.getSwift().initActivations(sys.getOrs());
  
  TaskVariable *V;
  
  //general target
  double midPrec, endPrec;
  MT::getParameter(midPrec, "reachPlanMidPrec");
  MT::getParameter(endPrec, "reachPlanEndPrec");
  arr xtarget;
  xtarget.setCarray(onto->X.pos.p, 3);
  xtarget(2) += .5*(onto->size[2]+obj->size[2])+.005; //above 'place' shape
  
  //endeff
  V=listFindByName(sys.vars(), "endeffector");
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState(sys.getOrs());
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
  V=listFindByName(sys.vars(), "up1");
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState(sys.getOrs());
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //up2
  V=listFindByName(sys.vars(), "up2");
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState(sys.getOrs());
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  V=listFindByName(sys.vars(), "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanColPrec"), 0.);
  V=listFindByName(sys.vars(), "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanLimPrec"), 0.);
  V=listFindByName(sys.vars(), "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
  
  //keep hand fixed
  //V=listFindByName(sys.vars(), "qhand");      V->y_target=V->y; V->setInterpolatedTargetsConstPrecisions(T, 1e1, 0.);
}

void setHomingGoals(OrsSystem& sys, uint T, const char* objShape, const char* belowToShape){
  sys.setx0ToCurrent();
  
  //deactivate all variables
  activateAll(sys.vars(), false);
  
  ors::Shape *obj  = sys.getOrs().getShapeByName(objShape);
  ors::Shape *onto = sys.getOrs().getShapeByName(belowToShape);
  obj->cont=true;
  onto->cont=true;
  sys.getSwift().initActivations(sys.getOrs());
  
  TaskVariable *V;
  
  //general target
  double midPrec, endPrec;
  MT::getParameter(midPrec, "homingPlanMidPrec");
  MT::getParameter(endPrec, "homingPlanEndPrec");
  
  //endeff
  V=listFindByName(sys.vars(), "endeffector");
  //V->irel = obj->rel;
  V->updateState(sys.getOrs());
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
  V=listFindByName(sys.vars(), "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanColPrec"), 0.);
  V=listFindByName(sys.vars(), "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanLimPrec"), 0.);
  V=listFindByName(sys.vars(), "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;
  V->setInterpolatedTargetsEndPrecisions(T,
                                         midPrec, endPrec,
                                         midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}

