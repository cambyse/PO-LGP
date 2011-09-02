#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>
#include <SD/graspObjects.h>
#include "SD/potentialTaskVariables.h"


void setNewGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId){
  sys.setq0AsCurrent();
  
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
  V->irel.setText("<t(0 0 -.22)>");
  V->updateState();
  V->y_target = xtarget;
  V->y_prec = palmPrec;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  
  //up
  V=listFindByName(sys.vars, "up1");
  V->irel.setText("<d(90 1 0 0)>");
  V->updateState();
  V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
  V->y_prec = endPrec;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  
  //finger tips -> REPLACE by potential
#if 0
  V=listFindByName(sys.vars, "pos1");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "pos2");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "pos3");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
#else

  MT::Array<ors::Shape*> tipsN;
  tipsN.append(sys.ors->getShapeByName("tipNormal1"));
  tipsN.append(sys.ors->getShapeByName("tipNormal2"));
  tipsN.append(sys.ors->getShapeByName("tipNormal3"));
  
  MT::Array<ors::Shape*> hooksN;
  hooksN.append(sys.ors->getShapeByName("tipHook1"));
  hooksN.append(sys.ors->getShapeByName("tipHook2"));
  hooksN.append(sys.ors->getShapeByName("tipHook3"));
  
  GraspObject *graspobj = new GraspObject_Cylinder1(xtarget, ARR(0,0,1), .04, 1., .12);
  graspobj->distanceMode = true;
  
  V = new PotentialValuesTaskVariable("hooksInsideLevel", *sys.ors, hooksN, *graspobj);
  //V=listGetByName(sys.vars,"zeroLevel");
  V->updateState();
  V->y_target = ARR(-.02,-.02,-.02);
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);
  
  V = new PotentialValuesTaskVariable("tipsOnZeroLevel", *sys.ors, tipsN, *graspobj);
  //V=listGetByName(sys.vars,"zeroLevel");
  V->updateState();
  V->y_target = ARR(.02,.02,.02); 
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);
  
  //if (!graspobj->m.V.N)  graspobj->buildMesh();
  //graspobj->saveMesh("grasp_mesh.tri");
  graspobj->loadMesh("grasp_mesh.tri");
  sys.gl->add(glDrawMeshObject, graspobj);
#endif

  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  //
  V=listFindByName(sys.vars, "qitself");
  V->y_prec=MT::getParameter<double>("reachPlanHomeComfort");
  V->v_prec=MT::getParameter<double>("reachPlanEndVelPrec");
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, V->y_prec, V->y_prec, midPrec, V->v_prec);
}


void problem1(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("cyl1")->index);

#if 1
  arr b,Binv;
  //OneStepDynamic(b, Binv, sys, T, 1e-1);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();
#else
  AICO solver(sys);
  solver.iterate_to_convergence();
#endif
  
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  //case 2:  problem2();  break;
  //case 3:  problem3();  break;
  default: NIY;
  }
  return 0;
}
