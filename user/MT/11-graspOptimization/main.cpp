#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>
#include <SD/graspObjects.h>
#include "SD/potentialTaskVariables.h"
#include <MT/ors.h>


void setOldGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
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
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, arr());
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  switch(obj->type){
    case ors::cylinderST:
      V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST:{
      /*rnd.clockSeed();
      static int side=-1;
      if(side==-1) side=rnd(3);
      cout <<"*** side = " <<side <<endl;*/
      ((DefaultTaskVariable*)V)->jrel=obj->X;
      if(side==1) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,1,0,0);
      if(side==2) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,0,1,0);
      V->y_target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    }break;
    default: NIY;
  }
  //cout <<V->irel <<V->jrel <<endl;
  V->updateState();
  if(V->y(0)<0.) ((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  V->updateState();
  V->y_prec = 1e3; //endPrec;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  sys.vars.append(V);
  
  //finger tip hooks
  //V=listFindByName(sys.vars, "pos1");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  //V=listFindByName(sys.vars, "pos2");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  //V=listFindByName(sys.vars, "pos3");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);

  double radius = .05;
  V=new DefaultTaskVariable("hook1", *sys.ors, posTVT, "tipNormal1", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation(0,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  V=new DefaultTaskVariable("hook2", *sys.ors, posTVT, "tipNormal2", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation( .033,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  V=new DefaultTaskVariable("hook3", *sys.ors, posTVT, "tipNormal3", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation(-.033,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  uintA shapes;  ors::Shape *s;
  s = listFindByName(sys.ors->shapes, "tip1Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip2Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip3Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  //shapes.append(shapeId);
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .04, true);
  V->updateState();
  V->y_target = ARR(.0,.0,.0);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  sys.vars.append(V);
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}


void setNewGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
  sys.setTox0();
  
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
  
  //-- create a grasp object for the ors shape
  GraspObject *graspobj;
  switch(obj->type){
      //graspobj = new GraspObject_InfCylinder(ARRAY(obj->X.pos), ARR(0,0,1), .04, 1.);
    case ors::cylinderST:  graspobj = new GraspObject_Cylinder1(obj);  break;
    case ors::boxST:  graspobj = new GraspObject_Box(obj);  break;
    default: NIY;
  }
  graspobj->distanceMode = true;
#if 0
  graspobj->buildMesh();
  graspobj->saveMesh("grasp_mesh.tri");
#else
  //graspobj->loadMesh("grasp_mesh.tri");
#endif
  //sys.gl->add(glDrawMeshObject, graspobj);

  TaskVariable *V;

#if 1 // graspCenter -> predefined point (xtarget)
  //general target
  arr xtarget(obj->X.pos.p, 3);
  xtarget(2) += .02; //grasp it 2cm above center
  
  //endeff
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NULL);
  V->updateState();
  V->y_target = xtarget;
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  sys.vars.append(V);
#else // graspCenter -> negative level
  //MT: somehow this doesn't work: it seems the gradients inside are just too messy!
  V = new PotentialValuesTaskVariable("graspCenterInsideLevel", *sys.ors, ARRAY(sys.ors->getShapeByName("graspCenter")), *graspobj);
  V->updateState();
  V->y_target = ARR(-.01);
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);
#endif
  
  //up
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, arr());
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  switch(obj->type){
    case ors::cylinderST:
      V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST:{
      /*rnd.clockSeed();
      static int side=-1;
      if(side==-1) side=rnd(3);
      cout <<"*** side = " <<side <<endl;*/
      ((DefaultTaskVariable*)V)->jrel=obj->X;
      if(side==1) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,1,0,0);
      if(side==2) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,0,1,0);
      V->y_target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    }break;
    default: NIY;
  }
  //cout <<V->irel <<V->jrel <<endl;
  V->updateState();
  if(V->y(0)<0.) ((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  V->updateState();
  V->y_prec = 1e3; //endPrec;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  sys.vars.append(V);

  if(phase==0) return;
  
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

  /* inside gradients just don't work :-(
  V = new PotentialValuesTaskVariable("hooksInsideLevel", *sys.ors, hooksN, *graspobj);
  //V=listGetByName(sys.vars,"zeroLevel");
  V->updateState();
  V->y_target = ARR(-.01,-.01,-.01);
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  //sys.vars.append(V); */
  
  V = new PotentialValuesTaskVariable("tipsOnZeroLevel", *sys.ors, tipsN, *graspobj);
  V->updateState();
  V->y_target = ARR(.005,.005,.005); 
  V->v_target = ARR(-1.,-1.,-1.); 
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(T,1e2,0.);
  //for(uint t=0;t<T;t++) V->y_trajectory[t]()=.2;  V->y_trajectory[T]=V->y_target;
  //V->v_trajectory.setZero();  V->v_trajectory[T]=V->v_target;
  //sys.vars.append(V);
  
  V = new PotentialFieldAlignTaskVariable("tips z align", *sys.ors, tipsN, *graspobj);
  V->updateState();
  V->y_target = ARR(-1.,-1.,-1.); 
  V->y_prec = 1e2;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);

#endif

  uintA shapes;  ors::Shape *s;
  s = listFindByName(sys.ors->shapes, "tip1Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip2Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip3Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  //shapes.append(shapeId);
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .04, true);
  V->updateState();
  V->y_target = ARR(.0,.0,.0);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,1e1,0.,0.);
  sys.vars.append(V);
  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  V=listFindByName(sys.vars, "qitself");
  V->y_prec=MT::getParameter<double>("reachPlanHomeComfort");
  V->v_prec=MT::getParameter<double>("reachPlanEndVelPrec");
  //V->y_target = ARR(0,0,0,0,0,0,0);
  //V->y_target.append(ARR(0,-.8,.6,-.8,.6,-.8,.6));
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


#if 1
  arr b,Binv;
  //OneStepDynamic(b, Binv, sys, T, 1e-1);
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, 2, 0);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  b.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  sys.setx(b);
  sys.gl->watch();
  
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, 2, 1);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  AICO solver(sys);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = b;
  solver.bwdMsg_Vinv = Binv + diag(1e1,b.N);
  solver.iterate_to_convergence();
#elif 1
  arr b,Binv;
  //OneStepDynamic(b, Binv, sys, T, 1e-1);
  setOldGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, 2, 1);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  AICO solver(sys);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = b;
  solver.bwdMsg_Vinv = Binv + diag(1e1,b.N);
  solver.iterate_to_convergence();
#else
  setOldGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, 2, 1);
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
