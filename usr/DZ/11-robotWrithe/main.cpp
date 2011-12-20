#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h> 
#include <DZ/aico_key_frames.h>      
#include <SD/graspObjects.h> 
#include "../../../share/src/MT/soc.h"
                                   
void createMyStandardRobotTaskVariables(soc::SocSystem_Ors& sys){
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  arr I2(7, 14); I2.setDiag(1.);
  //arr skinIdx; copy(skinIdx, ctrl->skinIndex);
  
  TaskVariable *TV_eff  = new DefaultTaskVariable("endeffector", *sys.ors, posTVT, "Link6", "<t(0 0 -.24)>", 0, 0, 0);
  TaskVariable *TV_q    = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  TaskVariable *TV_rot  = new DefaultTaskVariable("endeffector rotation", *sys.ors, rotTVT, "Link6", 0, 0, 0, 0);
  TaskVariable *TV_col  = new DefaultTaskVariable("collision", *sys.ors, collTVT, 0, 0, 0, 0, ARR(.03)); //MARGIN, perhaps .05?
 // TaskVariable *TV_lim  = new DefaultTaskVariable("limits", *sys.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  //TaskVariable *TV_skin = new TaskVariable("skin", *sys.ors, skinTVT, 0, 0, 0, 0, skinIdx);
  TaskVariable *TV_up   = new DefaultTaskVariable("up1", *sys.ors, zalignTVT, "Link6", "<d(90 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_up2  = new DefaultTaskVariable("up2", *sys.ors, zalignTVT, "Link6", "<d( 0 1 0 0)>", 0, 0, 0);
  TaskVariable *TV_z1   = new DefaultTaskVariable("oppose12", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  TaskVariable *TV_z2   = new DefaultTaskVariable("oppose13", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);

  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", *sys.ors, posTVT, "tip1", "<t( .0   -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", *sys.ors, posTVT, "tip2", "<t( .033 -.09 .0)>", 0, 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", *sys.ors, posTVT, "tip3", "<t(-.033 -.09 .0)>", 0, 0, 0);

 // TaskVariable *TV_qhand= new DefaultTaskVariable("qhand", *sys.ors, qLinearTVT, 0, 0, 0, 0, I2);
  TaskVariableList TVs;
  TVs.append(ARRAY(TV_eff, TV_q, TV_rot, TV_col)); //TV_skin
  TVs.append(ARRAY(TV_up, TV_up2, TV_z1, TV_z2, TV_f1, TV_f2, TV_f3));
  sys.setTaskVariables(TVs);
}

void setMyGraspGoals(soc::SocSystem_Ors& sys, uint T){
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
 // ors::Shape *obj = sys.ors->GetShapeByName("target");
 // obj->cont=true;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
 // xtarget.setCarray(obj->X.pos.p, 3);
   xtarget.setCarray(sys.ors->getShapeByName("cyl1")->X.pos.p, 3);
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
 // V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}
                                      
  


void problem7(){        
    cout <<"\n= ring task=\n" <<endl;
  

  soc::SocSystem_Ors sys;  
  ors::Graph ors;
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("kuka.ors")));

  OpenGL gl;                       
  arr p,q0;      
                   
    arr x0,r,R,bopt ;       
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
                                                        
  double alpha=MT::getParameter<double>("alpha");
  bool usebwd=MT::getParameter<double>("usebwd") ; 
  double time=4.0;//0.5; // For now - empirical time
  char* obj = "target"; 
  // soc for optimization     
  sys.initBasics(NULL,NULL,&gl,T,time,true,NULL); //dynamic!!!
  createMyStandardRobotTaskVariables(sys);    
  setMyGraspGoals(sys,T);           
                           
 

 
sys.getx0(x0);
cout<<x0<<endl;
// AICO solver(sys);  
   AICO solver(sys);  
   solver.init(sys);  
   solver.iterate_to_convergence();
   cout <<"\nOptimal time =" <<sys.getTau()*T<<endl;   
   
 arr q = solver.q; 
    sys.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
 
   
}
//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){

  case 7:  problem7();  break;
  default: NIY;
  }
  return 0;
}
