#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/util.h>
#include <Core/array.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h> 
#include <DZ/aico_key_frames.h>      
#include <SD/graspObjects.h> 
#include "../../../share/src/MT/soc.h"
#include "DZ/WritheMatrix.h"
#include <sstream>  
#include <WritheRobotTV.cpp>
                            

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
                                      
  
void GetRopesTest(arr& r1,arr& r2,ors::Graph& _ors,int rope_points){
  //// TODO change it all!!!
  
  arr rope1=arr(rope_points,3); 
  arr rope2=arr(rope_points,3);
  arr ty;
  
  uint start_body = 2; 
  ors::Vector rel; rel.setZero();
  
  for (int i=0;i<rope_points;i++) {// start with second body part
      _ors.kinematicsPos(ty,i+start_body,  &rel); 
      rope1[i]() = ty;
  } 
   arr xtarget;  
  for (int i=0;i<rope_points;i++){
   std::stringstream ss;
   ss << "ring" << i;
   xtarget.setCarray(_ors.getShapeByName(ss.str().c_str())->X.pos.p,3);
   rope2[i]()= xtarget;
  }
  r1=rope1;
  r2=rope2;
 cout<<rope1<<endl;
 cout<<rope2<<endl;
}
//! Matrix


void problem7(){        
  cout <<"\n=Kuka ring task, severe problems with control=\n" <<endl;
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
 // createMyStandardRobotTaskVariables(sys);    
//  setMyGraspGoals(sys,T);  
  sys.getq0(q0);   
  q0(1)=-1.3; 
  sys.setq0(q0);
  uint wrsize=5;  
   double eps =1e1;   
 TaskVariable *reach = new DefaultTaskVariable("reach",ors, posTVT,"endeff","<t(0 0 .0)>",0,0,ARR()); //arm20
  arr xtarget; 
  xtarget.setCarray(sys.ors->getShapeByName("cyl1")->X.pos.p, 3);
  reach->y_target = xtarget;   
  reach->setInterpolatedTargetsEndPrecisions(T, eps, eps, 0., 0.);//1e1

   
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y_prec=eps;//1e-0;/ /    
  wr->y_target.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  ifstream in("wr");  wr->y_target.readRaw(in); in.close(); 
// wr->y_target = wr->y ;//zeros(wrsize,wrsize);// zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   zeros(1,1); 
   
 // wr->y_target=zeros(wrsize,wrsize);  
// for (int tp=0;tp< wrsize;tp++)  wr->y_target(1,tp)=1.0;   
 wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);     
  MT::Array<TaskVariable*> Tlist;      
 //  Tlist.append(wr);
 // Tlist.append(reach);

// Tlist.append(sys.vars);   
  sys.setTaskVariables(Tlist);        
  uint segments=5;  
  arr rope1,rope2,yy,Jp,JM,points; 
 // wr->epsilon_check(q0); 
 // plot_writhe(wr->y_target,5); 
 
  sys.gl->watch();
  AICO solver(sys);  
  solver.init(sys);     
  solver.iterate_to_convergence();
            
   arr q = solver.q;  
   sys.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
   //sys.recordTrajectory(q,"writhe","wr_tr");
 
      GetRopes(rope1,rope2,*sys.ors,segments+1,"ds");
   GetWritheMatrix(yy,rope1,rope2,segments);
   cout << yy <<endl; 
 //  ofstream out("wr");  yy.writeRaw(out); out.close(); 
    plot_writhe(yy,5);
}
//===========================================================================

void problem1(){
  cout <<"Grasping test"<<endl;
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");

  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);
  setGraspGoals(sys,T,"cyl1");
  
  AICO solver;
  solver.init(sys);
  solver.iterate_to_convergence();

  for(;;) sys.displayTrajectory(solver.q, NULL, 1, "result"); 
}

void problem2(){
  cout <<"Ring test with writhe"<<endl;
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  arr rope2,rope1,yy;
  uint segments=5;
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  arr q0; q0.resize(sys.ors->getJointStateDimension());
   
   ifstream in("q_final");  q0.readRaw(in); in.close(); 
   sys.setq(q0,0);//!TODO test it!!
   sys.setx0ToCurrent(); 

 //   cout << q0<<endl;
  createStandardRobotTaskVariables(sys);
   setGraspGoals(sys,T,"cyl1");
  
  double eps = 1e-1;
  double wrsize= 5;
  WritheTaskVariable *wr = new WritheTaskVariable("writhe",*sys.ors,"rope",wrsize,1);
  wr->y_prec=eps;//1e-0;/ /    
  wr->y_target.reshape(wrsize,wrsize);  
 //  ifstream in2("wr_final");  wr->y_target.readRaw(in2); in2.close(); 
  wr->y_target= wr->y;
  //wr->y_target= zeros(wrsize,wrsize);  
  wr->y_target.reshape(wrsize,wrsize);  
  //for (int tp=0;tp< wrsize;tp++)  wr->y_target(2,tp)= 2.0* wr->y_target(2,tp); 
  for (int tp=0;tp< wrsize;tp++)  wr->y_target(2,tp)= 1.0;
  for (int tp=0;tp< wrsize;tp++)  wr->y_target(3,tp)= 0.5;
  for (int tp=0;tp< wrsize;tp++)  wr->y_target(4,tp)= 0.5;
  
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0,eps);   
 // cout<< wr->y_trajectory<<endl;
  //! TEST
/*    arr y_trajectory; y_trajectory=zeros(T,wrsize*wrsize);
    ifstream inp("wr_traj");  y_trajectory.readRaw(inp); inp.close(); 
    for (int i=0;i<T;i++) { wr->y_trajectory[i]() =y_trajectory[T-i-1]();  }//i
    wr->y_trajectory[T]()=y_trajectory[0]() ; //T-1
    wr->y_target=y_trajectory[0]() ;*/
  //!
  
  MT::Array<TaskVariable*> Tlist;      
 
 Tlist.append(wr);
// Tlist.append(sys.vars);   
  sys.setTaskVariables(Tlist);        
 
  
  
  AICO solver;
  solver.init(sys);
  solver.iterate_to_convergence();
  
   
  GetRopes(rope1,rope2,*sys.ors,segments+1,"ds");
  GetWritheMatrix(yy,rope1,rope2,segments);
  plot_writhe(yy,5);
  cout <<yy;
//ofstream out("q_final");  solver.q[T].writeRaw(out); out.close(); 
//  ofstream out("wr_final");  yy.writeRaw(out); out.close();
  //sys.recordTrajectory(solver.q,"writhe","wr_traj");
  for(;;) sys.displayTrajectory(solver.q, NULL, 1, "result"); 
}




int main(int argn,char **argv){
  MT::initCmdLine(argn,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){

  case 7:  problem7();  break;
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  default: NIY;
  }
  return 0;
}
