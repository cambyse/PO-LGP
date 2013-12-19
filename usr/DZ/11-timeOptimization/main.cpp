#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h> 
#include <DZ/aico_key_frames.h>      
#include <SD/graspObjects.h> 
#include "SD/potentialTaskVariables.h" 
#include "aicot.cpp"                                     
                                       
  

 
void problem7(){        
    cout <<"\n= problem 7: compare to Konrad's time optimization =\n" <<endl;
  
  //setup the problem 
  soc::SocSystem_Ors sys_one;// for 1-step              
  soc::SocSystem_Ors sys;  
  ors::KinematicWorld ors;
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("kuka.ors")));

  OpenGL gl;                       
  arr p,q0;      
  uint dim = 14;   
  arr opt = arr(dim);    
  arr Binv = arr(dim,dim);     
  arr B = arr(dim,dim);                    
    arr r,R,bopt ;       
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
                                                        
  double alpha=MT::getParameter<double>("alpha");
  bool usebwd=MT::getParameter<double>("usebwd") ; 
  double time=8.32;//0.5; // For now - empirical time
  char* obj = "target"; 
  // soc for optimization     
  sys_one.initBasics(NULL,NULL,&gl,T,time,true,NULL); //dynamic!!!
//  createStandardRobotTaskVariables(sys_one);    
  //setGraspGoals(sys_one,T,obj);           
         cout<<"HELP!!!";    
double dr,gr,tm;        
arr kopt=arr(dim);            
arr kBinv,b;                            
 
//  OneStepDynamicFull(b,Binv,sys_one,time,alpha, true); // final posture estimation
//////opt = b;tm=time;
 // time = 1.0;
  //   OneStepDynamicFull(opt,Binv,sys_one,time,alpha);
 // AICO init  
 sys.initBasics(NULL,NULL,&gl,T,time,true,NULL);
 //createStandardRobotTaskVariables(sys);
 //(sys,T,obj);
  double reachPlanHomeComfort=MT::getParameter<double>("reachPlanHomeComfort");
 double reachPlanEndPrec=MT::getParameter<double>("reachPlanEndPrec");
 double reachPlanMidPrec=MT::getParameter<double>("reachPlanMidPrec");
 double reachPlanEndVelPrec=MT::getParameter<double>("reachPlanEndVelPrec");
 double reachPlanColPrec=MT::getParameter<double>("reachPlanColPrec");
 double reachPlanLimPrec=MT::getParameter<double>("reachPlanLimPrec");
 
 
 
 
 
 TaskVariable *pos = new DefaultTaskVariable("position",ors, posTVT,"endeff","<t(0 0 .2)>",0,0,ARR());
  pos->setGainsAsNatural(20,.2);
  pos->targetType=positionGainsTT;
  pos->y_target = arr(ors.getBodyByName("target")->X.pos.p,3);
  
  TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT;
  col->y_prec=1e-0;
  col->y_target = ARR(0.);

  sys.setTaskVariables(ARRAY(pos,col)); 
  sys_one.setTaskVariables(ARRAY(pos,col)); 
  
  pos->setInterpolatedTargetsEndPrecisions(T,reachPlanMidPrec,reachPlanEndPrec,0.,reachPlanEndVelPrec);
  col->setInterpolatedTargetsConstPrecisions(T,reachPlanColPrec,0.);
 int cnt;
//GetOptimalDynamicTime(tm,cnt,opt,Binv,sys_one,alpha,0.05,1e-2,1e-4,1e-5,true);          
cout <<"\nOptimal time =" <<tm<<endl;    
time = tm;              
 
 
// AICO solver(sys);  
AICOT solver(sys);  
 solver.init(sys);  
 
//      ofstream data("dump.txt");
//     for (int k = 0; k <= T; ++k) {
//         data << "p[" << k << "] =" <<solver.p[k] << std::endl;
//         data << "P[" << k << "] =" <<solver.P[k] << std::endl;
//         if (k != T) {
//             data << "PP[" << k << "]=" << solver.PP[k] << std::endl;
//         }
//     }
//     data.close();
//     solver.plotQFun();    
        
if (usebwd){   
 inverse_SymPosDef(B,Binv);
 solver.bwdMsg_v = opt;  
 solver.bwdMsg_Vinv.setDiag(1e2,28);     
 solver.useBwdMsg = usebwd; 
}            
   solver.iterate_to_convergence();
   cout <<"\nOptimal time =" <<sys.getTau()*T<<endl;   
   
 arr   q = solver.q; 
    sys.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
 
   
}
//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){

  case 7:  problem7();  break;
  default: NIY;
  }
  return 0;
}
