#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/util.h>
#include <MT/aico.h> 
#include <MT/plot.h>
#include <DZ/GLI.h>   
#include <DZ/aico_key_frames.h> 
#include <sstream>   
      
const char* USAGE="usage: ./x.exe -orsfile test.ors -dynamic 1 -Hcost 1e-3";
 
int main(int argn,char **argv){     
  MT::initCmdLine(argn,argv);
  ors::Graph ors;
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope.ors")));
 //ors.init(MT::getParameter<MT::String>("orsfile",MT::String("writhe.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);
  
  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(ors::glDrawGraph,&ors);
  gl.camera.setPosition(5,-10,10);
  gl.camera.focus(0,0,1);
 
  gl.watch("loaded configuration - press ENTER");

  //uint T=200; 
uint T=20;
  soc::SocSystem_Ors soc;
  
  soc.initBasics(&ors,&swift,&gl,T,3.,true,NULL);
  soc.os=&std::cout;
  //-- setup the control variables (problem definition)
  arr q;
  soc.getq0(q); 
  TaskVariable *pos = new TaskVariable("position",ors, posTVT,"arm10","<t(0 0 .2)>",0,0,ARR());
  pos->setGainsAsNatural(20,.2);
  pos->targetType=positionGainsTT;
  pos->y_target = arr(ors.getBodyByName("target")->X.pos.p,3);
    
  TaskVariable *wr = new TaskVariable("writhe",ors,qWritheTVT,0,0,0,0,ARR(8));
  
   //wr->setGains(.5,.0);
 // wr->targetType=positionGainsTT; 
  wr->y_prec=1e-3;//1e-0;//  
 // wr->y =zeros(1,100);  
   //wr->y_target =zeros(10,10);          
 //   wr->y_target =ones(10,10);   
  wr->y_target.setId(10);
 
 wr->y_target = wr->y_target*3.0;
 arr yy=zeros(10,10);
//  ifstream os4("wr.traj"); yy.readRaw(os4); os4.close(); 
   ifstream os4("qit89.traj"); yy.readRaw(os4); os4.close(); 

 //wr->y_target(7,7)=0.0;
   wr->y_target = yy;
 // for (int t=8;t<10;t++) wr->y_target(t,t)=0.0;  
// for (int t=3;t<10;t++) wr->y_target(t,t)=10.0;
      
 wr->y_target.resize(1,100);
  cout<<size( wr->y_target)<<endl;
  
  wr->active = true;  
 // wr->params=3;//
  wr->targetType = trajectoryTT;
              
       
int QIT_NUMBER = 4;
  TaskVariable *qit = new TaskVariable("qit",ors,  qItselfTVT,NULL,"<t(0 0 .2)>",0,0,ARR());
  qit->setGainsAsNatural(20,.2);
  qit->targetType=positionGainsTT;
  for (int i=QIT_NUMBER+1;i<11;i++)
     q(i)=-1; 
/*  q(10)=-1.0;
  q(9)=-1.0;
  q(8)=-1.0;*/
//  q = -1.0*ones(1,11);

  //  q(2)=1;
//   q(1)=0.5;
  qit->y_target = q;

  
  TaskVariable *col = new TaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT;
  col->y_prec=1e0;//1e-0;// 
  col->y_target = ARR(0.);  
 
//  soc.setTaskVariables(ARRAY(wr,col));  
 //  soc.setTaskVariables(ARRAY(wr,col)); 
  soc.setTaskVariables(ARRAY(qit,col));  
 
  //-- feedback control (kinematic or dynamic) to reach the targets
  arr dq,qv;
 
  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  soc.getq0(q);  // IMPORTANT!
  //ifstream os3("last.traj"); q.readRaw(os3); os3.close();
 // ifstream os3("mid.traj"); q.readRaw(os3); os3.close();
    
  soc.setq(q);   
  soc.setq0AsCurrent();

  pos->setInterpolatedTargetsEndPrecisions(T,1e-3,1e3,0.,1e3);
  qit->setInterpolatedTargetsEndPrecisions(T,1e-3,1e3,0.,1e3);
  col->setInterpolatedTargetsConstPrecisions(T,1e-2,0.); 
  
  double eps=5e-3; 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
   
   
 // writhe for posterior
  arr B,b,W,H;
  double tm;
/* 
//test   
  arr rope1=arr(1,3);  
  arr rope2=arr(1,3); 
  arr points=arr(1,3); 
   
  arr t1,t2,WM,WJ,J;     
  WM = arr(10,10);
   WJ = arr(10,10);   
 
  for (int i=1;i<12;i++) {   
    rope2.append( ARRAY(-0.1,-0.27*i, 1.85));
    rope1.append( ARRAY(-0.1-0.1*i,-0.28*i, 2.15-0.1*i));

    col->ors->jacobianPos(J,11,NULL);
    //points.append( ARRAY(0.,0., 0.1));
  
    points.append( ARRAY(J(0,i-1),J(1,i-1),J(2,i-1 )));
                
  }  
  GetWritheMatrix(WM,rope1,rope2);
        
  WritheJacobian(WJ,rope1,rope2,points); 
//END of test  */
 //cout<<WJ<<endl  ;   
 
 // GetWritheJacobian() 
  
  
//  
        
  soc.set_plot_writhe(1); 
  q.clear(); 
  
for (int tt=9;tt>5; tt--){  
    
  
   std::stringstream ss;
  ss << "qit" << tt;
   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
  wr->y_target = yy;
  
  //wr->y_target.resize(10,10);
  //for (int t=7-tt;t<10;t++) wr->y_target(t,t)=0.0;  
  wr->params(0) = tt;
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
  soc.setTaskVariables(ARRAY(wr,col)); 
//soc.setTaskVariables(ARRAY(qit,col)); 
  soc.gl->watch();   
  AICO aico(soc); 
  aico.iterate_to_convergence(); 

   
 //test
/*  q = aico.q[T];
  soc.setq(q);    
  soc.setq0AsCurrent();
  wr->params(0) = 6;
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
  soc.setTaskVariables(ARRAY(wr,col)); 
  soc.gl->watch();  
  
  AICO aico2(soc);
  aico2.iterate_to_convergence(); 
  */
  q = aico.q; 
  //ofstream os("z.traj"); q.writeRaw(os); os.close();
  //for(;;){ 
    soc.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
  //  }

// arr r1,r2;
//   wr->GetRopes(r1,r2); 
//   GetWritheMatrix(yy,r1,r2);
//   
//   std::stringstream ss;
//   ss << "qit" << QIT_NUMBER;
//     
// ofstream os2(ss.str().c_str()); yy.writeRaw(os2); os2.close(); 
// cout<<yy;  

    
  q = aico.q[T];
  soc.setq(q);    
  soc.setq0AsCurrent(); 
  
 // soc.gl->watch();  
 
   
}    
//      cout <<yy;
  return 0;
  
}
