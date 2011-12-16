#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/util.h> 
#include <MT/aico.h>   
#include <MT/plot.h>     
#include <DZ/WritheMatrix.h>        
#include <DZ/aico_key_frames.h>    
#include <DZ/WritheTaskVariable.h>  
#include <MT/specialTaskVariables.h> 
#include <sstream>        
  
         
const char* USAGE="usage: ./x.exe -orsfile test.ors -dynamic 1 -Hcost 1e-3";
 

int problem1(){     
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20_test.ors")));
 // ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope.ors")));
// ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_20.ors")));

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
  
  soc.initBasics(&ors,&swift,&gl,T,2.,true,NULL);
  soc.os=&std::cout;
  //-- setup the control variables (problem definition)
  arr q;
  soc.getq0(q); 
 
/*  
  TaskVariable *pos = new TaskVariable("position",ors, posTVT,"tip1","<t(0 0 -.0)>",0,0,ARR());
//  TaskVariable *pos = new TaskVariable("position",ors, posTVT,"arm11","<t(0 0 -.2)>",0,0,ARR());
  pos->setGainsAsNatural(20,.2);
  pos->targetType=positionGainsTT;
  pos->y_target = arr(ors.getShapeByName("cyl1")->X.pos.p,3);*/
  MT::Array<ors::Shape*> Wlist;
  Wlist.append(soc.ors->getShapeByName("cyl1"));
    
 WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope2",100,0);
  
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
              
       
int QIT_NUMBER = 1;
//   TaskVariable *qit = new TaskVariable("qit",ors,  qItselfTVT,NULL,"<t(0 0 .2)>",0,0,ARR());
//   qit->setGainsAsNatural(20,.2);
//   qit->targetType=positionGainsTT;
//   for (int i=QIT_NUMBER+1;i<11;i++)
//      q(i)=-1; 
// /*  q(10)=-1.0;
//   q(9)=-1.0;
//   q(8)=-1.0;*/
// //  q = -1.0*ones(1,11);
// 
//   //  q(2)=1;
// //   q(1)=0.5;
//   qit->y_target = q;

  
//   TaskVariable *col = new TaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
//   col->setGains(.5,.0);
//   col->targetType=positionGainsTT;
//   col->y_prec=1e0;//1e-0;//  
//   col->y_target = ARR(0.);  
 
//  soc.setTaskVariables(ARRAY(wr,col));    
 //  soc.setTaskVariables(ARRAY(wr,col)); 
 // soc.setTaskVariables(ARRAY(qit,col));  
 
  //-- feedback control (kinematic or dynamic) to reach the targets
  arr dq,qv;
 
  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  soc.getq0(q);  // IMPORTANT!
  //ifstream os3("last.traj"); q.readRaw(os3); os3.close();
 // ifstream os3("mid.traj"); q.readRaw(os3); os3.close();
    
  soc.setx(q);     


 
  double eps=1e-2; //5e-3; 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
   
/*    
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

    col->ors->jacobian(J,11,NULL);
    //points.append( ARRAY(0.,0., 0.1));
  
    points.append( ARRAY(J(0,i-1),J(1,i-1),J(2,i-1 )));
                
  }  
  GetWritheMatrix(WM,rope1,rope2);
        
  WritheJacobian(WJ,rope1,rope2,points); 
//END of test  */
 //cout<<WJ<<endl  ;   
 
 // GetWritheJacobian() 
  
  
//  
        
//  soc.set_plot_writhe(1); 
 //  soc.plot_writhe();
  q.clear(); 
   
for (int tt=9;tt>3; tt--){  
     soc.initBasics(&ors,&swift,&gl,T,3.5-tt/5,true,NULL); //Fix time
//  soc.set_plot_writhe(1); 
     
   std::stringstream ss;
  ss << "qit" << tt;
   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
// for (int i=0;i<10;i++)
//   for (int j=0;j<10;j++) 
//     if (yy(i,j)>mean(yy)) yy(i,j)=6;
//     else yy(i,j)=0;
      
  wr->y_target = yy;    
   
  //wr->y_target.resize(10,10);
  //for (int t=7-tt;t<10;t++) wr->y_target(t,t)=0.0;  
  wr->params(0) = tt; 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
  MT::Array<TaskVariable*> Tlist;
  Tlist.append(wr);  
  
  soc.setTaskVariables(Tlist); 
//soc.setTaskVariables(ARRAY(qit,col)); 
 // soc.gl->watch();   
  AICO aico(soc); 
  aico.iterate_to_convergence(); 

   
 //test
/*  q = aico.q[T]; 8
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
 // ofstream os; os.open("z.traj",std::_Ios_Openmode(1)); q.writeRaw(os); os.close();

//  ifstream os; os.open("z.traj"); q.readRaw(os); os.close();

  //for(;;){  
    soc.displayTrajectory(q,NULL,20,"AICO (planned trajectory)");
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
  soc.setx(q);    
 // soc.setq0AsCurrent(); 
  
 // soc.gl->watch();  
 
    
} 
T=200;
     soc.initBasics(&ors,&swift,&gl,T,3.0,true,NULL); //Fix time
   
//        pos->setInterpolatedTargetsEndPrecisions(T,1e-3,1e3,0.,1e3);
//   qit->setInterpolatedTargetsEndPrecisions(T,1e-3,1e3,0.,1e3);
//   col->setInterpolatedTargetsConstPrecisions(T,1e-3,0.); 

//     soc.setTaskVariables(ARRAY(col,pos)); 
    AICO aico(soc); 
    aico.iterate_to_convergence(); 
    q = aico.q; 
    soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
    soc.gl->watch(); 

//      cout <<yy;
  return 0;
  
}

int problem2(){     
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_old.ors")));
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
  
  soc.initBasics(&ors,&swift,&gl,T,2.,true,NULL);
  soc.os=&std::cout;
  //-- setup the control variables (problem definition)

  MT::Array<ors::Shape*> Wlist;
  Wlist.append(soc.ors->getShapeByName("cyl1"));  
    
 WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope2",10,1);
 
  wr->y_prec=1e-3;//1e-0;//   
 double eps=1e-2; //5e-3;
 arr q,yy;
 yy = zeros(10,10);
   
for (int tt=9;tt>3; tt--){  
    soc.initBasics(&ors,&swift,&gl,T,3.5-tt/5,true,NULL); //Fix time
  
   std::stringstream ss;
   ss << "qit" << tt;
   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 

  wr->y_target = yy;    
  wr->param = tt; 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
  MT::Array<TaskVariable*> Tlist;
  Tlist.append(wr);  
  
  soc.setTaskVariables(Tlist); 
//soc.setTaskVariables(ARRAY(qit,col)); 
 // soc.gl->watch();  
 
//  arr b,Binv;
//  int cnt;
//  OneStepDynamicFull(b,Binv,cnt,soc,4.,1e-4,1e-4,1e-6,0,0);
//  
    AICO aico(soc); 
    aico.iterate_to_convergence(); 
    q = aico.q; 
    soc.displayTrajectory(q,NULL,20,"AICO (planned trajectory)");
    q = aico.q[T];
    soc.setq(q);    
 // soc.setq0AsCurrent(); 
/*  soc.displayState(&b);
  soc.gl->watch(); */ 
 
    
} 
 
  return 0;
  
}
 
int problem3(){      
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
  gl.camera.setPosition(5,-10,10); 
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 double eps=1e-1; //5e-3;  
 arr q,yy,x0;     
 int wrsize=20;//20;//11
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize,wrsize);              
 int tt=8;     
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);    
    
 std::stringstream ss;         
   ss << "qit" << tt;
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y_prec=1e-3;//1e-0;/ / 
  wr->y.reshape(wrsize,wrsize);  for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
 wr->y_target =zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
//wr->y_target =wr->y;
//ifstream in("init2"); wr->y_target.readRaw(in); in.close();  
  wr->param = tt;   
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;      
   
  TaskVariable *qit    = new DefaultTaskVariable("qitself", ors, qItselfTVT, 0, 0, 0, 0, 0);
  arr q0; soc.getq0(q0); //q0(wrsize-1)=-0.7; 
  qit->y_target = q0;
  qit->y_prec=1e-2;    
  qit->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);

  
  //createMyStandardRobotTaskVariables(soc);
  //setMyGraspGoals(soc,T,soc.ors->getShapeByName("cyl1")->index);
  
   
 // Tlist.append(soc.vars);    
  Tlist.append(wr);          
  wr->active = true;  
  soc.setTaskVariables(Tlist);   
 
 //cout << wr->y.reshape(wrsize,wrsize);
      
//wr->espilon_check();   
//soc.setTaskVariables(ARRAY(qit,col));  
 // soc.gl->watch();     
 arr b,Binv,R,r;             
 int cnt;        
 soc.setx(x0);
//OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,0);
cout<<wr->J<<endl;       
double tm;   
//GetOptimalDynamicTime(tm,cnt,b,Binv,soc,5e-2,1e-3,1e-3,1e-4,1e-5,false);
// OneStepKinematic(b,Binv,soc,1e-2,1e-3);

 cout << wr->y.reshape(wrsize,wrsize);
 soc.displayState(&b);
//cout <<b;
  soc.gl->watch();   
    soc.setx(x0);
    AICO aico(soc); 
    aico.iterate_to_convergence();  
    q = aico.q;   
    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
//ofstream out("init2"); wr->y.writeRaw(out); out.close(); 
    /*    q = aico.q[T];    
    soc.setq(q);       
  
            
 // soc.setq0AsCurrent(); 
  soc.displayState(&b);*/
  soc.gl->watch();    
 plot_writhe(wr->y,wrsize);
  return 0;  
  
}


int problem4(){      
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
  gl.camera.setPosition(5,-10,10); 
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 double eps=1e-1; //5e-3;  
 arr q,yy,x0;     
 int wrsize=20;//20;//11
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize,wrsize);              
 
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);     
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y_prec=1e-3;//1e-0;/ / 
  wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;      
   
  Tlist.append(wr);
  soc.setTaskVariables(Tlist);   
   
    
 arr b,Binv,R,r;             
 int cnt;        
 soc.setx(x0);
 OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,false);
// cout<<wr->J<<endl;       
 double tm;   
 soc.displayState(&b);
//cout <<b;
soc.gl->watch();   
    soc.setx(x0);
    AICO aico(soc); 
    aico.iterate_to_convergence();  
    q = aico.q;   
    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
  soc.gl->watch();    
// plot_writhe(wr->y,wrsize);
 
}
       

int problem41(){      
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
  gl.camera.setPosition(5,-10,10); 
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 double eps=1e1; //5e-3;  
 arr q,yy,x0;     
 int wrsize=20;//20;//11
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize,wrsize);              
 
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);    
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
 // wr->y_target =zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
 // wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps); 
  wr->setInterpolatedTargetsConstPrecisions(T,eps,0.);
  //!
    arr y_trajectory; y_trajectory=zeros(T,wrsize*wrsize);
    ifstream inp("writhe_space");  y_trajectory.readRaw(inp); inp.close(); 
    for (int i=0;i<T;i++) { wr->y_trajectory[i]() =y_trajectory[i]();  }
    wr->y_trajectory[50]()=y_trajectory[49]() ;
    wr->y_target=y_trajectory[49]() ;
  //!
    
     TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT; 
  col->y_prec=1e-2;
  col->y_target = ARR(0.);
  col->setInterpolatedTargetsConstPrecisions(T,1e0,0.);
  
  TaskVariable *reach = new DefaultTaskVariable("reach",ors, posTVT,"arm20","<t(0 0 .2)>",0,0,ARR()); //arm20
  arr xtarget;
  xtarget.setCarray(soc.ors->getShapeByName("cyl1")->X.pos.p, 3);
  reach->y_target = xtarget;   
  reach->setInterpolatedTargetsEndPrecisions(T, 1e1, 1e1, 0., 0.);
  //!
    MT::Array<TaskVariable*> Tlist;       
   Tlist.append(wr);
    Tlist.append(col); Tlist.append(reach);
  soc.setTaskVariables(Tlist);   
   
    
 arr b,Binv,R,r;             
 int cnt;           
 soc.setx(x0);
 q.resize(T,ors.getJointStateDimension());
// ifstream inp("q_space");  q.readRaw(inp); inp.close(); 
// soc.recordTrajectory(q,"writhe","writhe_space");
cout << "TADA"<<endl;
 OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,false);
// cout<<wr->J<<endl;       
 double tm;    
 soc.displayState(&b);
//cout <<b;
soc.gl->watch();   
    soc.setx(x0);
    AICO aico(soc); 
    aico.iterate_to_convergence();  
    q = aico.q;  
  //  soc.recordTrajectory(q,"writhe","writhe_space");
 //ofstream out("q_space");  q.writeRaw(out); out.close(); 
    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
   
  soc.gl->watch();    
// plot_writhe(wr->y,wrsize); 
 
}
int problem5(){      
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
  gl.camera.setPosition(5,-10,10); 
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 arr q,yy,x0;     
 int wrsize=20;//20;//11
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize,wrsize);              
 
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);  
   yy.reshape(1,wrsize*wrsize);
   ifstream qitstr("reach"); yy.readRaw(qitstr); qitstr.close(); 
       yy.reshape(wrsize,wrsize);
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y_prec=1e-3;//1e-0;/ / 
  wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =yy;//zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
//wr->y_target =wr->y;
//ifstream in("init2"); wr->y_target.readRaw(in); in.close();  
double eps=1e1; //5e-3;  
 
wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;      
   
  TaskVariable *qit    = new DefaultTaskVariable("qitself", ors, qItselfTVT, 0, 0, 0, 0, 0);
  arr q0; soc.getq0(q0); //q0(wrsize-1)=-0.7; 
  qit->y_target = q0;
  qit->y_prec=1e-2;    
  qit->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);
  
  TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT; 
  col->y_prec=1e-2;
  col->y_target = ARR(0.);
  col->setInterpolatedTargetsConstPrecisions(T,1e1,0.);
  
  TaskVariable *reach = new DefaultTaskVariable("reach",ors, posTVT,"arm20","<t(0 0 .2)>",0,0,ARR()); //arm20
  arr xtarget;
  xtarget.setCarray(soc.ors->getShapeByName("cyl1")->X.pos.p, 3);
  reach->y_target = xtarget;   
  reach->setInterpolatedTargetsEndPrecisions(T, 1e1, 1e1, 0., 0.);
  //reach->setIntervalPrecisions(T,ARR(0.,0.,0.,0.,1e-1),ARR(0.,0.,0.,0.,0.));


 // createMyStandardRobotTaskVariables(soc);
 // setMyGraspGoals(soc,T,soc.ors->getShapeByName("cyl1")->index);
Tlist.append(reach);    
  Tlist.append(wr);
   
 Tlist.append(col);
  wr->active = true;  
  soc.setTaskVariables(Tlist);   
   
//wr->espilon_check();   
// soc.gl->watch();     
 arr b,Binv,R,r;             
 int cnt;        
 soc.setx(x0);
 OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,0);
// ofstream out("reach"); wr->y.writeRaw(out); out.close(); 
// cout<<wr->J<<endl;       
 double tm;   
//GetOptimalDynamicTime(tm,cnt,b,Binv,soc,5e-2,1e-3,1e-3,1e-4,1e-5,false);
// OneStepKinematic(b,Binv,soc,1e-2,1e-3);

 cout << wr->y.reshape(wrsize,wrsize);
 soc.displayState(&b);
//cout <<b;
  soc.gl->watch();   
    soc.setx(x0);
    AICO aico(soc); 
    aico.iterate_to_convergence();  
    q = aico.q;   
    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
//ofstream out("init2"); wr->y.writeRaw(out); out.close(); 
    /*    q = aico.q[T];   
    soc.setq(q);       
  
            
 // soc.setq0AsCurrent();  
  soc.displayState(&b);*/
  soc.gl->watch();    
 //plot_writhe(wr->y,wrsize);
  return 0;  
  
}
//===========================================================================

void problem6(){      
    cout <<"\n= problem 6: dynamic grasping and time optimization =\n" <<endl;
  
  //setup the problem 
  soc::SocSystem_Ors sys_one;// for 1-step              
  soc::SocSystem_Ors sys;   
  OpenGL gl;                       
  arr p,q0;      
  uint dim = 14;   
  arr opt = arr(dim);    
  arr Binv = arr(dim,dim);    
  arr B = arr(dim,dim);                  
    arr r,R,bopt ;       
  uint T=40; //MT::getParameter<uint>("reachPlanTrajectoryLength");
                                                        
  double alpha=MT::getParameter<double>("alpha");
  bool usebwd=MT::getParameter<double>("usebwd") ; 
  double time=1.32;//0.5; // For now - empirical time
  char* obj = "cyl1"; 
  // soc for optimization          
  sys_one.initBasics(NULL,NULL,&gl,T,time,true,NULL); //dynamic!!!
  createStandardRobotTaskVariables(sys_one);    
  setGraspGoals(sys_one,T,obj);           
             
double dr,gr,tm;             
arr kopt=arr(dim);            
arr kBinv,b;                                
// OneStepKinematic(b,Binv,sys_one,1e-5,1e-3);
//  OneStepDynamicFull(b,Binv,sys_one,time,alpha, true); // final posture estimation
 cout<<b;
  //////opt = b;tm=time;
/*GetOptimalDynamicTime(tm,opt,Binv,sys_one,alpha,0.05,1e-2,1);          
cout <<"\nOptimal time =" <<tm<<endl;    
time = tm;              */  
 // time = 1.0;
  //   OneStepDynamicFull(opt,Binv,sys_one,time,alpha);
 // AICO init   
 sys.initBasics(NULL,NULL,&gl,T,time,true,NULL);
 //createStandardRobotTaskVariables(sys);
 setGraspGoals(sys,T,obj);     
  
 AICO solver(sys);  
 solver.init(sys);       
       
/*if (usebwd){   
 inverse_SymPosDef(B,Binv);
 solver.bwdMsg_v = opt;  
 solver.bwdMsg_Vinv.setDiag(1e2,28);     
 solver.useBwdMsg = usebwd; 
}            */
   solver.iterate_to_convergence();
  arr q;
  for (;;)
   sys.displayTrajectory(q,NULL,40,"AICO (planned trajectory)");
 
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
 case 2:  problem2();  break;
  case 3:  problem3();  break;
  case 4:  problem4();  break;
  case 41:  problem41();  break;
  case 5:  problem5();  break;
  case 6:  problem6();  break;
//  case 7:  problem7();  break;
  default: NIY;
  }
  return 0;
}

