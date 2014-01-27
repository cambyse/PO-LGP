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


void createMyStandardRobotTaskVariables(soc::SocSystem_Ors& sys){
  TaskVariable *TV_q    = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  TaskVariable *TV_z1   = new DefaultTaskVariable("oppose12", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  TaskVariable *TV_z2   = new DefaultTaskVariable("oppose13", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
  TaskVariable *TV_f1   = new DefaultTaskVariable("pos1", *sys.ors, posTVT, "tipHook1", 0, 0);
  TaskVariable *TV_f2   = new DefaultTaskVariable("pos2", *sys.ors, posTVT, "tipHook2", 0, 0);
  TaskVariable *TV_f3   = new DefaultTaskVariable("pos3", *sys.ors, posTVT, "tipHook3", 0, 0);
   TaskVariable *TV_up   = new DefaultTaskVariable("up1", *sys.ors, zalignTVT, "arm20", "<d(90 0 1 0)>", 0, 0, 0);
  TaskVariableList TVs;
  TVs.append(ARRAY( TV_z1, TV_z2, TV_f1, TV_f2, TV_f3,TV_up,TV_q));
  sys.setTaskVariables(TVs);
}

void setMyGraspGoals(soc::SocSystem_Ors& sys, uint T,double endPrec){
  sys.setx0ToCurrent();
  
  //load parameters only once!
 
 static double midPrec = 0.;
 // endPrec = 1e1;
  //set the time horizon
  //activate collision testing with target shape
   arr xtarget;
  xtarget.setCarray(sys.ors->getShapeByName("cyl1")->X.pos.p, 3);
  
  TaskVariable *V;
  
  //general target
  
  xtarget(2) += .01; //grasp it 2cm above center
   V=listFindByName(sys.vars, "up1");
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 0 1 0)>");
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
   V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, 0., 0., 1e-1, 0.);
 }


int get_index(const arr& y){

  
 // for (uint i=y.N;i>1;i--)
   //  if (y(i-1)>mean(y)) return i;  
    return y.maxIndex();  
  
}

void sine_trajectory(const arr& q,arr& sine_q,const uint _N)
{
  uint N=_N;
  sine_q.resize(N,q.d1);
  arr l_metric;
  double l,coef;
  double l_max = 100;
  const double PI  = 3.141592653589793238;
  //!metric
  l_metric.resize(q.d0,1);l_metric.reshape(q.d0);
  l_metric(0)=.0;
   for (uint i=1;i<q.d0;i++) 
    l_metric(i) = l_metric(i-1) + norm(q[i]()-q[i-1]());
  l_max = l_metric(q.d0-1);  
  //!end of metric
//  sine_q.reshape(q.d1,N);
    uint j=1;

  for (uint i=0;i<N;i++){ 
    l = 0.5 *(1.- cos(PI* i/N ))*l_max;  
    while (l>l_metric(j)) j++;
    coef = (l - l_metric(j-1)) /(l_metric(j)-l_metric(j-1));
    sine_q[i]() = q[j-1]() + (q[j]()-q[j-1]())*coef;
  }
    
    
  
}

int problem4(){      
  ors::KinematicWorld ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));//e.g._test
  bool usebwd=MT::getParameter<double>("usebwd") ;
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
 // gl.camera.setPosition(5,-10,10);  
   gl.camera.setPosition(5,-10,5);  
  
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 double eps=1e-1; //1e-1  
 arr q,yy,x0;     
 int wrsize1=20;//20;//11        
 int wrsize2=1;
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize1,wrsize2);              
 
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);     
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
         
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize1,wrsize2,1);
  wr->y_prec=eps;//1e-0;/ / 
   wr->y.reshape(wrsize1,wrsize2);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =zeros(wrsize1,wrsize2);//wr->y;//zeros(10,10);//yy;   zeros(1,1); 
//   wr->y_target =zeros(wrsize,1);//wr->y;//zeros(10,10);//yy;   zeros(1,1); 
   
  for (int tp=0;tp< 10;tp++)  wr->y_target(tp,0) =   wr->y(tp,0);
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,0.);  

  MT::Array<TaskVariable*> Tlist;        
      
  //!col 
       TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT; 
  col->y_prec=1e-2; 
  col->y_target = ARR(0.);  
  col->setInterpolatedTargetsConstPrecisions(T,1e0,0.);//1e0
  //! END of col 
    TaskVariable *reach = new DefaultTaskVariable("reach",ors, posTVT,"arm20","<t(0 0 -.36)>",0,0,ARR()); //arm20 -0.26
  arr xtarget;
  xtarget.setCarray(soc.ors->getShapeByName("cyl1")->X.pos.p, 3);
  reach->y_target = xtarget;   
  reach->setInterpolatedTargetsEndPrecisions(T, 0., 1e1, 0., 0.);//1e1
 // arr pr = ARRAY(0.,0.,0.,0.,1e0);  
  //reach->setIntervalPrecisions(T,pr,pr); 
  //! end of reach
 // Tlist.append(wr);
  Tlist.append(col);   
  Tlist.append(reach);
 // createMyStandardRobotTaskVariables(soc);
 // setMyGraspGoals(soc,T,1e0);
  Tlist.append(soc.vars);
  soc.setTaskVariables(Tlist);   

 arr b,Binv,R,r;               
 int cnt;        
 soc.setx(x0);
 //plot_writhe(wr->y,wrsize1,wrsize2);
 OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,false);
// cout<<wr->J<<endl;       
 double tm;   
//b = x0; //for (uint i=20-1;i>20-20;i--) b(i)=-0.5;

// ofstream out("b_space");  b.writeRaw(out); out.close(); 
 //
 soc.setx(b); 
 wr->userUpdate();
 // plot_writhe(wr->y,wrsize1,wrsize2);

  
 soc.gl->watch();   
  //! One step method for writhe
   MT::Array<TaskVariable*> Alist;        
  wr->y_target = wr->y; 
  
  int ind = get_index(wr->y);
  cout<< ind; 
 wr->y_target.setZero();
 for (uint i=0;i<10;i++) wr->y_target(i) = wr->y(i); //!For reaching demo
  //ind=5; 
  b=x0;
for (uint i=20-1;i>20-20;i--) b(i)=-0.5; soc.setx(b);soc.setx0ToCurrent();
 soc.displayState(&b);
 soc.gl->watch();    
wr->userUpdate();
//wr->y_target = wr->y; 
soc.setx(x0);soc.setx0ToCurrent();
wr->userUpdate();
// for (int tp=0;tp<ind;tp++)  wr->y_target(tp) =   wr->y(tp);
  //  wr->y_target.reshape(20);
  //  for (int tp=wr->y.N-1;tp>ind;tp--)  wr->y_target(tp) =  0.;
     
 //  eps=1e1;
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,0.);  
//! plot trajcetory from file
//  arr tr_wr;tr_wr.resize(wrsize1,T);
//  ifstream qitstr("rss_new"); tr_wr.readRaw(qitstr); qitstr.close(); //new_wr full_unwr
//   ifstream qitstr("rss_trajectory"); tr_wr.readRaw(qitstr); qitstr.close(); //new_wr full_unwr
//   arr fe = tr_wr;
//   arr temp ;
//   fe.reshape(20,T); temp.resize(20,T);
//   for (uint i=0;i<20;i++){
//     for (uint j=0;j<T;j++)
//     temp(i,j) = fe(20-i-1,j);
//   }
//   plot_writhe(temp,wrsize1,T);
//   ofstream oitstr("rss_trajectory_final"); temp.writeRaw(oitstr); oitstr.close(); //new_wr full_unwr
  //plot_writhe(~tr_wr,wrsize1,T);
  //! end of plotting
//   for (uint i=0; i<wrsize1;i++)  
//     for (uint j=0; j<i*2;j++)  
//     wr->y_trajectory(T-j,i)=0;
 // plot_writhe(~wr->y_trajectory,wrsize1,T+1);
  //!end of diagonal exp
 // for (uint u=40;u<T;u++)  wr->y_prec_trajectory(u)=0.;
  
  Alist.append(col);
  Alist.append(wr);
  reach->setInterpolatedTargetsEndPrecisions(T, 0., 1e1, 0., 0.);//1e1
 
  //!reach
     Alist.append(reach);
    createMyStandardRobotTaskVariables(soc);
    setMyGraspGoals(soc,T,1e1);
    Alist.append(soc.vars);
  //!end of reach
  soc.setTaskVariables(Alist);   
  //! End of one step
//cout <<b;

    soc.setx(x0); 
    AICO aico(soc);
      
    //! AICO
    aico.iterate_to_convergence();  
    q = aico.q;   
//! intepolated traj 
    arr sine_q;
    uint N=100; 
    sine_trajectory(q,sine_q,N);
    soc.initBasics(&ors,&swift,&gl,N-1,2.0,true,&ones(jsize,1).reshape(jsize) );
    for (;;) soc.displayTrajectory(sine_q,NULL,1,"AICO (planned trajectory)");
//      //     ofstream out("traj_test");  sine_q.writeRaw(out); out.close(); 
//! end of inter
 //   plot_writhe(sine_q,20,500);
// soc.recordTrajectory(q,"writhe","rss_new");
  // for (;;)  soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
    //! end of AICO
 
    
  
//    arr tr_q; tr_q.resize(21,20);  
   // ifstream qitstr("q.rrt"); tr_q.readRaw(qitstr);  qitstr.close(); 
  //! Replaying RRT trajectory here
//     arr tr_q; tr_q.resize(21,20); 
//   tr_q <<FILE("q.rrt");
//   soc.totalCost(NULL,tr_q, 1);
//   for (;;)  soc.displayTrajectory(tr_q,NULL,20,"AICO (planned trajectory)");
  //! end of REPLAY
  //ofstream out("cyl1.pos");  q[T].writeRaw(out); out.close(); 

      soc.gl->watch();    
//  plot_writhe(wr->y,wrsize);
 
}
       

int problem41(){      
  ors::KinematicWorld ors;  
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
 soc.initBasics(&ors,&swift,&gl,T,1.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);     
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,wrsize,1);
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


int problem42(){      
  ors::KinematicWorld ors;  
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
 soc.initBasics(&ors,&swift,&gl,T,1.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
  
  soc.getx0(x0);     
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,wrsize,1);
  wr->y_prec=eps;//1e-0;/ / 
  //wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =zeros(1,1); //zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   zeros(1,1); 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;        
  
  //!col 
       TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT; 
  col->y_prec=1e-2;
  col->y_target = ARR(0.);
  col->setInterpolatedTargetsConstPrecisions(T,1e0,0.);
  //! END of col
   TaskVariable *reach = new DefaultTaskVariable("reach",ors, posTVT,"arm20","<t(0 0 .2)>",0,0,ARR()); //arm20
  arr xtarget;
  xtarget.setCarray(soc.ors->getShapeByName("cyl1")->X.pos.p, 3);
  reach->y_target = xtarget;   
  reach->setInterpolatedTargetsEndPrecisions(T, 1e1, 1e1, 0., 0.);
  //! end of reach
  Tlist.append(wr);Tlist.append(col);//Tlist.append(reach);
  soc.setTaskVariables(Tlist);  
  
  //! delta check
//   arr q0;soc.getq0(q0);
// q=q0; 
//   cout << "initial state"<<x0<<endl;  
//   arr delta_q; 
//   for (int i=0;i<100;i++){ 
//     wr->delta_check(delta_q);
//    // wr->epsilon_check(delta_q);
//     q+=delta_q;
//     soc.setq(q);
//     soc.displayState(&q);
//     //soc.gl->watch();
//   } 
  //! end of check           
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

int problem43(){      
  ors::KinematicWorld ors;  
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
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,wrsize,1);
  wr->y_prec=1e-3;//1e-0;/ / 
 // wr->y.reshape(1,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =zeros(1,wrsize);//wr->y;//zeros(10,10);//yy;   zeros(1,1); 
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;        
  
  //!col
 /*      TaskVariable *col = new DefaultTaskVariable("collision",ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT; 
  col->y_prec=1e-2;
  col->y_target = ARR(0.);
  col->setInterpolatedTargetsConstPrecisions(T,1e0,0.);*/
  //! END of col
  Tlist.append(wr);//Tlist.append(col);
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

int problem5(){      
  ors::KinematicWorld ors;  
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
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,wrsize,1);
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


int problem6(){      
  ors::KinematicWorld ors;  
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
 for (int z=0;z<jsize;z++) ID(z,0)= jsize*(1.0-z/jsize);//11-z;//z+1;

 soc.initBasics(&ors,&swift,&gl,T,1.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
// soc.initBasics(&ors,&swift,&gl,T,1.0,true,&ID.reshape(jsize)); //Fix time
  
  soc.getx0(x0);     
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
        
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,wrsize,1);
  wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
 // wr->y_target =zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
 // wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps); 
  wr->setInterpolatedTargetsConstPrecisions(T,eps,0.); 
  //!
    arr y_trajectory; y_trajectory=zeros(T,wrsize*wrsize);
    ifstream inp("writhe_space");  y_trajectory.readRaw(inp); inp.close(); 
     
    int mid_traj = 40;
    for (int i=0;i<mid_traj;i++) { wr->y_trajectory[i]() =y_trajectory[i]();  }
    for (int i=mid_traj;i<T+1;i++) { wr->y_trajectory[i]() =y_trajectory[mid_traj-1]();  }
    
//    wr->y_trajectory[50]()=y_trajectory[49]() ;
    wr->y_target=y_trajectory[mid_traj-1]() ;
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
 
///   wr->epsilon_check();
soc.gl->watch();   
    
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
//    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
   
  soc.gl->watch();    
// plot_writhe(wr->y,wrsize); 
 
} 
//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 4:  problem4();  break;
  case 41:  problem41();  break;
  case 42:  problem42();  break;
  case 43:  problem43();  break;
  case 5:  problem5();  break;
  case 6:  problem6();  break;
//  case 7:  problem7();  break;
  default: NIY;
  }
  return 0;
}

