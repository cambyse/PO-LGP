#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/soc.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/aico.h>

using namespace soc;

void drawEnv(void*){
  glStandardLight(NULL);
  //glDrawFloor(4.,1,1,1);
}
void init(ors::Graph& ors,OpenGL& gl,const char *filename){
  ors.init(filename);
  
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  //gl.setPosition(.0,0.,10.);
  //gl.focus(.0,0,0);
}

void defineReachControlVariables(SocSystem_Ors& soci,ors::Graph& ors,uint T){
  //set task variables
  TaskVariable *x0 = new TaskVariable("finger-tip",ors,posTVT ,"effector","<t(0 0 .2)>",0,0,0);
  TaskVariable *x1 = new TaskVariable("collision", ors,collTVT,0,0,0,0,ARR(.05));
  soci.setTaskVariables(ARRAY(x0,x1));

  double midPrec,endPrec,balPrec,colPrec;
  MT::getParameter(midPrec,"midPrec");
  MT::getParameter(endPrec,"endPrec");
  MT::getParameter(balPrec,"balPrec");
  MT::getParameter(colPrec,"colPrec");

  x0->y_target.setCarray(ors.getBodyByName("target")->X.pos.p,3);
  x0->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);

  
  x1->y_target = 0.;
  x1->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  if(!colPrec) x1->active=false;
}

void testSoc(){
  ors::Graph ors;
  SwiftInterface swift;
  OpenGL gl;
  SocSystem_Ors soci;
  init(ors,gl,"ors/graspTest.ors");
  swift.init(ors);
  //soci.initKinematic(&ors,&swift,&gl,200);
  soci.initBasics(&ors,&swift,&gl,200,1.,false,NULL);
  defineReachControlVariables(soci,ors,200);
  soci.os=&cout;

  arr q;
  //bayesianIKTrajectory(soci,q);
  NIY;
  soci.displayTrajectory(q,NULL,20,"");

  //AICO(soci,q,1e-2,.9,.1,.0,20);
  AICO solver(soci);
  solver.iterate_to_convergence();
  //iLQG_kinematic(soci,q,10,.9,20);
  gl.watch();
}

/*struct soc::OrsSocWorkspace{
  ors::Graph *ors;
  SwiftModule *swift;
  ControlVariableList CVlist;
  
  uint T;
  arr q0,v0,W,H,Q,v_act;
  double tau;
  bool pseudoDynamic;
};*/

void testGradients(){
  ors::Graph ors;
  SwiftInterface swift;
  OpenGL gl;
  SocSystem_Ors soci;
  init(ors,gl,"ors/graspTest.ors");
  ors.getBodyByName("finga2")->shapes(0)->contactOrientation.set(0, 1,0);
  ors.getBodyByName("fingb2")->shapes(0)->contactOrientation.set(0,-1,0);
  swift.init(ors,.05);
  //soci.initKinematic(&ors,&swift,&gl,200);
  soci.initBasics(&ors,&swift,&gl,200,1.,false,NULL);
  soci.os=&cout;

  TaskVariable *arm = new TaskVariable("arm",ors,posTVT,"effector","<t(0 0 .2)>",0,0,0);
  TaskVariable *n1  = new TaskVariable("normal1",ors,zoriTVT,"finga2","<d(-90 1 0 0)>",0,0,0);
  TaskVariable *n2  = new TaskVariable("normal2",ors,zoriTVT,"fingb2","<d( 90 1 0 0)>",0,0,0);
  TaskVariable *p1  = new TaskVariable("pos1",ors,posTVT,"finga2","<t(0 -.065 .02)>",0,0,0);
  TaskVariable *p2  = new TaskVariable("pos2",ors,posTVT,"fingb2","<t(0  .065 .02)>",0,0,0);
  TaskVariable *col = new TaskVariable("collision", ors,collTVT,0,0,0,0,ARR(.05));
  soci.setTaskVariables(ARRAY(arm,n1,n2,p1,p2,col));
  
  //n1->setGains(.001,0.);
  //n2->setGains(.001,0.);
  p1->setGains(.1,0.);
  p2->setGains(.1,0.);
  //col->setGains(1.,0.);
  //col->prec=1e2;

  uint t,T=200;
  arr q,target,org,dq,d;
  org = arm->y;
  target.setCarray(ors.getBodyByName("target")->X.pos.p,3);
  soci.getq0(q);
  for(t=0;t<T;t++){ //fwd
    p1->y_target = target;
    p2->y_target = target;
    
    d=p1->y-target; d/=norm(d);
    n1->y_target = d;
    d=p2->y-target; d/=norm(d);
    n2->y_target = d;
    
    col->y_target = 0.;
    
    dq=q;
    bayesianIKControl2(soci,q,dq,0);
    //q += dq;
    soci.setq(q);
    cout <<t <<' ' <<col->y <<endl;
    gl.watch();
  }
  gl.watch();

  n1->active=false;
  n2->active=false;
  p1->active=false;
  p2->active=false;
  arm->setGains(.02,0.);
  col->setGains(1.,0.);
  col->y_prec=1e2;
  for(t=0;t<T;t++){//bwd
    arm->y_target = org;
    col->y_target = 0.;
    
    dq = q;
    bayesianIKControl2(soci,q,dq,0);
    //q += dq;
    soci.setq(q);
    cout <<t <<' ' <<col->y <<endl;
    gl.watch();
  }
  gl.watch();

}

void testPlan(){
  ors::Graph ors;
  SwiftInterface swift;
  OpenGL gl;
  SocSystem_Ors soci;
  init(ors,gl,"ors/graspTest.ors");
  swift.init(ors,.05);
  ors.getBodyByName("finga2")->shapes(0)->contactOrientation.set(0, 1,0);
  ors.getBodyByName("fingb2")->shapes(0)->contactOrientation.set(0,-1,0);
  //createKinematic(soci,&ors,&swift,&gl,200);
  //soci.initPseudoDynamic(&ors,&swift,&gl,2.,200);
  soci.initBasics(&ors,&swift,&gl,200,2.,true,NULL);
  soci.os=&cout;
  gl.watch();

  uint T=200;
  double midPrec,endPrec,balPrec,colPrec;
  MT::getParameter(midPrec,"midPrec");
  MT::getParameter(endPrec,"endPrec");
  MT::getParameter(balPrec,"balPrec");
  MT::getParameter(colPrec,"colPrec");
  
  TaskVariable *n1 = new TaskVariable("normal1",ors,zoriTVT,"finga2","<d(-90 1 0 0)>","fingb2","<d( 90 1 0 0)>",0);
  TaskVariable *n2 = new TaskVariable("normal2",ors,zoriTVT,"fingb2","<d( 90 1 0 0)>",0,0,0);
  TaskVariable *za = new TaskVariable("align",ors,zalignTVT,"finga2","<d(-90 1 0 0)>","fingb2","<d( 90 1 0 0)>",0);
  TaskVariable *p1 = new TaskVariable("pos1",ors,posTVT,"finga2","<t(0 -.065 .02)>",0,0,0);
  TaskVariable *p2 = new TaskVariable("pos2",ors,posTVT,"fingb2","<t(0  .065 .02)>",0,0,0);
  TaskVariable *col = new TaskVariable("collision", ors,collTVT,0,0,0,0,ARR(.05));
  soci.setTaskVariables(ARRAY(n1,n2,za,p1,p2,col));

  //n1->x_target = -n2->x; //ARR(1.,0.,0.);
  //n2->x_target = -n1->x;
  arr target;
  target.setCarray(ors.getBodyByName("target")->X.pos.p,3);
  p1->y_target = target;
  p2->y_target = target;

  n1->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  n2->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);

  n1->active=n2->active=false;
  
  za->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  
  p1->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  p2->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);

  col->y_target = 0.;
  col->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  if(!colPrec) col->active=false;
  

  arr q;
  //BayesianKinematicMotionPlanning(soci,q,30,.7,.001,1);
  //AICO_dynamic(soci,q,30,.7,.001,1);
  //AICO_solver(soci,q,1e-2,.7,.001,.0,1);
  AICO solver(soci);
  solver.iterate_to_convergence();
  gl.watch();
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  uint mode=MT::getParameter<uint>("mode",(uint)2);
  switch(mode){
    case 0:  testSoc();  break;
    case 1:  testGradients();  break;
    case 2:  testPlan();  break;
    default: NIY;
  }

  return 0;
}
