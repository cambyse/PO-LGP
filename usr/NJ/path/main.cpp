#define MT_IMPLEMENTATION
#include <MT/util.h>
#include <MT/ors.h>

#include <MT/motion_planning.h>
#include <MT/joystick.h>


void testTracking(){

  uint T=200;

  ors::Graph ors;
  MT::load(ors,"../../../share/configurations/schunk.ors",true);
  ors.calcNodeFramesFromEdges();
  
  OpenGL gl;
  gl.add(glStandardScene,0);
  gl.add(glDrawSlGraph,&ors);
  //gl->setClearColors(1.,1.,1.,1.);
  gl.s->camera.setPosition(7.,-0.,2.);
  gl.s->camera.focus(0,0,.8);
  gl.watch();

  SwiftInterface swift;
  swift.init(ors);
  
  mop::OrsMopInterface mop;
  //mop::createKinematic(mop,ors,swift,gl,T);
  mop::createPseudoDynamic(mop,ors,swift,gl,2.,T);
  mop.os=&std::cout;

  double midPrec=1e2, endPrec=1e4, colPrec=1e6, limPrec=1e6;
  
  ControlVariable x("endeffector",ors,posCVT,"m9",0,0,0,0);
  x.updateState();
  x.y_target=ARR(.4,.1,1.);
  x.setInterpolatedTargetTrajectory(mop.s->T);
  x.setPrecisionTrajectoryFinal(mop.s->T,midPrec,endPrec);
  x.setPrecisionVTrajectoryConstant(mop.s->T,0.);

  ControlVariable c("collision", ors,collCVT,0,0,0,0,0);
  c.updateState();
  c.y_target = 0.;
  c.setInterpolatedTargetTrajectory(mop.s->T);
  c.setPrecisionTrajectoryConstant(mop.s->T,colPrec);
  c.setPrecisionVTrajectoryConstant(mop.s->T,0.);
  if(!colPrec) c.active=false;

  arr limits;
  limits <<"[-3.2 3.2; -3.2 3.2; -3.2 .2; -3.2 3.2; -3.2 .2; -3.2 3.2; -2. 2. \
      -.2 .2; -.2 .2; -.2 .2; -.2 .2; -.2 .2; -.2 .2; -.2 .2; -.2 .2; -.2 .2 ]";
      
  ControlVariable lim("limits", ors, qLimitsCVT,0,0,0,0,&limits);
  lim.updateState();
  lim.y_target = 0.;
  lim.setInterpolatedTargetTrajectory(mop.s->T);
  lim.setPrecisionTrajectoryConstant(mop.s->T,limPrec);
  lim.setPrecisionVTrajectoryConstant(mop.s->T,0.);
  if(!limPrec) lim.active=false;

  arr I; I.setId(ors.getJointStateDimension());
  ControlVariable qc("qmotion", ors, qLinearCVT,0,0,0,0,&I);
  qc.updateState();
  qc.y_target = 0.;
  qc.v_target = 0.;
  qc.setInterpolatedTargetTrajectory(mop.s->T);
  qc.setPrecisionTrajectoryConstant(mop.s->T,0.);
  qc.setPrecisionVTrajectoryConstant(mop.s->T,0.);
  qc.vprec_trajectory(0)=endPrec;
  qc.vprec_trajectory(100)=endPrec;
  qc.vprec_trajectory(T-1)=endPrec;
  
  
  arr q;
  //mop::bayesianIKTrajectory(mop,q);
  //mop::bayesianIKTrajectory(mop,q,1e-4);
  //mop.displayTrajectory(q,1,"initially planned");
  double rate=MT::getParameter<double>("rate");
  double threshold=MT::getParameter<double>("threshold");
  uint display=MT::getParameter<uint>("display");
  uint iterations=MT::getParameter<uint>("iterations");
  mop.dynamic=false;
  mop::BayesianKinematicMotionPlanning(mop,q,iterations,rate,threshold,display);
  plotClear();
  plotFunctions(q);
  plot();
  mop.dynamic=true;
  //mop::BayesianMotionInference(mop,q,iterations,rate,threshold,display);
  mop::iLQG_dynamic(mop,q,iterations,rate,display);
  plotClear();
  plotFunctions(q);
  plot();
  
}













#include <signal.h>

#include <stdio.h>
#include <termios.h>
#include <term.h>
#include <curses.h>
#include <unistd.h>
static struct termios clInitTerm, clNewTerm;
static int iPeekChar = -1;


using namespace SDH;

cSDH *global_sdh=NULL;

void STOP(int){
  if(global_sdh) global_sdh->EmergencyStop();
  global_sdh->Close();
  cerr <<"caught signal -- STOPPED SDH HAND" <<endl;
}



void drawBase(void*){
  glStandardLight();
  glDrawFloor(10,.8,.8,.8);
  glColor(1.,.5,0.);
}

void loadOrsFile(ors::Graph& C, OpenGL& gl,const char *file="../../../share/configurations/schunk.ors"){
  char *path,*name,cwd[200];
  MT::decomposeFilename(path,name,file);
  getcwd(cwd,200);
  chdir(path);
  
  gl.add(drawBase,0);
  gl.add(glDrawSlGraph,&C);
  //gl->setClearColors(1.,1.,1.,1.);
  gl.s->camera.setPosition(7.,-0.,2.);
  gl.s->camera.focus(0,0,.8);

  MT::load(C,name);
  C.calcNodeFramesFromEdges();

  chdir(cwd);
}


void testSchunk(){
  SchunkModule schunk;
  schunk.openLWA();
  testCube( schunk, 9 );

  printf( "\nClosing device returned %d\n", schunk.pDev->exit() );
  printf( "That's all folks!\n" );
}

void testControl(){
  OpenGL gl;
  ors::Graph ors;
  loadOrsFile(ors,gl);
  gl.watch();

  SchunkModule schunk;
  schunk.openLWA();

  uintA motorIndex(7);
  motorIndex(0) = ors.getName("m3")->inLinks(0)->index;
  motorIndex(1) = ors.getName("m4")->inLinks(0)->index;
  motorIndex(2) = ors.getName("m5")->inLinks(0)->index;
  motorIndex(3) = ors.getName("m6")->inLinks(0)->index;
  motorIndex(4) = ors.getName("m7")->inLinks(0)->index;
  motorIndex(5) = ors.getName("m8")->inLinks(0)->index;
  motorIndex(6) = ors.getName("m9")->inLinks(0)->index;

  uint t;
  arr q,dq;
  ors.getJointState(q);
  
  JoystickInterface joy;
  joy.open();

  arr qm(7);
  schunk.getPos(qm);
  for(uint m=0;m<7;m++) q(motorIndex(m)) = qm(m);
  cout <<t <<" schunk position = " <<qm <<endl;
  ors.setJointState(q);
  ors.calcNodeFramesFromEdges();
  gl.watch();

  TaskVariable x("endeffector",ors,posCVT,"m9",0,0,0,0);
  x.setAttractorDynamics(20,.2);
  x.y_prec=1000.;
  updateState(globalSpace); //sets the target to CURRENT STATE
  //x.y_target=ARR(0,-.4,1.);

  arr W;
  ors.computeNaturalQmetric(W);
  
  for(t=0;t<100000;t++){
    joy.step();
    
    x.y_target(0)-=0.005*((double)(joy.state(3)/2))/128.;
    x.y_target(1)+=0.005*((double)(joy.state(6)/2))/128.;
    x.y_target(2)-=0.005*((double)(joy.state(2)/2))/128.;
    cout <<t <<" target=" <<x.y_target <<flush;
    
    updateState(globalSpace);
    computeXchangeWithAttractor(globalSpace);
    bayesianControl(globalSpace,dq,W);
    q += dq;

    for(uint m=0;m<7;m++){
      schunk.pDev->setMaxVel(m+3, .1);
      schunk.pDev->setMaxAcc(m+3, .1);
      //schunk.pDev->moveRamp(m+3, q(motorIndex(m)));
      schunk.pDev->moveStep(m+3, q(motorIndex(m)), 300);
    }
    
    schunk.getPos(qm);
    cout <<" real_q=" <<qm <<flush;
    
    ors.setJointState(q);
    ors.calcNodeFramesFromEdges();

    gl.text.clear() <<"time " <<t <<endl;
    gl.update();
    //if(x.state==1) break;
    cout <<std::setprecision(2) <<MT::timerRead(true) <<"secs" <<endl;
  }
  gl.watch();
  schunk.zeroVelAll();
  schunk.closeLWA();
}

int main(int argc,char** argv){
  //testSchunk();
  //testControl();
  testTracking();

  return 0;
}

