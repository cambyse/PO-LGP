#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/util.h>
#include <MT/aico.h>

const char* USAGE="usage: ./x.exe -orsfile test.ors -dynamic 1 -Hrate 1e-0";


void testRobotSystem(bool testFeedbackControl=false){
  OpenGL gl;
  
  double D=MT::getParameter<double>("time_duration",4.);
  uint T=MT::getParameter<uint>("time_steps",200);
  soc::SocSystem_Ors sys;
  sys.initBasics(NULL, NULL, &gl, T, D, MT::getParameter<bool>("dynamic",false), NULL);
  sys.os=&std::cout;
 
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position", *sys.ors, posTVT,"endeff","<t(0 0 .2)>",0,0,ARR());
  pos->setGainsAsNatural(20,.2);
  pos->targetType=positionGainsTT;
  pos->y_target = arr(sys.ors->getBodyByName("target")->X.pos.p,3);
  
  TaskVariable *col = new DefaultTaskVariable("collision", *sys.ors, collTVT,0,0,0,0,ARR(.15));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT;
  col->y_prec=1e-0;
  col->y_target = ARR(0.);

  TaskVariable *qtv = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  qtv->y_target.setZero();

  sys.setTaskVariables(ARRAY(pos,col,qtv));
  
  arr q,dq,x;
  if(testFeedbackControl){
    //-- feedback control (kinematic or dynamic) to reach the targets
    sys.getq0(q);
    sys.getx0(x);
    for(uint t=0;t<10;t++){
      if(!sys.dynamic){
        soc::bayesianIKControl2(sys,q,q,0);
        sys.setq(q);
      }else{
        soc::bayesianDynamicControl(sys,x,x,0);
        sys.setx(x);
      }
      //soc.reportOnState(cout); //->would generate detailed ouput on the state of all variables...
      sys.gl->update(STRING("Inverse Kinematics: iteration "<<t));
      //gl.watch();
    }
    //sys.gl->watch("IK solution <press ENTER>");
  }
  
  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  sys.getq0(q);
  sys.setq(q);
  pos->setInterpolatedTargetsEndPrecisions(T, 1e-3, 1e3, 0., 1e-3);
  col->setInterpolatedTargetsConstPrecisions(T, 1e-2, 0.);
  qtv->setInterpolatedTargetsEndPrecisions(T, 0., 0., 1e-2, 1e4);

  q.clear();

#if 0
  AICO_solver(soc,q,1e-2,.7,.01,0,0);
#else 

  //sys.checkGrad = 1.; //force gradient checks in each call of getTaskCost[Terms]
  AICO aico(sys);
  soc::straightTaskTrajectory(sys, q, 0);
  aico.init_trajectory(q);
  aico.iterate_to_convergence();
  //sys.costChecks(aico.b);
  sys.analyzeTrajectory(aico.b,true);
  q = aico.q;
#endif
  ofstream os("z.traj"); q.writeRaw(os); os.close();
  for(;;) sys.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");

}


int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);
  cout <<USAGE <<endl;

  testRobotSystem();

  return 0;
}
