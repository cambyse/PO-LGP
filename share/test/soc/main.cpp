#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/util.h>
#include <MT/aico.h>

const char* USAGE="usage: ./x.exe -orsfile test.ors -dynamic 1 -Hcost 1e-3";

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);
  cout <<USAGE <<endl;

  OpenGL gl;
  
  uint T=200;
  soc::SocSystem_Ors sys;
  sys.initBasics(NULL, NULL, &gl, T, 3., MT::getParameter<bool>("dynamic",false), NULL);
  sys.os=&std::cout;
  sys.checkGrad = 1.; //force gradient checks in each call of getTaskCost[Terms]
 

  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position", *sys.ors, posTVT,"endeff","<t(0 0 .2)>",0,0,ARR());
  pos->setGainsAsNatural(20,.2);
  pos->targetType=positionGainsTT;
  pos->y_target = arr(sys.ors->getBodyByName("target")->X.pos.p,3);
  
  TaskVariable *col = new DefaultTaskVariable("collision", *sys.ors, collTVT,0,0,0,0,ARR(.05));
  col->setGains(.5,.0);
  col->targetType=positionGainsTT;
  col->y_prec=1e-0;
  col->y_target = ARR(0.);

  sys.setTaskVariables(ARRAY(pos,col));
  
  //-- feedback control (kinematic or dynamic) to reach the targets
  arr q,dq,qv;
  sys.getq0(q);
  sys.getqv0(qv);
  for(uint t=0;t<10;t++){
    if(!sys.dynamic){
      soc::bayesianIKControl2(sys,q,q,0);
      sys.setq(q);
    }else{
      soc::bayesianDynamicControl(sys,qv,qv,0);
      sys.setqv(qv);
    }
    //soc.reportOnState(cout); //->would generate detailed ouput on the state of all variables...
    sys.gl->update(STRING("bayesian Inverse Kinematics: iteration "<<t));
    //gl.watch();
  }
  sys.gl->watch("<press ENTER>");
  
  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  sys.getq0(q);
  sys.setq(q);
  pos->setInterpolatedTargetsEndPrecisions(T, 1e-3, 1e3, 0., 1e3);
  col->setInterpolatedTargetsConstPrecisions(T, 1e-2, 0.);
  
  q.clear();
#if 0
  AICO_solver(soc,q,1e-2,.7,.01,0,0);
#else
  AICO aico(sys);
  soc::straightTaskTrajectory(sys, q, 0);
  aico.init_trajectory(q);
  aico.iterate_to_convergence();
  q = aico.q;
#endif
  ofstream os("z.traj"); q.writeRaw(os); os.close();
  for(;;) sys.displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
  
  return 0;
}
