#include <MT/socNew.h>
#include <Kin/kin.h>
#include <MT/soc_orsSystem.h> //socSystem_ors.h>
#include <Gui/opengl.h>
#include <Core/util.h>
#include <MT/aico.h>

const char* USAGE="usage: ./x.exe -orsfile test.ors -dynamic 1 -Hrate 1e-0";


void testRobotSystem(bool testFeedbackControl=false){
  OpenGL gl;
  
  double D=mlr::getParameter<double>("time_duration",4.);
  uint T=mlr::getParameter<uint>("time_steps",200);
  OrsSystem sys;
  sys.initBasics(NULL, NULL, &gl, T, D, mlr::getParameter<bool>("dynamic",false), NULL);
  sys.os=&std::cout;
 
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position", sys.getOrs(), posTVT,"endeff","<t(0 0 .2)>",0,0,arr());
  pos->y_target = conv_vec2arr(sys.getOrs().getBodyByName("target")->X.pos);
  
  TaskVariable *col = new DefaultTaskVariable("collision", sys.getOrs(), collTVT,0,0,0,0,ARR(.15));
  col->y_prec=1e-0;
  col->y_target = ARR(0.);

  TaskVariable *qtv = new DefaultTaskVariable("qitself", sys.getOrs(), qItselfTVT, 0, 0, 0, 0, 0);
  qtv->y_target.setZero();

  sys.setTaskVariables({pos,col,qtv});
  
#if 0
  arr q,dq,x;
  if(testFeedbackControl){
    //-- feedback control (kinematic or dynamic) to reach the targets
    sys.getq0(q);
    sys.getx0(x);
    pos->setGainsAsNatural(20,.2);
    pos->targetType=positionGainsTT;
    col->setGains(.5,.0);
    col->targetType=positionGainsTT;
    
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
#endif
  
  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  arr x;
  sys.get_x0(x);
  sys.setx(x);
  pos->setInterpolatedTargetsEndPrecisions(T, 1e-3, 1e3, 0., 1e-3);
  col->setInterpolatedTargetsConstPrecisions(T, 1e-2, 0.);
  qtv->setInterpolatedTargetsEndPrecisions(T, 0., 0., 1e-2, 1e4);

  x.clear();

//  sys.checkGrad = 1.; //force gradient checks in each call of getTaskCost[Terms]
  //soc::straightTaskTrajectory(sys, q, 0);

  AICO aico(sys);
  aico.init_messages();
  //aico.init_trajectory(q);
  aico.iterate_to_convergence();
  //sys.costChecks(aico.b);
  analyzeTrajectory(sys, aico.b(),true, &cout);
  x = aico.b();
  ofstream os("z.traj"); x.writeRaw(os); os.close();
  displayTrajectory(sys, x, NULL,1,"AICO (planned trajectory)");

  //test iterated optimization with changing goals/initial condition
  for(uint k=0;k<10;k++){
    pos->y_target(2) += .1;
    pos->setInterpolatedTargetsEndPrecisions(T, 1e-3, 1e3, 0., 1e-3);

//    cout <<"** from scratch with previous" <<endl;
//    AICO aico2(sys);
//    aico2.init_trajectory(q);
//    aico2.iterate_to_convergence();
//    sys.analyzeTrajectory(aico2.b(),true);
//    sys.displayTrajectory(aico2.q(),NULL,1,"AICO (planned trajectory)");

    cout <<"** updated" <<endl;
    aico.prepare_for_changed_task();
    aico.iterate_to_convergence();
    analyzeTrajectory(sys, aico.b(), true, &cout);
    displayTrajectory(sys, aico.b(), NULL, 2, "AICO_replanned (planned trajectory)");

//    cout <<"** from scratch" <<endl;
//    //from scratch
//    //arr tmp;
//    AICO aico1(sys);
//    //soc::straightTaskTrajectory(sys, tmp, 0);
//    //aico1.init_trajectory(tmp);
//    aico1.iterate_to_convergence();
//    sys.analyzeTrajectory(aico1.b(),true);
//    sys.displayTrajectory(aico1.q(),NULL,1,"AICO (planned trajectory)");

//    q=aico1.q();
  }
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  cout <<USAGE <<endl;

  testRobotSystem();

  return 0;
}
