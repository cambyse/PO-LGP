#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>

//===========================================================================

void problem1(){
  cout <<"\n= problem 1: simple kinematic reaching with 1 TV =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("trajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,3.,false,NULL);
  
  //setup the task
  TaskVariable *pos = new TaskVariable("position" , *sys.ors, posTVT, "palmCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,0.);

  soc::SocSolver solver;
  solver.init();

#if 1
  cout <<"\n== first test: 1 step planning ==\n" <<endl;
  sys.setTimeInterval(3.,1);
  sys.setToq0();
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,10*1e4);
  solver.q.clear();  solver.method=soc::SocSolver::AICO;               solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::LQG_straightInit;   solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::gradient;           solver.go(sys);
#endif

  cout <<"\n== second test: T step planning ==\n" <<endl;
  T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.setTimeInterval(3.,T);
  sys.setToq0();
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,10*1e4);
  solver.q.clear();  solver.method=soc::SocSolver::AICO;               solver.go(sys);
  //solver.q.clear();  solver.method=soc::SocSolver::AICO_ms;            solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::LQG_straightInit;   solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::gradient;           solver.go(sys);
}

//===========================================================================

void problem2(){
  cout <<"\n= problem 2: dynamic reaching =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  //setup the task
  TaskVariable *pos = new TaskVariable("position" , *sys.ors, posTVT, "palmCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,
                                           MT::getParameter<double>("reachPlanMidPrec"),
                                           MT::getParameter<double>("reachPlanEndPrec"),
                                           0.,
                                           MT::getParameter<double>("reachPlanEndVelPrec"));

  soc::SocSolver solver;
  solver.init();

  solver.q.clear();  solver.method=soc::SocSolver::AICO;               solver.go(sys);
  //solver.q.clear();  solver.method=soc::SocSolver::AICO_ms;            solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::LQG_straightInit;  solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::gradient;           solver.go(sys);
}


//===========================================================================

void problem3(){
  cout <<"\n= problem 3: dynamic grasping =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);
  setGraspGoals(sys,T,"cyl1");

  soc::SocSolver solver;
  solver.init();

  solver.q.clear();  solver.method=soc::SocSolver::AICO;               solver.go(sys);
  //solver.q.clear();  solver.method=soc::SocSolver::LQG_ms;             solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::LQG_straightInit;   solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::AICO_ms;            solver.go(sys);
  solver.q.clear();  solver.method=soc::SocSolver::gradient;           solver.go(sys);
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  case 3:  problem3();  break;
  default: NIY;
  }
  return 0;
}
