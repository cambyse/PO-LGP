#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));

  MotionProblem MP(G);
  MP.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = MP.addTaskMap("position",
                    new DefaultTaskMap(posTMT, G, "endeff", ors::Vector(0, 0, .2)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly,
                           ARRAY(MP.world.getBodyByName("target")->X.pos), 1e2);
  MP.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                              ARRAY(0.,0.,0.), 1e1);

  //  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
  //  P.setInterpolatingCosts(c, MotionProblem::1constFinalMid, ARRAY(0.), 1e-0);

  //  c = P.addDefaultTaskMap("qitself", qItselfTMT, (int)0, Transformation_Id, 0, Transformation_Id, 0);
  //  P.setInterpolatingCosts(   c, MotionProblem::constFinalMid, ARRAY(0.), 1e-4);
  //  //P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e4, ARRAY(0.), 1e-2);

  //-- collisions with other objects
  uintA shapes = ARRAY<uint>(MP.world.getBodyByName("endeff")->shapes(0)->index);
  c = MP.addTaskMap("proxyColls",
                    new ProxyTaskMap(allVersusListedPTMT, shapes, .2, true));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e2);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  uint T=MF.get_T();
  uint k=MF.get_k();
  uint n=MF.dim_x();
  cout <<"Problem parameters:"
      <<"\n T=" <<T
     <<"\n k=" <<k
    <<"\n n=" <<n
   <<endl;

  //mini evaluation test:
  arr x(T+1,n);
  if(MP.x0.N==3){ //assume 3D ball!
    for(uint t=0;t<=T;t++){
      double a=(double)t/T;
      x[t]() = (1.-a)*MP.x0 + a*ARRAY(MP.world.getBodyByName("target")->X.pos);
    }
  }else{
    for(uint t=0;t<=T;t++) x[t]() = MP.x0;
  }
  cout <<"fx = " <<evaluateVF(Convert(MF), x) <<endl;

  //gradient check
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(MF), x, 1e-5);
  }
  
  //  OpenGL costs(STRING("PHI ("<<F.dim_phi(0)<<" tasks)"), 3*T+10, 3*F.dim_phi(0)+10 );
  //-- optimize
  for(uint k=0;k<10;k++){
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=40, useAdaptiveDamping=false, damping=1e-0, maxStep=1.));
    //costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
    MP.costReport();
    write(LIST<arr>(x),"z.output");
    //gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, G.gl(),"planned trajectory", 0.01);
  }
  
  return 0;
}


