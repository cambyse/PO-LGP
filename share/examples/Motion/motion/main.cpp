#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  G.getShapeByName("target")->cont=false;
  cout <<"loaded model: n=" <<G.q.N <<endl;
//  G.gl().watch();

  MotionProblem MP(G);
  MP.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target")->X.pos), 1e3);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .1));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);

//  c = MP.addTask("collisionConstraints", new CollisionConstraint(.1));

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

  arr x(T+1,n);

  //gradient check
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(MF), x, 1e-5);
    /* Why the gradient check fails:
     * When final velocity is conditioned: the Jacobian w.r.t. the final time slice depends on the final configuration -- in motion.cpp:231
     * the Hessian is not used to estimate the velocity gradient -- that's an approximation! For small velocities (optimized traj) it should still be ok.
     * When collisions are conditions: the Jacobian is in principle approximate. */
  }

  //initialize trajectory
//  if(MP.x0.N==3){ //assume 3D ball!
//    for(uint t=0;t<=T;t++){
//      double a=(double)t/T;
//      x[t]() = (1.-a)*MP.x0 + a*ARRAY(MP.world.getBodyByName("target")->X.pos);
//    }
//  }else{
    for(uint t=0;t<=T;t++) x[t]() = MP.x0;
//  }

  //evaluation test
  //  cout <<"fx = " <<evaluateVF(Convert(MF), x) <<endl;

  //  OpenGL costs(STRING("PHI ("<<F.dim_phi(0)<<" tasks)"), 3*T+10, 3*F.dim_phi(0)+10 );
  //-- optimize
  for(uint k=0;k<5;k++){
    MT::timerStart();
#if 1
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2., nonStrict=(!k?15:5)));
#else
    ConstrainedMethodType method = (ConstrainedMethodType)MT::getParameter<int>("method");
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, damping=1., maxStep=1., nonStrict=5, constrainedMethod=method));
#endif

    cout <<"** optimization time=" <<MT::timerRead() <<endl;
    //costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
    MP.costReport();
//    checkJacobian(Convert(MF), x, 1e-5);
    write(LIST<arr>(x),"z.output");
    //gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
  
  return 0;
}


