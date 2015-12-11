#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  ors::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  G.getShapeByName("target")->cont=false;
  cout <<"loaded model: n=" <<G.q.N <<endl;
//  G.gl().watch();

  MotionProblem MP(G);

  //-- setup the motion problem
  Task *c;
  c = MP.addTask("transitions", new TransitionTaskMap(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

  c = MP.addTask("position", new DefaultTaskMap(posDiffTMT, G, "endeff", NoVector, "target"));
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  c = MP.addTask("quat", new DefaultTaskMap(quatDiffTMT, G, "endeff", NoVector, "target"));
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

//  c = MP.addTask("q_vel", new TaskMap_qItself());
//  c->map.order=1; //make this a velocity variable!
//  c->setCostSpecs(MP.T, MP.T, NoArr, 1e1);

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

  for(uint t=0;t<=T;t++) x[t]() = MP.x0;

  //evaluation test
  //  cout <<"fx = " <<evaluateVF(Convert(MF), x) <<endl;

  //  OpenGL costs(STRING("PHI ("<<F.dim_phi(0)<<" tasks)"), 3*T+10, 3*F.dim_phi(0)+10 );
  //-- optimize
  for(uint k=0;k<5;k++){
    mlr::timerStart();
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2., nonStrictSteps=(!k?15:5)));

    cout <<"** optimization time=" <<mlr::timerRead() <<endl;
    MP.costReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
  
  return 0;
}


