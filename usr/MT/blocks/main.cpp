#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>
#include <Ors/ors_swift.h>

void createWorld(ors::KinematicWorld &G){
  G.init("blocks.kvg");

  int K=1;
  for(int x=-K-1;x<=K+1;x++) for(int y=-K;y<=K;y++){
    ors::Body *b = new ors::Body(G);
    b->X.pos.set(.3*x, .3*y, 1.15);
    ors::Shape *s = new ors::Shape(G, *b);
    s->type=ors::sphereST;
    s->size[0]=s->size[1]=.1; s->size[2]=.2; s->size[3]=.08;
    s->color[0]=.9; s->color[1]=s->color[2]=.2;
    s->parseAts();
    s->cont = true;

    ors::Joint *j = new ors::Joint(G, G.bodies(0), b);
    j->X = b->X;
    j->type = ors::JT_trans3;
  }
  G.calc_missingAB_from_BodyAndJointFrames();
  G.getJointStateDimension();
  G.calc_q_from_Q();
  G.calc_fwdPropagateFrames();
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld G;
  createWorld(G);

  arr q0,qT;
  q0=qT=G.q;
  qT(qT.N/2-1) += -2.;
  G.setJointState(qT);
//  G.gl().watch();
  rndGauss(q0, .02, true);
  G.setJointState(q0);
//  G.gl().watch();

  MotionProblem MP(G);
  MP.loadTransitionParameters();
  MP.postfix.resize(2,MP.x0.N);
  MP.postfix[0]=MP.postfix[1]=qT;
  MP.world.swift().initActivations(0);
  MP.world.swift().step();
  MP.world.reportProxies();
//  return 0;

  TaskCost *c;
  bool con=true;
  if(con){
    c = MP.addTask("collisionConstraints", new CollisionConstraint(.02));
  }else{
    c = MP.addTask("collision",
                   new DefaultTaskMap(collTMT, G, NULL, NoVector, NULL, NoVector, ARR(.1)));
    MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  Convert CP(MF);
  UnconstrainedProblem UCP(CP);
  UCP.mu = 10.;

  //gradient check
  arr x(MP.T+1 - MP.postfix.d0, qT.N);
//  for(uint t=0;t<x.d0;t++) x[t]() = MP.x0;
  for(uint k=0;k<0;k++){
//    rndUniform(x,-1.,1.);
    checkJacobian(Convert(MF), x, 1e-5);
    /* Why the gradient check fails:
     * When final velocity is conditioned: the Jacobian w.r.t. the final time slice depends on the final configuration -- in motion.cpp:231
     * the Hessian is not used to estimate the velocity gradient -- that's an approximation! For small velocities (optimized traj) it should still be ok.
     * When collisions are conditions: the Jacobian is in principle approximate. */
  }

  for(uint t=0;t<x.d0;t++) x[t]() = MP.x0;

  //-- optimize
  if(con){
    for(uint k=0;k<20;k++){
//      checkAllGradients(CP, x, 1e-4);
      optNewton(x, UCP, OPT(verbose=2, stopIters=100, damping=1e-3, nonStrict=(!k?15:5), maxStep=1.));
      MP.costReport();
      UCP.augmentedLagrangian_LambdaUpdate(x, 1.);
      cout <<"f(x)=" <<UCP.f_x <<" \tmu=" <<UCP.mu <<" \tmuLB=" <<UCP.muLB <<endl;
//      if(x.N<5)
        cout <<"lambda=" <<UCP.lambda <<endl;
      displayTrajectory(x, 1, G, "planned trajectory");
      UCP.mu *= 2;
    }
  }else{
    for(uint k=0;k<5;k++){
      MT::timerStart();
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2., nonStrict=(!k?15:5), damping=.1));

      cout <<"** optimization time=" <<MT::timerRead() <<endl;
      //costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
      MP.costReport();
      checkJacobian(Convert(MF), x, 1e-5);
      write(LIST<arr>(x),"z.output");
      //gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
      gnuplot("load 'z.costReport.plt'", false, true);
      displayTrajectory(x, 1, G, "planned trajectory", 0.01);
    }
  }

  return 0;
}


