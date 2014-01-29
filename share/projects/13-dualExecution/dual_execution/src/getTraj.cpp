#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>
#include <Core/thread.h>

#include "getTraj.h"

void getTrajectory(arr& x, arr& y, arr& ori, arr& dual, ors::KinematicWorld& world){
  MotionProblem P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();

  //-- setup the motion problem
  TaskCost *pos =
      P.addTaskMap("position",
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,
                          ARRAY(P.world.getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(pos, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  //c = P.addTaskMap("collisionConstraints", new CollisionConstraint());
  TaskCost *cont = P.addTaskMap("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);
  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
//    optNewton(x, UCP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    P.costReport();
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.augmentedLagrangian_LambdaUpdate(x, .9);
    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;
  }
  P.costReport();

  if(&y){
    //-- get a map to map into SL endeff space
    TaskMap *m = new DefaultTaskMap(posTMT, world, "SL_endeff", NoVector);

    y.resize(x.d0, pos->map.dim_phi(world));
    ori.resize(x.d0, 4);
    ors::Shape *s_SL_endeff = world.getShapeByName("SL_endeff");
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      world.calcBodyFramesFromJoints();
      m->phi(y[t](), NoArr, world);
      ori[t]() = ARRAY(s_SL_endeff->X.rot);
    }
    delete m;
  }
  if(&dual) dual = UnConstrainedP.lambda;
}
