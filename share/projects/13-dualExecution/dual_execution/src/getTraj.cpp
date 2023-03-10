#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Core/thread.h>

#include "getTraj.h"

void getTrajectory(arr& x, arr& y, arr& ori, arr& dual, mlr::KinematicWorld& world){
  KOMO P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();

  //-- setup the motion problem
  Task *pos =
      P.addTask("position",
                   new TaskMap_Default(posTMT, world, "endeff", NoVector));
  pos->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(pos, KOMO::finalOnly, {0.,0.,0.}, 1e1);

  //c = P.addTask("collisionConstraints", new CollisionConstraint());
  Task *cont = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);
  LagrangianProblem LagrangianP(ConstrainedP);
  LagrangianP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, LagrangianP, OPT(verbose=2, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
//    optNewton(x, UCP, OPT(verbose=2, stopIters=100, damping=1e-3, maxStep=1.));
    P.costReport();
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    LagrangianP.augmentedLagrangian_LambdaUpdate(x, .9);
    P.dualMatrix = LagrangianP.lambda;
    LagrangianP.mu *= 2.;
  }
  P.costReport();

  if(&y){
    //-- get a map to map into SL endeff space
    TaskMap *m = new TaskMap_Default(posTMT, world, "SL_endeff", NoVector);

    y.resize(x.d0, pos->map.dim_phi(world));
    ori.resize(x.d0, 4);
    mlr::Shape *s_SL_endeff = world.getShapeByName("SL_endeff");
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      m->phi(y[t](), NoArr, world);
      ori[t]() = conv_quat2arr(s_SL_endeff->X.rot);
    }
    delete m;
  }
  if(&dual) dual = LagrangianP.lambda;
}
