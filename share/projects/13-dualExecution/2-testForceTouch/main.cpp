#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/constrained.h>


void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world){
  MotionProblem P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();

  //-- setup the motion problem
  TaskCost *pos =
      P.addTask("position",
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,
                          ARRAY(P.world.getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(pos, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  //c = P.addTask("collisionConstraints", new CollisionConstraint());
  TaskCost *cont = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));

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
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  if(&dual) dual = UnConstrainedP.lambda;
}

void testExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world){
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MC(world);
  MC.nullSpacePD.active=false;

  PDtask *pd_y=
      MC.addPDTask("position", .1, .8,
                    new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  pd_y->prec = 10.;

  PDtask *pd_x=
      MC.addPDTask("pose", .1, .8,
                    new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  ConstraintForceTask *pd_c =
//      MC.addConstraintForceTask("planeConstraint",
//                                new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));
      MC.addConstraintForceTask("touchTable",
                                new PairCollisionConstraint(world, "endeff2", "table"));

  double tau=0.01;
  for(uint t=0;t<x.d0;t++){
    MC.setState(q, qdot);

    pd_y->y_ref = y[t];
    pd_x->y_ref = x[t];
    pd_c->desiredForce=dual(t);

    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    world.watch(true, STRING(t));
  }

}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));

  arr x, y, dual;
  getTrajectory(x, y, dual, world);

//  arr x2 = reverseTrajectory(x);
//  x.append(x2);
//  for(uint i=0;i<2;i++)
//    displayTrajectory(x, 1, world, "planned trajectory");

  world.getBodyByName("table")->X.pos.z += .1;
  world.setJointState(x[0]);

  testExecution(x, y, dual, world);

  return 0;
}


