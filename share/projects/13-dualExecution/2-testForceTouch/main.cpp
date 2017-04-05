#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Control/taskControl.h>
#include <Optim/optimization.h>


void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world){
  MotionProblem P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();

  //-- setup the motion problem
  Task *pos =
      P.addTask("position",
                   new TaskMap_Default(posTMT, world, "endeff", NoVector));
  pos->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(pos, MotionProblem::finalOnly, {0.,0.,0.}, 1e1);

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
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  if(&dual) dual = LagrangianP.lambda;
}

void testExecution(const arr& x, const arr& y, const arr& dual, mlr::KinematicWorld& world){
  arr q, qdot;
  world.getJointState(q, qdot);

  TaskControlMethods MC(world);
  MC.qitselfPD.active=false;

  CtrlTask *pd_y=
      MC.addPDTask("position", .1, .8,
                    new TaskMap_Default(posTMT, world, "endeff", NoVector));
  pd_y->prec = 10.;

  CtrlTask *pd_x=
      MC.addPDTask("pose", .1, .8,
                    new TaskMap_qItself());
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
  mlr::initCmdLine(argc,argv);

  mlr::KinematicWorld world(mlr::getParameter<mlr::String>("orsFile"));

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


