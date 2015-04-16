#include "TrajectoryFactory.h"
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>


void createToyDemonstrations1(MT::Array<Demonstration> &demos) {
  uint trajIter;
  for (trajIter=0;trajIter<5;trajIter++) {

    ors::KinematicWorld world("scene");
    arr q, qdot;
    world.getJointState(q, qdot);

    makeConvexHulls(world.shapes);
    MotionProblem MP(world);
    MP.loadTransitionParameters();
    arr refGoal = ARRAY(MP.world.getBodyByName("goalRef")->X.pos);
    refGoal(2) = refGoal(2) + trajIter*0.05;

    Task *c;
    c = MP.addTask("position_right_hand", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
    MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 1e5);
    c = MP.addTask("final_vel", new TaskMap_qItself());
    MP.setInterpolatingCosts(c,MotionProblem::finalOnly,{0.},1e3);
    c->map.order=1;
    MP.x0 = {0.,0.,0.,0.,0.};

    MotionProblemFunction F(MP);
    uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

    arr x(T+1,n); x.setZero();
    optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
//    displayTrajectory(x, 1, world, "planned trajectory", 0.01);

    arr kinPos, xRefPos;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      world.setJointState(x[t]);
      world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff"));
      xRefPos.append(~kinPos);
    }
    Demonstration dem;
    dem.world = world;
    dem.q0 = x[0];
    dem.qTraj = x;

    // Save trajectory
    demos.append(dem);
  }

}
