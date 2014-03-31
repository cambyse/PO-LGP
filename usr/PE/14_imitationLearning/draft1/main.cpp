#include <Ors/ors.h>
#include <Optim/search.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/feedbackControl.h>
#include <vector>

void createToyDemonstrations(std::vector<arr> &demos,arr &q0) {
  uint trajIter;
  for (trajIter=0;trajIter<10;trajIter++) {

    ors::KinematicWorld world("scene");
    arr q, qdot;
    world.getJointState(q, qdot);

    makeConvexHulls(world.shapes);
    MotionProblem MP(world);
    MP.loadTransitionParameters();
    arr refGoal = ARRAY(MP.world.getBodyByName("goalRef")->X.pos);
    refGoal(2) = refGoal(2) + trajIter*0.05;

    TaskCost *c;
    c = MP.addTask("position_right_hand", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
    MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 1e5);
    c = MP.addTask("final_vel", new DefaultTaskMap(qItselfTMT,world));
    MP.setInterpolatingCosts(c,MotionProblem::finalOnly,ARRAY(0.),1e3);
    c->map.order=1;
    MP.x0 = {0.,0.,0.,0.,0.};

    MotionProblemFunction F(MP);
    uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

    arr x(T+1,n); x.setZero();
    optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    //    MP.costReport(true);
//    displayTrajectory(x, 1, world, "planned trajectory", 0.01);

    arr kinPos, xRefPos;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      world.setJointState(x[t]);
      world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff")->index);
      xRefPos.append(~kinPos);
    }
    // Save trajectory
    demos.push_back(xRefPos);
//    write(LIST<arr>(xRefPos),STRING("data/"<<"xRefPos"<<trajIter));
    q0 = x;
  }

}

double trajectoryCosts(arr &a, arr &b) {
  return sum((a-b)%(a-b));
}

void execRun(arr param, arr &y, arr q0, arr refGoal, bool vis=false) {
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);

//  makeConvexHulls(world.shapes);
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  world.getBodyByName("goalRef")->X.pos = refGoal;
  TaskCost *c;
  c = MP.addTask("position_right_hand", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, param(0));
  c = MP.addTask("vel_right_hand", new DefaultTaskMap(vecTMT,world,"endeff", ors::Vector(0., 1., 0.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARR(0.,1.,0.), param(1));
  c = MP.addTask("final_vel", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c,MotionProblem::finalOnly,ARRAY(0.),param(2));
  c->map.order=1;
  MP.x0 = {0.,0.,0.,0.,0.};

  MotionProblemFunction F(MP);
  uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;
//  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();

  x = q0;
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
//  MP.costReport(true);
  if (vis)
      displayTrajectory(x, 1, world, "planned trajectory", 0.01);

  arr kinPos;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff")->index);
    y.append(~kinPos);
  }
}



int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  std::vector<arr> demos;
  arr q0;
  createToyDemonstrations(demos,q0);

  SearchCMA cma;
  cma.init(3);

  arr samples, values;
  arr y,yRef;
  double costs;
  for(uint t=0;t<100;t++){
    /// run cma
    cma.step(samples, values);
    cout << exp(samples) << endl;
    for(uint i=0;i<samples.d0;i++) {
      /// simulate parameters for each scenario
      costs = 0.;
      for(uint j=0;j<demos.size();j++) {
        y.clear();
        yRef = demos.at(j);
        execRun(exp(samples[i]),y,q0,yRef[yRef.d0-1]);
        costs = costs + trajectoryCosts(yRef,y);
      }

      values(i) = costs/demos.size();
    }
    cout << values.min() << endl;
  }
  arr optParam = samples[values.minIndex()];
  execRun(exp(optParam),y,q0,yRef[yRef.d0-1],true);

  return 0;
}
