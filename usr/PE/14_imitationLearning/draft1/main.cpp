#include <Kin/kin.h>
#include <Optim/blackbox.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Control/taskControl.h>
#include <vector>
#include <future>


void createToyDemonstrations(std::vector<arr> &demos,arr &q0) {
  uint trajIter;
  for (trajIter=0;trajIter<10;trajIter++) {

    mlr::KinematicWorld world("scene");
    arr q, qdot;
    world.getJointState(q, qdot);

    makeConvexHulls(world.shapes);
    KOMO MP(world);
    MP.loadTransitionParameters();
    arr refGoal = ARR(MP.world.getBodyByName("goalRef")->X.pos);
    refGoal(2) = refGoal(2) + trajIter*0.05;

    Task *c;
    c = MP.addTask("position_right_hand", new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.)));
    c->setCostSpecs(MP.T, MP.T, refGoal, 1e5);
    c = MP.addTask("final_vel", new TaskMap_qItself());
    MP.setInterpolatingCosts(c,KOMO::finalOnly,{0.},1e3);
    c->map.order=1;
    MP.x0 = {0.,0.,0.,0.,0.};

    MotionProblemFunction F(MP);
    uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

    arr x(T+1,n); x.setZero();
    optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
    //    MP.costReport(true);
//    displayTrajectory(x, 1, world, "planned trajectory", 0.01);

    arr kinPos, xRefPos;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      world.setJointState(x[t]);
      world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff"));
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

arr execRun(arr param, arr q0, arr refGoal) {
  bool vis=false;
  mlr::KinematicWorld world("scene");
  arr q, qdot;
  arr y;
  world.getJointState(q, qdot);

//  makeConvexHulls(world.shapes);
  KOMO MP(world,false);
  MP.loadTransitionParameters();

  world.getBodyByName("goalRef")->X.pos = refGoal;
  Task *c;
  c = MP.addTask("position_right_hand", new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.)));
  c->setCostSpecs(MP.T, MP.T, refGoal, param(0));
  c = MP.addTask("vel_right_hand", new TaskMap_Default(vecTMT,world,"endeff", mlr::Vector(0., 1., 0.)));
  c->setCostSpecs(MP.T, MP.T, ARR(0.,1.,0.), param(1));
  c = MP.addTask("final_vel", new TaskMap_qItself());
  MP.setInterpolatingCosts(c,KOMO::finalOnly,{0.},param(2));
  c->map.order=1;
  MP.x0 = {0.,0.,0.,0.,0.};

  MotionProblemFunction F(MP);
  uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;
//  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();

  x = q0;
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
//  MP.costReport(true);
  if (vis)
      displayTrajectory(x, 1, world, "planned trajectory", 0.01);

  arr kinPos;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff"));
    y.append(~kinPos);
  }
  return y;
}


int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  std::vector<arr> demos;
  arr q0;
  createToyDemonstrations(demos,q0);

  SearchCMA cma;
  cma.init(3);

  arr samples, values;
  arr yRef1,yRef2,yRef3,y1,y2,y3;
  double costs;
  for(uint t=0;t<100;t++){
    /// run cma
    cma.step(samples, values);
//    cout << exp(samples) << endl;
    mlr::timerStart(true);
    for(uint i=0;i<samples.d0;i++) {
      /// simulate parameters for each scenario
      costs = 0.;
      std::vector<std::future<arr>> runs;
      double t_d = mlr::timerRead();
      uint j = 0;
      while (j < demos.size()) {
        yRef1 = demos.at(j);
        runs.push_back(std::async(std::launch::async,execRun,exp(samples[i]),q0,yRef1[yRef1.d0-1]));
        j++;
      }
      j = 0;
      while (j < demos.size()) {
        y1 = runs[j].get();
        yRef1 = demos.at(j);
        costs = costs + trajectoryCosts(yRef1,y1);
        j++;
      }
      values(i) = costs/demos.size();
    }
    cout <<"Time: "<< mlr::timerPause() << endl;
    cout <<"Min Value: " << values.min() << endl;
  }
  arr optParam = samples[values.minIndex()];
//  execRun(exp(optParam),y,q0,yRef[yRef.d0-1],true);

  return 0;
}
