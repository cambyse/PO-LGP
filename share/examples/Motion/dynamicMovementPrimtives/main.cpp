#include <Core/util.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Motion/dynamicMovementPrimitives.h>
#include <vector>
#include <GL/glu.h>
#include <stdlib.h>

const double PI = 3.1415926535897932384626;

//** test trajectory in 2d space without robot **//
void scenario1() {
  //  Create a test tracetory
  arr traj;
  uint i;
  double n = 100;
  for (i=0;i<n;i++) {
    traj.append(~ARRAY(sin(i/n*PI/2*3),2+cos(PI*i/n)));
  }

  // Create DMP
  DynamicMovementPrimitives d(traj, 100, 0.01);
  d.trainDMP();

  // Simulate DMP
  while(d.X>1e-3) {
    d.iterate();
  }

  // print DMP
  d.printDMP();
  d.plotDMP();

  // Reset DMP
  d.reset();

  // Simulate DMP
  d.changeGoal(ARRAY(2.,2.));

  // Simulate DMP
  for (i=0;i<300;i++) {
    d.iterate();
  }

  // print DMP
  d.printDMP();
  d.plotDMP();

}

//** optimized trajectory in joint space **//
void scenario2() {
  ors::KinematicWorld world("scenes/scene1.ors");
  world.gl().resize(800, 800);

  makeConvexHulls(world.shapes);
  cout << "Loaded scene: " << endl;

  MotionProblem P(world);
  P.loadTransitionParameters();

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTask("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));

  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);
  c = P.addTask("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                             ARRAY(0.,0.,0.), 1e3,
                             ARRAY(0.,0.,0.), 0.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();

  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n<<" # joints=" <<world.getJointStateDimension()<<endl;

  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));

  P.costReport();
  //  displayTrajectory(x, 1, G, gl,"planned trajectory");


  // create DMP with optimized trajectory in joint space
  arr traj;

  // Create DMP
  DynamicMovementPrimitives d(x, 100, 0.01);
  d.trainDMP();

  // Simulate DMP
  while(d.X>1e-3) {
    d.iterate();
  }

  // print DMP
  d.printDMP();
  d.plotDMP();

  // reset DMP
  d.reset();

  // Set start state
  arr q0,dq0,q;
  q0 = x[0]; dq0 = 0.*q0;
  world.setJointState(q0,dq0);
  world.getJointState(q);

  // apply DMP on robot
  while(d.X>1e-3) {

    world.setJointState(d.Y);

    d.iterate();
    world.gl().update();
  }

}

//** optimized trajectory in cartesian space **//
void scenario3() {
  ors::KinematicWorld G("scenes/scene1.ors");
  G.gl().resize(800, 800);

  makeConvexHulls(G.shapes);
  cout << "Loaded scene: " << endl;

  MotionProblem P(G);
  P.loadTransitionParameters();

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTask("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);
  c = P.addTask("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                             ARRAY(0.,0.,0.), 1e3,
                             ARRAY(0.,0.,0.), 0.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt=P.tau;
  cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<"dt="<<dt<<" # joints=" <<G.getJointStateDimension()<<endl;

  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));

  P.costReport();
  //  displayTrajectory(x, 1, G, gl,"planned trajectory");


  //------------------------------------------------//
  // Transform trajectory into task space
  //------------------------------------------------//
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    G.setJointState(x[t]);
    G.kinematicsPos(kinPos, NoArr, P.world.getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec, NoArr, P.world.getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  // create DMP with optimized trajectory in cartesian space

  // Create DMP
  DynamicMovementPrimitives d(xRefPos, 100, dt);
  d.trainDMP();

  // Simulate DMP
  while(d.X>1e-3) {
    d.iterate();
  }

  // print DMP
  d.printDMP();
  d.plotDMP();

  // reset DMP
  d.reset();

  // Set start state
  arr q0,dq0,q,qd;
  q0 = x[0]; dq0 = 0.*q0;
  G.setJointState(q0,dq0);
  G.getJointState(q);

  double fPos_deviation = 1e-2;
  double w_reg = 100.;

  // apply DMP on robot
  while(d.X>1e-3) {
    arr W, yPos, JPos, yPos_target, y_target, Phi, PhiJ ,costs;

    W.setDiag(1.,G.getJointStateDimension());  // W is equal the Id_n matrix
    W = W*w_reg;

    // Compute current task states
    G.kinematicsPos(yPos, JPos, G.getBodyByName("endeff")->index);

    // iterate dmp
    d.iterate();
    arr y = yPos;

    // next target
    y_target = d.Y;

    // task 1: POSITION
    yPos_target = y_target.subRange(0,2);
    costs = (yPos - yPos_target)/ fPos_deviation;
    Phi = ((yPos - yPos_target)/ fPos_deviation);
    PhiJ = (JPos / fPos_deviation);

    // compute joint updates
    qd = inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    q -= qd;

    G.setJointState(q);

    G.gl().update();
  }
  G.gl().watch();
}

//** optimized trajectory in cartesian space with orientation **//
void scenario4() {
  bool useOrientation = true;

  ors::KinematicWorld G("scenes/scene1.ors");
  G.gl().resize(800, 800);

  makeConvexHulls(G.shapes);
  cout << "Loaded scene: " << endl;

  MotionProblem P(G);
  P.loadTransitionParameters();

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTask("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);
  c = P.addTask("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                             ARRAY(0.,0.,0.), 1e3,
                             ARRAY(0.,0.,0.), 0.);

  if (useOrientation) {
    c = P.addTask("orientation", new DefaultTaskMap(vecTMT,G,"endeff",ors::Vector(0., 1., 0.)));
    P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                            ARRAY(1.,0.,0.), 1e3,
                            ARRAY(0.,0.,0.), 1e-3);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt=P.tau;
  cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<"dt="<<dt<<" # joints=" <<G.getJointStateDimension()<<endl;

  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));

  P.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");


  //------------------------------------------------//
  // Transform trajectory into task space
  //------------------------------------------------//
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    G.setJointState(x[t]);
    G.kinematicsPos(kinPos, NoArr, P.world.getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec, NoArr, P.world.getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  // create DMP with optimized trajectory in cartesian space

  // Create DMP
  DynamicMovementPrimitives d(xRefPos, 100, dt);
  d.trainDMP();

  DynamicMovementPrimitives d_vec(xRefVec, 100, dt);
  d_vec.trainDMP();

  // Simulate DMP
  while(d.X>1e-3) {
    d.iterate();
  }

  // print DMP
  d.printDMP();
  d.plotDMP();

  // reset DMP
  d.reset();

  // Set start state
  arr q0,dq0,q,qd;
  q0 = x[0]; dq0 = 0.*q0;
  G.setJointState(q0,dq0);
  G.getJointState(q);

  double fPos_deviation = 1e-2;
  double fVec_deviation = 1e-3;
  double w_reg = 100.;

  // apply DMP on robot
  while(d.X>1e-3) {
    arr W, yPos, JPos, yPos_target, y_target, Phi, PhiJ ,costs, yVec, JVec, yVec_target;

    W.setDiag(1.,G.getJointStateDimension());  // W is equal the Id_n matrix
    W = W*w_reg;

    // Compute current task states
    G.kinematicsPos(yPos, JPos, G.getBodyByName("endeff")->index);
    G.kinematicsVec(yVec, JVec, G.getBodyByName("endeff")->index);

    // iterate dmp
    d.iterate();
    d_vec.iterate();
    arr y = yPos;
    if (useOrientation) {
      y.append(yVec);
    }

    // next target
    y_target = d.Y;
    y_target.append(d_vec.Y);

    // task 1: POSITION
    yPos_target = y_target.subRange(0,2);
    costs = (yPos - yPos_target)/ fPos_deviation;
    Phi = ((yPos - yPos_target)/ fPos_deviation);
    PhiJ = (JPos / fPos_deviation);

    // task  2: ORIENTATION
    if (useOrientation) {
      yVec_target = y_target.subRange(3,5);
      costs = (yVec - yVec_target)/ fVec_deviation;
      Phi.append(costs);
      PhiJ.append(JVec / fVec_deviation);
    }

    // compute joint updates
    qd = inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    q -= qd;

    G.setJointState(q);

    G.gl().update();
  }
  G.gl().watch();
}

int main(int argc,char **argv){
  switch(MT::getParameter<int>("mode",4)){
  case 1:  scenario1();  break;
  case 2:  scenario2();  break;
  case 3:  scenario3();  break;
  case 4:  scenario4();  break;
  }
  return 0;
}
