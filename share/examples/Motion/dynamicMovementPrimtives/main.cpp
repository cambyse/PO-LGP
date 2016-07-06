#include <Core/util.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
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
    traj.append(~ARR(sin(i/n*PI/2*3),2+cos(PI*i/n)));
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
}

//** optimized trajectory in joint space **//
void scenario2() {
  ors::KinematicWorld world("scenes/scene1.ors");
  world.gl().resize(800, 800);

  makeConvexHulls(world.shapes);
  cout << "Loaded scene: " << endl;

  MotionProblem P(world);

  //-- create an optimal trajectory to trainTarget
  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(world));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, ARR(0.),1e-2);


  c = P.addTask("position", new TaskMap_Default(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));

  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          {0.,0.,0.}, 1e-3);
  c = P.addTask("position", new TaskMap_Default(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T,
                             {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0.);

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

  //-- create an optimal trajectory to trainTarget
  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, ARR(0.),1e-2);

  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          {0.,0.,0.}, 1e-3);
  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T,
                             {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0.);

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
    G.kinematicsPos(kinPos, NoArr, P.world.getBodyByName("endeff"));
    G.kinematicsVec(kinVec, NoArr, P.world.getBodyByName("endeff"));
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

    W.setDiag(1.,G.getJointStateDimension());
    W = W*w_reg;

    // Compute current task states
    G.kinematicsPos(yPos, JPos, G.getBodyByName("endeff"));

    // iterate dmp
    d.iterate();
    arr y = yPos;

    // next target
    y_target = d.Y;

    // task 1: POSITION
    yPos_target = y_target.refRange(0,2);
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

  //-- create an optimal trajectory to trainTarget
  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, ARR(0.),1e-2);

  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          {0.,0.,0.}, 1e-3);
  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T,
                             {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0.);

  if (useOrientation) {
    c = P.addTask("orientation", new TaskMap_Default(vecTMT,G,"endeff",ors::Vector(0., 1., 0.)));
    c->setCostSpecs(P.T, P.T,
                            {1.,0.,0.}, 1e3,
                            {0.,0.,0.}, 1e-3);
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
    G.kinematicsPos(kinPos, NoArr, P.world.getBodyByName("endeff"));
    G.kinematicsVec(kinVec, NoArr, P.world.getBodyByName("endeff"));
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
    G.kinematicsPos(yPos, JPos, G.getBodyByName("endeff"));
    G.kinematicsVec(yVec, JVec, G.getBodyByName("endeff"));

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
    yPos_target = y_target.refRange(0,2);
    costs = (yPos - yPos_target)/ fPos_deviation;
    Phi = ((yPos - yPos_target)/ fPos_deviation);
    PhiJ = (JPos / fPos_deviation);

    // task  2: ORIENTATION
    if (useOrientation) {
      yVec_target = y_target.refRange(3,5);
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







void scenario5(){
  // PoWeR Example with a single DMP (based on MATLAB implementation PoWER_Q_toy.m of J. Kober)
  double dt = 0.005;
  uint D = 2;
  uint n_base = 10;
  uint n_param = D*n_base;
  uint n_iter = 2000;
  uint N = 201;

  arr Q; Q.resize(n_iter+1,N).setZero();
  arr Return; Return.resize(1,n_iter+1).setZero(); Return.flatten();
  arr s_Return; s_Return.resize(n_iter+1,2).setZero();
  uintA s_ReturnI; s_ReturnI.resize(n_iter+1).setZero();
  arr param; param.resize(n_iter+1,n_param).setZero();
  arr basis; basis.resize(n_base,N).setZero();

  arr variance = 4000.*ones(n_param,1); variance.flatten();

  arr xRef;
  xRef << FILE("data/y.dat");
  xRef = catCol(xRef,xRef);

  cout << xRef << endl;

  DynamicMovementPrimitives dmp(xRef, n_base, dt);
  dmp.trainDMP();

  basis = dmp.PHI%(1./repmat(sum(dmp.PHI,1),1,n_base));
  basis = repmat(basis,1,D);

  arr current_param = param.row(0); current_param.flatten();
  dmp.weights = current_param;
  dmp.weights.reshape(D,n_base);
  dmp.weights = ~dmp.weights;


  arr q(N); q.flatten();
  for (uint t=0;t<N;t++) {
    dmp.iterate();
    q.subRange(0,t) = q.subRange(0,t)+exp(-pow(1.-dmp.Y(0),2.)) + exp(-pow(1.-dmp.Y(1),2.));
  }

  q = q/(double)N;
  Q[0] = q;

  for (uint iter=0;iter<n_iter;iter++) {
    dmp.reset();

    Return(iter) = Q(iter,0);
    s_Return[0] = ARR(Return(iter),iter);
    for (uint l=0;l<=n_iter;l++){s_ReturnI(l)=l;}

    std::sort(s_ReturnI.begin(), s_ReturnI.end(),[&](const int& a, const int& b) {
               return (s_Return(a,0) < s_Return(b,0)); });
    s_Return.permuteRows(s_ReturnI);

    arr param_nom(n_param); param_nom.setZero();
    arr param_dnom(n_param); param_nom.setZero();

    for (uint i=0; i<min(ARR(iter+1,20)); i++) {
      uint j = s_Return(s_Return.d0-1-i,1);
      arr temp_W = ~(basis%basis%(1./repmat(sum(basis%basis%repmat(~variance,N,1),1),1,n_param)));
      arr temp_explore = repmat(param[j]-current_param,1,N);
      arr temp_Q = repmat(~Q[j],n_param,1);

      param_nom = param_nom + sum(temp_W%temp_explore%temp_Q,1);
      param_dnom = param_dnom + sum(temp_W%temp_Q,1);
    }

    param[iter+1] = current_param + param_nom%(1./(param_dnom+1e-10));
    current_param = param[iter+1];

    if (iter!=n_iter-2) {param[iter+1] = param[iter+1] + pow(variance,.5)%randn(n_param,1);}
    dmp.weights = param[iter+1];
    dmp.weights.reshape(D,n_base);
    dmp.weights = ~dmp.weights;

    q.setZero();
    for (uint t=0;t<N;t++) {
      dmp.iterate();
      q.subRange(0,t) = q.subRange(0,t)+exp(-pow(1.-dmp.Y(0),2.)) + exp(-pow(1.-dmp.Y(1),2.));
    }
    q = q/(double)N;
    Q[iter+1] = q;
  }

  Return(n_iter) = Q(n_iter,0);

  Return >> FILE("data/Return.dat");
  dmp.plotDMP();
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
  switch(mlr::getParameter<int>("mode",4)){
  case 1:  scenario1();  break;
  case 2:  scenario2();  break;
  case 3:  scenario3();  break;
  case 4:  scenario4();  break;
  case 5:  scenario5();  break;
  }
  return 0;
}
