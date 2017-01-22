#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin_swift.h>

struct ValueFctLearning {
  uint nParam;     // parameter vector dimension
  uint nFeat;      // feature vector dimension

  double gamma;    // discount factor
  double lambda;   // regularization
  uint T;          // motion length

  double eps;      // eps greedy probability
  double dir_noise;// greedy noise variance
  double epsParam; // change in parameter space

  bool Wdiag;      // Set W to diagonal matrix
  double x0_noise; // noise on start position
  bool vis;
  arr param;

  int num_d;                   // number of data points for least squares
  arr VAL_d,PHI_d,X_d;          // current data for least squares
  arr VAL_hist,PHI_hist,X_hist; // all experience data
  arr PARAM_hist;

  arr PHIc_hist, PHIn_hist, REWARD_hist;

  arr A,b,z; // Iterative LSTD

  mlr::KinematicWorld *world;
  MotionProblem *MP;

  /// initialize W to diagonal matrix and all values to 1
  void initParam() {
    if (Wdiag) {
      nParam = 2*nFeat+1;
      param = ones(nFeat);
    } else{
      nParam = nFeat*(nFeat+1)+1;
      param = eye(nFeat);
      param.resize(nFeat*nFeat);
    }

//    world = new mlr::KinematicWorld("scene0");
//    arr w = ARR(world->getBodyByName("target")->X.pos);
//    param.append(w);

    param.append(ones(nFeat));
    param.append(1);

    epsParam = 1e3;
    A.resize(nParam,nParam).setZero();
    b.resize(nParam).setZero();
  }

  /// convert parameter vector of ~param*feat to ~feat*W*feat+~w*feat+b
  void paramToMatrixForm(const arr &param, arr &W, arr&w, arr &b){
    if (Wdiag){
      W = diag(param.subRange(0,nFeat-1));
      w = param.subRange(nFeat,2*nFeat-1);
      b = param.subRange(2*nFeat,2*nFeat);
    }else{
      W = param.subRange(0,nFeat*nFeat-1);
      W.resize(nFeat,nFeat);
      w = param.subRange(nFeat*nFeat,nFeat*(nFeat+1)-1);
      b = param.subRange(nFeat*(nFeat+1),nFeat*(nFeat+1));
    }
  }
  /// convert feature vector of ~feat*W*feat+~w*feat+b to ~param*FEAT
  void featToMatrixForm(const arr &feat, arr &FEAT){
    if (Wdiag){
      FEAT = feat%feat;
    }else{
      FEAT = vectorShaped(feat*~feat);
    }
    FEAT.append(feat);
    FEAT.append(ARR(1.));
  }

  void writeToFile(){
    write(LIST<arr>(VAL_d),"out/VAL_d");
    write(LIST<arr>(PHI_d),"out/PHI_d");
    write(LIST<arr>(X_d),"out/X_d");
    write(LIST<arr>(VAL_hist),"out/VAL_hist");
    write(LIST<arr>(PHI_hist),"out/PHI_hist");
    write(LIST<arr>(X_hist),"out/X_hist");
    write(LIST<arr>(PARAM_hist),"out/PARAM_hist");

    arr W,w,b;
    paramToMatrixForm(param,W,w,b);
    write(LIST<arr>(W),"out/W");
    write(LIST<arr>(w),"out/w");
    write(LIST<arr>(b),"out/b");
  }

  /// linear regression on parameters
  void updateParam() {    
    PHI_d = PHI_hist.subRange(max(ARR(0,(int)PHI_hist.d0-num_d)),PHI_hist.d0-1);
    VAL_d = VAL_hist.subRange(max(ARR(0,(int)VAL_hist.d0-num_d)),VAL_hist.d0-1);
    X_d = X_hist.subRange(max(ARR(0,(int)X_hist.d0-num_d)),X_hist.d0-1);

    arr PHIc = PHIc_hist.subRange(max(ARR(0,(int)PHIc_hist.d0-num_d)),PHIc_hist.d0-1);
    arr PHIn = PHIn_hist.subRange(max(ARR(0,(int)PHIn_hist.d0-num_d)),PHIn_hist.d0-1);
    arr R = REWARD_hist.subRange(max(ARR(0,(int)REWARD_hist.d0-num_d)),REWARD_hist.d0-1);

    arr oldParam = param;
//    cout << ~PHIc*(PHIc-gamma*PHIn) << endl;
//    cout << C << endl;
    param = inverse(~PHIc*(PHIc-gamma*PHIn) + lambda*eye(PHIc.d1))*~PHIc*R;
//    param = inverse(A+lambda*eye(A.d0))*b;
//    param = inverse(A)*b;
//    param = inverse_SymPosDef(~PHI_d*PHI_d + lambda*eye(PHI_d.d1))*~PHI_d*VAL_d;
    epsParam = sum(fabs(param-oldParam));
    cout << "parameter change: " << epsParam << endl;
  }

  double compReward(const arr &x,uint t) {
    // distance to final position
    world->setJointState(x);
    arr d = ARR(world->getBodyByName("target")->X.pos - world->getShapeByName("endeff")->X.pos);
    double dist = length(d);

    double reward=0.;
    /// reward at final time step only
//    if (t==T-1) {
//      reward = dist;
//    }

    /// reward at each time step
//    reward = exp(-dist);
    reward = -dist;



    /// reward in area close to target
//    reward = 20.; // transition reward
//    if (dist<.01) {
//      reward = -10.01;
//    }
    return reward;
  }

  /// compute value function for reward of a rollout
  void compValue(const arr &reward, arr &value) {
    value.resizeAs(reward).setZero();
    value(T-1) = reward(T-1);
    for (int t =T-2;t>-1;t--) {
      value(t) = reward(t) + gamma*value(t+1);
    }
//    cout << reward << endl;
//    cout << value << endl;
  }

  /// compute features + Jacobian
  void compFeat(const arr &x, arr &phi, arr &Jphi) {
    MP->setState(x);
//    phi.resize(nFeat).setZero();
//    Jphi.resize(nFeat,x.N).setZero();
    phi.clear();
    Jphi.clear();
    for (uint i=0;i<MP->taskCosts.d0;i++) {
      arr phi_i,Jphi_i;
      MP->taskCosts(i)->map.phi(phi_i,Jphi_i,*world);
//      phi = phi + phi_i;
//      Jphi = Jphi + Jphi_i;
      phi.append(phi_i);
      Jphi.append(Jphi_i);
    }
  }

  void runVF() {
    world = new mlr::KinematicWorld("scene0");
    MP = new MotionProblem(*world,false);

    MP->addTask("pos", new DefaultTaskMap(posTMT, *world, "endeff", NoVector));
//    MP->addTask("q", new TaskMap_qItself());
//    MP->addTask("qItself", new DefaultTaskMap(TaskMap_qItself));

    arr x;
    world->getJointState(x);

    // add noise on start state
    x = x + randn(x.d0,1)*x0_noise;
    world->setJointState(x);

    arr W,w,c; // unroll param
    paramToMatrixForm(param,W,w,c);
    cout <<"W: " << W << endl;
    cout <<"w: " << w << endl;
    cout <<"b: " << c << endl;

    arr phi,Jphi;
    arr FEAT;
    arr REWARD,PHI,X;
    REWARD.resize(T).setZero();
    PHI.resize(T,nParam).setZero();
    X.resize(T,x.d0).setZero();

    double alpha = 0.05;
    for (uint t=0;t<T;t++) {
      // compute feature and value fct.
      REWARD(t) = compReward(x,t);
      compFeat(x,phi,Jphi);
      featToMatrixForm(phi,FEAT);
      PHI[t] = ~FEAT;
      X[t] = ~x;

      if (t>0){
        A = A + z*~(PHI[t-1]-PHI[t]);
        b = b + z*REWARD(t-1);
        z = gamma*z + PHI[t];

//        A = A + PHI[t-1]*~(PHI[t-1]-gamma*PHI[t]);
//        b = b + PHI[t-1]*REWARD(t-1);
      } else {
        z = PHI[t];
      }

      // go into steepest direction of value function
      arr dir =2.*(~phi*W*Jphi + ~w*Jphi);

//      cout << "value: " << ~phi*W*phi+~w*phi+c << endl;

      // apply greedy exploration
      if (((double)rand() / RAND_MAX) > eps){
//        dir = dir + randn(dir.d0,1)*dir_noise;
        dir = -dir;
      }
      dir = dir/length(dir);
      x = x + alpha*dir;

//      arr C = eye(4)*1e3;
//      x = x-inverse_SymPosDef(~Jphi*W*Jphi + C)*~Jphi*(0.5*w + W*phi);

      world->setJointState(x);
      if (vis) { world->gl().update();}
    }
    if (vis) { world->gl().~OpenGL();}

    // store value function & features in database
    arr VAL;
    compValue(REWARD,VAL);

    PHI_hist.append(PHI);
    VAL_hist.append(VAL);
    X_hist.append(X);
    PARAM_hist.append(~param);

    PHIc_hist.append(PHI.subRange(0,PHI.d0-2));
    PHIn_hist.append(PHI.subRange(1,PHI.d0-1));
    REWARD_hist.append(REWARD.subRange(0,REWARD.d0-2));

    cout << "SUMMED REWARD: " << sum(REWARD) << endl;
  }
};

void run() {
  /// fix random seed
  rnd.seed(1);

  ValueFctLearning vfl = ValueFctLearning();
  vfl.nFeat = 3;
  vfl.lambda = 1e-3;
  vfl.gamma = 0.5;
  vfl.T = 200;
  vfl.vis = false;
  vfl.Wdiag = false;
  vfl.eps = 1.;
  vfl.dir_noise = 1e1;
  vfl.num_d = 2000;

  uint maxIter = 1000;
  vfl.initParam();

  /// start learning
  uint i =0;
  while ((vfl.epsParam > 1e-5) && (i < maxIter)) {
    vfl.x0_noise = 0.;
    vfl.vis = false;
    vfl.runVF();
    vfl.updateParam();
    i++;
  }

//  vfl.writeToFile();

  for (uint i =0;i<10;i++) {
//    vfl.x0_noise = 1e0;
    vfl.vis = true;
    vfl.runVF();
  }
  return;
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  run();
  return 0;
}
