#include <Core/thread.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <Algo/MLcourse.h>

#include <sensor_msgs/JointState.h>
//#include <pr2/baxter.h>

#include <Motion/komo.h>
#include <Motion/motion.h>

struct Poser{
  ors::KinematicWorld W;
  arr q0;
  Poser(ors::KinematicWorld& W):W(W){
    q0 = W.getJointState();
  }

  arr getPose(arr &qInit){
    arr posR = .3*randn(3);  posR += ARR(.6, -.3, 1.);
    arr posL = .3*randn(3);  posL += ARR(.6,  .3, 1.);
    arr vecR = randn(3); if(vecR(0)<0.) vecR(0) *=-1.;  vecR/=length(vecR);
    arr vecL = randn(3); if(vecL(0)<0.) vecL(0) *=-1.;  vecL/=length(vecL);

//    KOMO komo;
//    komo.setModel(W);
//    komo.setTiming(1, 1, 5., 1, true);
//    komo.setSquaredQVelocities();
//    komo.setCollisions(true);
//    komo.setLimits(true);
//    komo.setPosition(1., 1., "endeffR", NULL, sumOfSqrTT, posR);
//    komo.setPosition(1., 1., "endeffL", NULL, sumOfSqrTT, posL);
//    komo.setAlign(1., 1., "endeffR", ARR(1.,0.,0.), NULL, vecR, sumOfSqrTT, {1.});
//    komo.setAlign(1., 1., "endeffL", ARR(1.,0.,0.), NULL, vecL, sumOfSqrTT, {1.});
//    komo.reset();
//    komo.run();

//    Graph result = komo.getReport();
////    cout <<result <<endl;
//    double cost = result.get<double>({"total","sqrCosts"});
//    double constraints = result.get<double>({"total","constraints"});

//    if(constraints<.1 && cost<5.){
//      komo.x.refRange(0,2)=0.;
//      W.setJointState(komo.x);
//    }else{
//      return getPose(qInit);
//    }
//    W.watch(false);
//    arr qT = komo.x;

    MotionProblem MP(W);
    Task *t;
    MP.world.setJointState(qInit);

    t = MP.addTask("transitions", new TaskMap_Transition(W), sumOfSqrTT);
    t->map.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

    t = MP.addTask("endeffR", new TaskMap_Default(posTMT,W,"endeffR"), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, posR, 1e2);
    t = MP.addTask("endeffL", new TaskMap_Default(posTMT,W,"endeffL"), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, posL, 1e2);
    t = MP.addTask("alignL", new TaskMap_Default(vecAlignTMT,W,"endeffL",ors::Vector(1.,0.,0.),NULL,ors::Vector(vecL)), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, {1.}, 1e1);
    t = MP.addTask("alignR", new TaskMap_Default(vecAlignTMT,W,"endeffR",ors::Vector(1.,0.,0.),NULL,ors::Vector(vecR)), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, {1.}, 1e1);

    t = MP.addTask("collisionConstraints", new CollisionConstraint(.1), ineqTT);
    t->setCostSpecs(0, MP.T, {0.}, 1.);
    t = MP.addTask("limits", new LimitsConstraint(0.05), ineqTT);
    t->setCostSpecs(0, MP.T, {0.}, 1.);

    // sample a head joint
    arr lim = W.getLimits();
    uint qIdx = W.getJointByName("head_pan_joint")->qIndex;
    double qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
    t = MP.addTask("head1", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);
    qIdx = W.getJointByName("head_tilt_joint")->qIndex;
    qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
    t = MP.addTask("head2", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);

    //-- create the Optimization problem (of type kOrderMarkov)
    arr x = MP.getInitialization(); //replicate(MP.x0, MP.T+1);
    x.reshape(MP.T,W.getJointStateDimension());

    //-- optimize
    optConstrained(x, NoArr, Convert(MP), OPT(verbose=1, stopIters=100, damping=1., maxStep=1.,aulaMuInc=2, nonStrictSteps=5));
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
    MP.costReport();

    Graph result = MP.getReport();
    double cost = result.get<double>({"total","sqrCosts"});
    double constraints = result.get<double>({"total","constraints"});

    if(constraints<.1 && cost<5.){
      W.setJointState(x[x.d0-1]);
      W.gl().update();
      gnuplot("load 'z.costReport.plt'", false, true);
//      displayTrajectory(x, 1, W, "planned trajectory", 0.01);
      return x;
    }else{
      return getPose(qInit);
    }
  }
};


// =================================================================================================
void recordData() {
  rosCheckInit("gamepadControl");

  Access_typed<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
  Access_typed<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
  Access_typed<arr>     pr2_odom(NULL, "pr2_odom");
  Access_typed<bool> fixBase(NULL, "fixBase");


  Access_typed<arr> q_ref(NULL, "q_ref");
  Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

  TaskControllerModule tcm;
  OrsViewer view;
  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  if(mlr::getParameter<bool>("useRos")){
    mlr::String robot = mlr::getParameter<mlr::String>("robot", "pr2");
    if(robot=="pr2"){
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> ("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          ("/marc_rt_controller/jointReference", ctrl_ref);
      new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       ("/robot_pose_ekf/odom_combined", pr2_odom);
    }
    if(robot=="baxter"){
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
    }
  }

  rnd.clockSeed();
  Poser pose(tcm.realWorld);
  threadOpenModules(true);
  fixBase.set() = true;
  mlr::wait(1.);
  cout <<"NOW" <<endl;

  arr q0 = tcm.modelWorld.get()->q;
  TaskMap_qItself map;
  CtrlTask task("qItself", &map, 1., 1., 1., 10.);
  task.setTarget(q0);
  tcm.ctrlTasks.set() = { &task };

  arr Q,U;
  arr q;
  arr qTraj;
  for(uint j=0;j<1000;j++){
    // execute a trajectory
    tcm.realWorld.getJointState(q);
    qTraj = pose.getPose(q);
    double duration = 4.;
    for (uint t=0;t<qTraj.d0;t++) {
      tcm.ctrlTasks.writeAccess();
      task.setTarget(qTraj[t]);
      tcm.ctrlTasks.deAccess();
      mlr::wait(duration/(double)qTraj.d0);
    }

    mlr::wait(4.);
    CtrlMsg obs;
    // read out state and save to file
    if (mlr::getParameter<bool>("useRos") == true) {
      arr Qi,Ui;
      for (uint i=0;i<10;i++) {

        obs = ctrl_obs.get();
        Qi.append(~obs.q);
        Ui.append(~obs.u_bias);
        mlr::wait(0.1);
      }
      Qi = sum(Qi,0)/(double)Qi.d0;
      Ui = sum(Ui,0)/(double)Ui.d0;
      Q.append(~Qi);
      U.append(~Ui);

      write(LIST<arr>(Q),STRING("Q.dat"));
      write(LIST<arr>(U),STRING("U.dat"));
    }
  }

  //  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
}


// =================================================================================================
void learnModel() {
  arr X,Phi,Y,Z;
  arr beta;

  //provide virtual train and test routines for CV
  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      beta = ridgeRegression(X, y, param); //returns optimal beta for training data
    };
    double test(const arr& X, const arr& y, const arr& beta) {
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N; //returns MSE on test data
    };
  } cv;

  // define which joints to learn [l or r]
  String arm = "r";
  StringA joints = {"_elbow_flex_joint","_wrist_roll_joint","_wrist_flex_joint","_forearm_roll_joint",
                    "_upper_arm_roll_joint","_shoulder_lift_joint","_shoulder_pan_joint"};
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);

  // load the data, split in input and output
  Z <<FILE("data/Q1.dat");
  Y <<FILE("data/U1.dat");

  // select the relevant joints
  arr T = zeros(joints.N,Z.d1);
  for (uint i=0;i<joints.N;i++) {
    T(i,world.getJointByName(STRING(arm<<joints(i)))->qIndex) = 1;
  }
  X = Z*~T;

  Phi = makeFeatures(X,quadraticFT);
  // add sin/cos features
  Phi = catCol(Phi,sin(X));
  Phi = catCol(Phi,cos(X));

  // add dynamics features
  arr Phi_tmp;
  arr phi_t;
  for (uint t = 0; t<Z.d0; t++) {
    phi_t.clear();
    world.setJointState(Z[t],Z[t]*0.);

    arr M,F;
    world.equationOfMotion(M,F);
    phi_t.append(T*F);

    Phi_tmp.append(~phi_t);
  }
  Phi = catCol(Phi,Phi_tmp);



  for (uint i=0;i<joints.N;i++) {
    arr y;
    y = Y.col(world.getJointByName(STRING(arm<<joints(i)))->qIndex);
    //cross valide
    cv.crossValidateMultipleLambdas(Phi, y,
                                    ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5),
                                    10, false);
    cv.plot();
    cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;
  }
}
// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  recordData();
  //  learnModel();

  return 0;
}
