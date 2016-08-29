#include <Core/module.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <Algo/MLcourse.h>

#include <Control/gravityCompensation.h>

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

  arr getPose(){
    arr posR = .3*randn(3);  posR += ARR(.6, -.3, 1.);
    arr posL = .3*randn(3);  posL += ARR(.6,  .3, 1.);
    arr vecR = randn(3); if(vecR(0)<0.) vecR(0) *=-1.;  vecR/=length(vecR);
    arr vecL = randn(3); if(vecL(0)<0.) vecL(0) *=-1.;  vecL/=length(vecL);

    W.setJointState(q0);

    KOMO komo;
    komo.setModel(W);
    komo.setTiming(1, 1, 5., 1, true);
    komo.setSquaredQVelocities();
    komo.setCollisions(true);
    komo.setLimits(true, 0.1);
    komo.setPosition(1., 1., "endeffR", NULL, sumOfSqrTT, posR);
    komo.setPosition(1., 1., "endeffL", NULL, sumOfSqrTT, posL);
    komo.setAlign(1., 1., "endeffR", ARR(1.,0.,0.), NULL, vecR, sumOfSqrTT, {1.});
    komo.setAlign(1., 1., "endeffL", ARR(1.,0.,0.), NULL, vecL, sumOfSqrTT, {1.});

    arr lim = W.getLimits();
    uint qIdx = W.getJointByName("head_tilt_joint")->qIndex;
    double qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
    Task* t = komo.MP->addTask("headTilt", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
    t->setCostSpecs(komo.MP->T-1, komo.MP->T, ARR(qh), 1.);

    komo.reset();
    komo.run();

    Graph result = komo.getReport();
    //    cout <<result <<endl;
    double cost = result.get<double>({"total","sqrCosts"});
    double constraints = result.get<double>({"total","constraints"});

    if(constraints<.1 && cost<5.){
      komo.x.refRange(0,2)=0.;
      W.setJointState(komo.x);
      W.watch(false);
      return komo.x;
    }else{
      return getPose();
    }
  }

  arr getTraj(arr qInit, arr qEnd) {
    MotionProblem MP(W);
    Task *t;
    MP.world.setJointState(qInit);

    t = MP.addTask("transitions", new TaskMap_Transition(W), sumOfSqrTT);
    t->map.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

    t = MP.addTask("q", new TaskMap_qItself, sumOfSqrTT);
    t->setCostSpecs(MP.T-1, MP.T, qEnd, 1e2);

    t = MP.addTask("collisionConstraints", new CollisionConstraint(.1), ineqTT);
    t->setCostSpecs(0, MP.T, {0.}, 1.);
    t = MP.addTask("limits", new LimitsConstraint(0.1), ineqTT);
    t->setCostSpecs(10, MP.T, {0.}, 1.);

    //    // sample a head joint
    //    arr lim = W.getLimits();
    //    uint qIdx = W.getJointByName("head_pan_joint")->qIndex;
    //    double qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
    //    t = MP.addTask("head1", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
    //    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);
    //    qIdx = W.getJointByName("head_tilt_joint")->qIndex;
    //    qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
    //    t = MP.addTask("head2", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
    //    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);

    //-- create the Optimization problem (of type kOrderMarkov)
    arr x = MP.getInitialization(); //replicate(MP.x0, MP.T+1);
    x.reshape(MP.T,W.getJointStateDimension());

    //-- optimize
    optConstrained(x, NoArr, Convert(MP), OPT(verbose=1, stopIters=100, stopTolerance=1e-3, damping=1., maxStep=1.,aulaMuInc=2, nonStrictSteps=5));
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
    MP.costReport();

    Graph result = MP.getReport();
    double cost = result.get<double>({"total","sqrCosts"});
    double constraints = result.get<double>({"total","constraints"});

    W.setJointState(x[x.d0-1]);
    TaskMap_qLimits limits;
    arr y;
    limits.phi(y,NoArr,W);
    cout <<"Limits: " <<y<< endl;
    //displayTrajectory(x, 1, W, "planned trajectory", 0.01);
    if(constraints<.1 && cost<5.){
      W.setJointState(x[x.d0-1]);
      W.gl().update();
      gnuplot("load 'z.costReport.plt'", false, true);
      //      displayTrajectory(x, 1, W, "planned trajectory", 0.01);
      return x;
    } else {
      return getTraj(qInit, getPose());
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

  arr Q,U,FR,FL,UMeasured,QRef,QRefMinusOne;
  arr q;
  arr qTraj;
  for(uint j=0;j<1000;j++){
    // execute a trajectory
    tcm.realWorld.getJointState(q);
    qTraj = pose.getTraj(q, pose.getPose());
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
      arr Qi,Ui,FLi,FRi,UMeasuredi;
      for (uint i=0;i<10;i++) {
        obs = ctrl_obs.get();
        Qi.append(~obs.q);
        Ui.append(~obs.u_bias);
        UMeasuredi.append(~obs.Kp);
        FLi.append(~obs.fL);
        FRi.append(~obs.fR);
        mlr::wait(0.1);
      }
      Qi = sum(Qi,0)/(double)Qi.d0;
      Ui = sum(Ui,0)/(double)Ui.d0;
      UMeasuredi = sum(UMeasuredi,0)/(double)UMeasuredi.d0;
      FLi = sum(FLi,0)/(double)FLi.d0;
      FRi = sum(FRi,0)/(double)FRi.d0;
      Q.append(~Qi);
      U.append(~Ui);
      UMeasured.append(~UMeasuredi);
      FL.append(~FLi);
      FR.append(~FRi);
      QRef.append(~qTraj[qTraj.d0-1]);
      QRefMinusOne.append(~qTraj[qTraj.d0-2]);

      write(LIST<arr>(Q),STRING("Q.dat"));
      write(LIST<arr>(U),STRING("U.dat"));
      write(LIST<arr>(UMeasured),STRING("UMeasured.dat"));
      write(LIST<arr>(FL),STRING("FL.dat"));
      write(LIST<arr>(FR),STRING("FR.dat"));
      write(LIST<arr>(QRef),STRING("QRef.dat"));
      write(LIST<arr>(QRefMinusOne),STRING("QRefMinusOne.dat"));
    }
  }

  //  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

void testCorrPlot() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  GravityCompensation gc(world);

  arr Q;
  Q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/Q.dat"));
  arr U;
  U << FILE(mlr::mlrPath("examples/pr2/calibrateControl/U.dat"));

  gc.learnModels(false);

  uint index = world.getJointByName("l_upper_arm_roll_joint")->qIndex;

  arr UPred;

  for(uint i = 0; i < Q.d0; i++) {
    world.setJointState(Q[i]);
    UPred.append(gc.compensate(world.getJointState(), true, false, false)(index));
  }
  U = U.sub(0,-1,index,index);
  FILE("uPred.dat") << UPred;
  FILE("u.dat") << U;
}

void testModel() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  GravityCompensation gc(world);

  gc.learnModels(false); //set to false if you dont want to see plots

  cout << gc.compensate(world.getJointState(), true, true, true) << endl;

  gc.saveBetas();
  gc.loadBetas();
  cout << gc.compensate(world.getJointState(), true, true, true) << endl;


  //mlr::String h = "l_shoulder_lift_joint";

  //mlr::String n = "l_wrist_flex_joint";

  //uint index = world.getJointByName(h)->qIndex;
  //arr q = world.getJointState();

  //arr limits = world.getJointByName(h)->limits;
  //arr Q = linspace(limits(0),limits(1),100);

  /*
  arr Q;
  Q = grid(ARR(world.getJointByName(h)->limits(0),world.getJointByName(n)->limits(0)),ARR(world.getJointByName(h)->limits(1),world.getJointByName(n)->limits(1)),{100,100});
  FILE("testQ.dat") << Q;
  q(17) += 0.5;
  q(21) += 0.5;
  arr U;
  for(uint i = 0; i < Q.d0; i++) {
    q(index) = Q(i,0);
    q(world.getJointByName(n)->qIndex) = Q(i,1);
    world.setJointState(q);
    U.append(gc.compensate(world.getJointState(), {h})(index));
    //cout << gc.compensate(world.getJointState(), {"l_shoulder_lift_joint"}) << endl;
  }

  FILE("testU.dat") << U;

  cout << gc.compensate(world.getJointState(), {"l_shoulder_lift_joint"}) << endl;
*/

  //gc.testForLimits();
}


void testHead() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  GravityCompensation gc(world);

  gc.learnModels(false);

  arr q = linspace(-0.5,1.4,100);
  q.reshapeFlat();

  arr U;

  for(uint i = 0; i < q.N; i++) {
    arr qAct = world.getJointState();
    qAct(world.getJointByName("head_tilt_joint")->qIndex) = q(i);
    arr u = gc.compensate(qAct,false,false,true);
    cout << u << endl;
    U.append(ARR(u(world.getJointByName("head_tilt_joint")->qIndex)));
  }

  FILE("uHeadPred.dat") << U;
  FILE("qHead.dat") << q;
}

void testHeadRealRobot() {
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

  //tcm.compensateGravity = true;

  threadOpenModules(true);
  fixBase.set() = true;
  mlr::wait(1.);
  cout <<"NOW" <<endl;

  arr q0 = tcm.modelWorld.get()->q;
  TaskMap_qItself map;
  CtrlTask task("qItself", &map, 1., 1., 1., 10.);
  task.setTarget(q0);
  tcm.ctrlTasks.set() = { &task };

  //uint index = world.getJointByName("head_tilt_joint")->qIndex;

  /*arr q = q0;

  for(uint i = 0; i < 1; i++) {
    q(index) = q(index);
    tcm.ctrlTasks.writeAccess();
    task.setTarget(q);
    tcm.ctrlTasks.deAccess();
    mlr::wait(10.0);
  }*/

  mlr::wait(10.0);

  CtrlMsg obs;
  CtrlMsg ref;

  obs = ctrl_obs.get();
  ref = ctrl_ref.get();

  FILE("uGravityCompensation") << ref.u_bias;
  FILE("uEffort") << obs.u_bias;
  cout << "saved" << endl;
  mlr::wait(100.0);

  //tcm.modelWorld.get()->watch(true,"press to stop");

  //  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

void testLimitsWorld() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  cout << world.getJointByName("l_shoulder_lift_joint")->limits << endl;
}

void testCorr() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  GravityCompensation gc(world);
  arr Q;
  Q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/Q.dat"));
  arr U;
  U << FILE(mlr::mlrPath("examples/pr2/calibrateControl/U.dat"));
  gc.generatePredictionsOnDataSet(Q,U,gc.rightJoints);
}


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //recordData();
  testModel();
  //testCorrPlot();
  //testHead();
  //testHeadRealRobot();
  //testLimitsWorld();
  //testCorr();
  return 0;
}
