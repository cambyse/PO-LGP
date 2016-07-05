#include <Core/module.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>

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
    komo.setLimits(true);
    komo.setPosition(1., 1., "endeffR", NULL, sumOfSqrTT, posR);
    komo.setPosition(1., 1., "endeffL", NULL, sumOfSqrTT, posL);
    komo.setAlign(1., 1., "endeffR", ARR(1.,0.,0.), NULL, vecR, sumOfSqrTT, {1.});
    komo.setAlign(1., 1., "endeffL", ARR(1.,0.,0.), NULL, vecL, sumOfSqrTT, {1.});
    komo.reset();
    komo.run();

    Graph result = komo.getReport();
    cout <<result <<endl;
    double cost = result.get<double>({"total","sqrCosts"});
    double constraints = result.get<double>({"total","constraints"});

    if(constraints<.1 && cost<5.){
      komo.x.refRange(0,2)=0.;
      W.setJointState(komo.x);
    }else{
      return getPose();
    }
    W.watch(false);
    return komo.x;
  }
};


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);



  rosCheckInit("gamepadControl");

  Access_typed<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
  Access_typed<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
  Access_typed<arr>     pr2_odom(NULL, "pr2_odom");

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

  Poser pose(tcm.realWorld);

  threadOpenModules(true);

  mlr::wait(1.);
  cout <<"NOW" <<endl;

  arr q0 = tcm.modelWorld.get()->q;
  TaskMap_qItself map;
  CtrlTask task("qItself", &map, 1., 1., 1., 10.);
  tcm.ctrlTasks.set() = { &task };

  for(uint i=0;i<10;i++){
    q0 = pose.getPose();

    tcm.ctrlTasks.writeAccess();
    task.setTarget(q0);
    tcm.ctrlTasks.deAccess();

    mlr::wait(6.);
  }


  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  cout <<"bye bye" <<endl;
  return 0;
}
